#include "radar_telemetry_queue.h"

/* S3 遥测有界队列实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <math.h>
#include <string.h>

#include "srp_codec.h"
#include "srp_wire.h"

enum {
    RADAR_TELEMETRY_OBSERVATION_CHASSIS = 0,
    RADAR_TELEMETRY_OBSERVATION_ATTITUDE = 1,
    RADAR_TELEMETRY_OBSERVATION_IMU_LSM303 = 2,
    RADAR_TELEMETRY_OBSERVATION_IMU_BMI323 = 3,
    RADAR_TELEMETRY_OBSERVATION_COUNT = 4
};

/**
 * @brief  将遥测队列分类映射到对应的可变统计计数器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  queue 队列实例；可为 NULL。
 * @param  stream wheel、chassis、attitude、LSM303 或 BMI323 分类。
 * @return 有效分类时返回 queue 内部统计对象的借用指针；参数或分类无效时返回 NULL。
 * 调用方式：push 路径完成 SRPv4 分类后调用，用于更新 accepted/dropped/overwritten。
 * 线程约束：不加锁、不阻塞；返回指针只在 queue 生命周期内有效，访问必须受上层同一 mutex 保护。
 */
static radar_telemetry_queue_stream_stats_t *stream_stats(
    radar_telemetry_queue_t *queue,
    radar_telemetry_queue_class_t stream)
{
    if (queue == NULL) {
        return NULL;
    }

    switch (stream) {
    case RADAR_TELEMETRY_QUEUE_CLASS_WHEEL:
        return &queue->stats.wheel;
    case RADAR_TELEMETRY_QUEUE_CLASS_CHASSIS:
        return &queue->stats.chassis;
    case RADAR_TELEMETRY_QUEUE_CLASS_ATTITUDE:
        return &queue->stats.attitude;
    case RADAR_TELEMETRY_QUEUE_CLASS_IMU_LSM303:
        return &queue->stats.imu_lsm303;
    case RADAR_TELEMETRY_QUEUE_CLASS_IMU_BMI323:
        return &queue->stats.imu_bmi323;
    default:
        return NULL;
    }
}

/**
 * @brief  根据待发槽位重算总深度和各流深度，并更新历史高水位。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] queue 可为 NULL；非 NULL 时更新 stats.depth/high_watermark 等字段。
 * @return 无。
 * 调用方式：队列初始化以及每次成功 push/pop 后调用，保持统计与槽位状态同步。
 * 线程约束：纯内存更新、无内部锁；只能在持有队列外层 mutex 或单线程初始化时调用。
 */
static void refresh_depth_stats(radar_telemetry_queue_t *queue)
{
    size_t observation_depth = 0U;

    if (queue == NULL) {
        return;
    }

    for (size_t index = 0U; index < RADAR_TELEMETRY_OBSERVATION_COUNT;
         ++index) {
        if ((index == RADAR_TELEMETRY_OBSERVATION_CHASSIS &&
             queue->chassis_pending) ||
            (index == RADAR_TELEMETRY_OBSERVATION_ATTITUDE &&
             queue->attitude_pending) ||
            (index == RADAR_TELEMETRY_OBSERVATION_IMU_LSM303 &&
             queue->imu_pending[0]) ||
            (index == RADAR_TELEMETRY_OBSERVATION_IMU_BMI323 &&
             queue->imu_pending[1])) {
            ++observation_depth;
        }
    }

    queue->stats.wheel.depth = queue->wheel_count;
    queue->stats.chassis.depth = queue->chassis_pending ? 1U : 0U;
    queue->stats.attitude.depth = queue->attitude_pending ? 1U : 0U;
    queue->stats.imu_lsm303.depth = queue->imu_pending[0] ? 1U : 0U;
    queue->stats.imu_bmi323.depth = queue->imu_pending[1] ? 1U : 0U;
    queue->stats.depth = queue->wheel_count + observation_depth;

    if (queue->stats.wheel.depth > queue->stats.wheel.high_watermark) {
        queue->stats.wheel.high_watermark = queue->stats.wheel.depth;
    }
    if (queue->stats.chassis.depth > queue->stats.chassis.high_watermark) {
        queue->stats.chassis.high_watermark = queue->stats.chassis.depth;
    }
    if (queue->stats.attitude.depth > queue->stats.attitude.high_watermark) {
        queue->stats.attitude.high_watermark = queue->stats.attitude.depth;
    }
    if (queue->stats.imu_lsm303.depth >
        queue->stats.imu_lsm303.high_watermark) {
        queue->stats.imu_lsm303.high_watermark =
            queue->stats.imu_lsm303.depth;
    }
    if (queue->stats.imu_bmi323.depth >
        queue->stats.imu_bmi323.high_watermark) {
        queue->stats.imu_bmi323.high_watermark =
            queue->stats.imu_bmi323.depth;
    }
    if (queue->stats.depth > queue->stats.high_watermark) {
        queue->stats.high_watermark = queue->stats.depth;
    }
}

/**
 * @brief  检查小端字节缓冲指定区间内的 float 数组是否全部为有限值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  data 非 NULL 的 payload 起始地址。
 * @param  offset 第一个 float 相对 data 的字节偏移。
 * @param  count 连续 float 元素数量。
 * @return 所有元素均非 NaN/Inf 时为 true；data 为空或任一元素非有限时为 false。
 * 调用方式：validate_payload() 已确认消息精确长度后调用；本函数自身不接收容量参数也不做越界检查。
 * 线程约束：只读、可重入、不阻塞；浮点解码循环不应在 ISR 中执行。
 */
static bool finite_f32_array(const uint8_t *data,
                             size_t offset,
                             size_t count)
{
    if (data == NULL) {
        return false;
    }

    for (size_t index = 0U; index < count; ++index) {
        if (!isfinite(srp_wire_read_f32_le(
                &data[offset + index * sizeof(float)]))) {
            return false;
        }
    }
    return true;
}

/**
 * @brief  按 SRPv4 消息类型验证遥测 payload 长度、schema、保留位和有限浮点值。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  frame 已由 srp_decode() 解码的借用视图，frame/payload 均须非 NULL。
 * @param  message_id 仅支持 WHEEL_SPEED_STATUS、CHASSIS_STATE、ATTITUDE 或 IMU_TELEMETRY。
 * @param[out] stream 非 NULL；成功时写入对应 wheel/chassis/attitude/两路 IMU 队列分类。
 * @return payload 完整满足对应契约时为 true，否则为 false；失败时 stream 不保证被改写。
 * 调用方式：decode_and_validate() 完成外层 type/flags/priority 检查后调用，不复制 payload。
 * 线程约束：只读、可重入、不阻塞；包含浮点扫描，禁止 ISR 调用。
 */
static bool validate_payload(const srp_frame_t *frame,
                             uint16_t message_id,
                             radar_telemetry_queue_class_t *stream)
{
    if (frame == NULL || frame->payload == NULL || stream == NULL) {
        return false;
    }

    switch (message_id) {
    case SRP_MSG_ID_WHEEL_SPEED_STATUS:
        if (frame->length != SRP_PAYLOAD_WHEEL_SPEED_STATUS_SIZE ||
            !finite_f32_array(frame->payload, 0U, 4U)) {
            return false;
        }
        *stream = RADAR_TELEMETRY_QUEUE_CLASS_WHEEL;
        return true;

    case SRP_MSG_ID_CHASSIS_STATE:
        if (frame->length != SRP_PAYLOAD_CHASSIS_STATE_SIZE ||
            frame->payload[0] != SRP_CHASSIS_STATE_SCHEMA ||
            (frame->payload[1] &
             (uint8_t)~SRP_CHASSIS_STATE_FLAGS_MASK) != 0U ||
            frame->payload[2] != 0U || frame->payload[3] != 0U ||
            !finite_f32_array(frame->payload, 8U, 4U) ||
            srp_wire_read_f32_le(&frame->payload[20]) < 0.0f) {
            return false;
        }
        *stream = RADAR_TELEMETRY_QUEUE_CLASS_CHASSIS;
        return true;

    case SRP_MSG_ID_ATTITUDE:
        if (frame->length != SRP_PAYLOAD_DUAL_AHRS_SIZE ||
            frame->payload[0] != SRP_DUAL_AHRS_SCHEMA ||
            frame->payload[2] != 0U || frame->payload[3] != 0U ||
            !finite_f32_array(frame->payload, 12U, 17U)) {
            return false;
        }
        *stream = RADAR_TELEMETRY_QUEUE_CLASS_ATTITUDE;
        return true;

    case SRP_MSG_ID_IMU_TELEMETRY:
        if (frame->length != SRP_PAYLOAD_IMU_TELEMETRY_SIZE ||
            !finite_f32_array(frame->payload, 6U, 6U)) {
            return false;
        }
        if (frame->payload[0] == SRP_IMU_SENSOR_LSM303) {
            *stream = RADAR_TELEMETRY_QUEUE_CLASS_IMU_LSM303;
            return true;
        }
        if (frame->payload[0] == SRP_IMU_SENSOR_BMI323) {
            *stream = RADAR_TELEMETRY_QUEUE_CLASS_IMU_BMI323;
            return true;
        }
        return false;

    default:
        return false;
    }
}

/**
 * @brief  解码完整 SRPv4 线缆帧并验证消息元数据和遥测 payload 契约。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  encoded_frame 非 NULL 的完整 SRPv4 线缆帧。
 * @param  encoded_length 帧总长度，须在 SRP 最小帧长与遥测队列最大帧长之间。
 * @param  message_id 期望的 SRPv4 type，必须与解码结果一致。
 * @param[out] decoded 非 NULL；成功时 payload 借用 encoded_frame 内存，失败时不得使用。
 * @param[out] stream 非 NULL；成功时写入队列分类。
 * @return CRC/边界、type、flags、priority 和 payload 全部合法时为 true，否则为 false。
 * 调用方式：radar_telemetry_queue_push_ex() 在复制入槽前调用；源帧在本函数返回前必须保持有效。
 * 线程约束：只读解码、可重入但计算有界；禁止 ISR 调用，decoded 借用指针不得跨源缓冲生命周期。
 */
static bool decode_and_validate(const uint8_t *encoded_frame,
                                size_t encoded_length,
                                uint16_t message_id,
                                srp_frame_t *decoded,
                                radar_telemetry_queue_class_t *stream)
{
    if (encoded_frame == NULL || decoded == NULL || stream == NULL ||
        encoded_length < (size_t)SRP_HEADER_SIZE + SRP_TRAILER_SIZE ||
        encoded_length > RADAR_TELEMETRY_QUEUE_MAX_FRAME_SIZE ||
        srp_decode(encoded_frame, encoded_length, decoded) != 0) {
        return false;
    }

    if (decoded->type != message_id || decoded->flags != SRP_FLAG_STREAM_DATA ||
        (message_id == SRP_MSG_ID_ATTITUDE
             ? (decoded->priority != SRP_PRIORITY_COMMAND &&
                decoded->priority != SRP_PRIORITY_TELEMETRY)
             : decoded->priority != SRP_PRIORITY_TELEMETRY)) {
        return false;
    }

    return validate_payload(decoded, message_id, stream);
}

/**
 * @brief  将 observation 索引映射到底盘状态、姿态或两路 IMU 的 latest-only 存储槽。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  queue 已绑定外部存储的队列；可为 NULL。
 * @param  observation CHASSIS、ATTITUDE、IMU_LSM303 或 IMU_BMI323 内部索引。
 * @return 有效时返回外部 entry 存储的借用可写指针；状态/索引无效时返回 NULL。
 * 调用方式：observation push/pop 路径选择固定槽位；函数不检查 pending 状态。
 * 线程约束：无锁、不阻塞；返回指针由 queue 的外部存储拥有，必须在同一外层 mutex 内使用。
 */
static radar_telemetry_entry_t *observation_entry(
    radar_telemetry_queue_t *queue,
    unsigned int observation)
{
    if (queue == NULL || queue->storage.chassis_entry == NULL ||
        queue->storage.attitude_entry == NULL ||
        queue->storage.imu_entries == NULL) {
        return NULL;
    }

    if (observation == RADAR_TELEMETRY_OBSERVATION_CHASSIS) {
        return queue->storage.chassis_entry;
    }
    if (observation == RADAR_TELEMETRY_OBSERVATION_ATTITUDE) {
        return queue->storage.attitude_entry;
    }
    if (observation == RADAR_TELEMETRY_OBSERVATION_IMU_LSM303) {
        return &queue->storage.imu_entries[0];
    }
    if (observation == RADAR_TELEMETRY_OBSERVATION_IMU_BMI323) {
        return &queue->storage.imu_entries[1];
    }
    return NULL;
}

/**
 * @brief  查询指定底盘状态、姿态或 IMU observation 槽是否含待发送数据。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  queue 队列实例；可为 NULL。
 * @param  observation CHASSIS、ATTITUDE、IMU_LSM303 或 IMU_BMI323 内部索引。
 * @return 对应 pending 标志为 true 时返回 true；队列/索引无效时返回 false。
 * 调用方式：push 用于统计覆盖，pop 用于公平轮转选择；结果只是瞬时状态。
 * 线程约束：无锁、不阻塞；必须在持有队列外层 mutex 或单线程访问时调用。
 */
static bool observation_pending(const radar_telemetry_queue_t *queue,
                                unsigned int observation)
{
    if (queue == NULL) {
        return false;
    }
    if (observation == RADAR_TELEMETRY_OBSERVATION_CHASSIS) {
        return queue->chassis_pending;
    }
    if (observation == RADAR_TELEMETRY_OBSERVATION_ATTITUDE) {
        return queue->attitude_pending;
    }
    if (observation == RADAR_TELEMETRY_OBSERVATION_IMU_LSM303) {
        return queue->imu_pending[0];
    }
    if (observation == RADAR_TELEMETRY_OBSERVATION_IMU_BMI323) {
        return queue->imu_pending[1];
    }
    return false;
}

/**
 * @brief  清除指定底盘状态、姿态或 IMU observation 槽的待发送标志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] queue 队列实例；可为 NULL。
 * @param  observation CHASSIS、ATTITUDE、IMU_LSM303 或 IMU_BMI323 内部索引；未知值不动作。
 * @return 无。
 * 调用方式：pop_observation() 完成 entry 复制后调用，表示该 latest-only 样本已消费。
 * 线程约束：修改 pending 标志且无内部锁；必须在同一队列外层 mutex 内调用。
 */
static void clear_observation_pending(radar_telemetry_queue_t *queue,
                                       unsigned int observation)
{
    if (queue == NULL) {
        return;
    }
    if (observation == RADAR_TELEMETRY_OBSERVATION_CHASSIS) {
        queue->chassis_pending = false;
    } else if (observation == RADAR_TELEMETRY_OBSERVATION_ATTITUDE) {
        queue->attitude_pending = false;
    } else if (observation == RADAR_TELEMETRY_OBSERVATION_IMU_LSM303) {
        queue->imu_pending[0] = false;
    } else if (observation == RADAR_TELEMETRY_OBSERVATION_IMU_BMI323) {
        queue->imu_pending[1] = false;
    }
}

/**
 * @brief  按轮转游标弹出一个待发底盘状态、姿态或 IMU latest-only 样本。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] queue 非 NULL 的已初始化队列；函数更新 pending、轮转游标和深度统计。
 * @param[out] entry 非 NULL；成功时复制完整队列项。
 * @return 成功找到并消费一个 observation 时为 true；参数错误、无待发项或存储异常时为 false。
 * 调用方式：radar_telemetry_queue_pop() 在 wheel burst 达到上限或 wheel 为空时调用。
 * 线程约束：复制完整 SRPv4 帧且无内部锁；必须持有外层 mutex，禁止 ISR 调用。
 */
static bool pop_observation(radar_telemetry_queue_t *queue,
                            radar_telemetry_entry_t *entry)
{
    if (queue == NULL || entry == NULL) {
        return false;
    }

    for (unsigned int offset = 0U;
         offset < RADAR_TELEMETRY_OBSERVATION_COUNT;
         ++offset) {
        const unsigned int observation =
            (unsigned int)((queue->next_observation + offset) %
                           RADAR_TELEMETRY_OBSERVATION_COUNT);
        if (!observation_pending(queue, observation)) {
            continue;
        }

        radar_telemetry_entry_t *source = observation_entry(queue, observation);
        if (source == NULL) {
            return false;
        }
        *entry = *source;
        clear_observation_pending(queue, observation);
        queue->next_observation = (uint8_t)((observation + 1U) %
                                            RADAR_TELEMETRY_OBSERVATION_COUNT);
        queue->wheel_burst = 0U;
        refresh_depth_stats(queue);
        return true;
    }
    return false;
}

/**
 * @brief  从 wheel 有界 FIFO 复制并消费最旧一项。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] queue 非 NULL 的已初始化队列；须有有效 wheel 存储和非零 count。
 * @param[out] entry 非 NULL；成功时复制完整队列项。
 * @return 成功弹出为 true；参数、存储或空队列状态无效时为 false。
 * 调用方式：radar_telemetry_queue_pop() 的 wheel 优先路径调用，并递增有饱和值的 wheel_burst。
 * 线程约束：复制完整 SRPv4 帧且无内部锁；必须持有外层 mutex，禁止 ISR 调用。
 */
static bool pop_wheel(radar_telemetry_queue_t *queue,
                      radar_telemetry_entry_t *entry)
{
    if (queue == NULL || entry == NULL || queue->wheel_count == 0U ||
        queue->storage.wheel_entries == NULL ||
        queue->storage.wheel_capacity == 0U) {
        return false;
    }

    *entry = queue->storage.wheel_entries[queue->wheel_tail];
    queue->wheel_tail = (queue->wheel_tail + 1U) %
                        queue->storage.wheel_capacity;
    --queue->wheel_count;
    if (queue->wheel_burst < UINT8_MAX) {
        ++queue->wheel_burst;
    }
    refresh_depth_stats(queue);
    return true;
}

/**
 * @brief  校验调用方存储并建立各遥测流槽位和公平调度初始状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[out] queue 非 NULL；成功时整体清零并复制 storage 的指针/容量描述。
 * @param  storage 非 NULL；wheel_entries、chassis_entry、attitude_entry、imu_entries 均有效，
 *                 wheel_capacity 大于 0，且 imu_entries 至少包含 LSM303/BMI323 两项。
 * @return 存储绑定成功为 true；任一要求不满足时返回 false，函数不分配或清零 entry 存储。
 * 调用方式：上行任务创建前调用一次；所有外部存储生命周期必须覆盖 queue 使用期。
 * 线程约束：纯内存初始化、无 RTOS 原语；初始化期间禁止其他上下文访问 queue。
 */
bool radar_telemetry_queue_init(
    radar_telemetry_queue_t *queue,
    const radar_telemetry_queue_storage_t *storage)
{
    if (queue == NULL || storage == NULL || storage->wheel_entries == NULL ||
        storage->wheel_capacity == 0U || storage->chassis_entry == NULL ||
        storage->attitude_entry == NULL ||
        storage->imu_entries == NULL) {
        return false;
    }

    (void)memset(queue, 0, sizeof(*queue));
    queue->storage = *storage;
    queue->initialized = true;
    refresh_depth_stats(queue);
    return true;
}

/**
 * @brief  校验并复制一条完整 SRPv4 遥测线缆帧到对应有界流。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] queue 已成功初始化的队列。
 * @param  message_id 仅支持 WHEEL_SPEED_STATUS、CHASSIS_STATE、ATTITUDE、IMU_TELEMETRY，
 *                    且须与帧 type 一致。
 * @param  encoded_frame 非 NULL 的完整 SRPv4 线缆帧；成功时复制，不保留指针。
 * @param  encoded_length 完整帧长度，不超过 RADAR_TELEMETRY_QUEUE_MAX_FRAME_SIZE。
 * @param  ingress_timestamp_ms S3 接收时间，单位 ms，原样保存在 entry。
 * @return 普通接受、latest 覆盖或拒绝的精确枚举结果；wheel 满和无效帧返回 REJECTED。
 *         已初始化队列的协议/schema/NaN/Inf 无效帧增加 rejected，
 *         wheel 满增加该流 dropped；queue 为 NULL 或尚未初始化时无法记录到该队列统计。
 * 调用方式：可观测 telemetry sink 调用；旧 radar_telemetry_queue_push() 保留 bool 兼容包装。
 * 线程约束：执行整帧解码/复制且无内部锁；生产者和消费者必须使用同一外层 mutex，禁止 ISR 调用。
 */
radar_telemetry_queue_push_result_t radar_telemetry_queue_push_ex(
    radar_telemetry_queue_t *queue,
    uint16_t message_id,
    const uint8_t *encoded_frame,
    size_t encoded_length,
    uint32_t ingress_timestamp_ms)
{
    srp_frame_t decoded;
    radar_telemetry_queue_class_t stream;

    if (queue == NULL || !queue->initialized ||
        !decode_and_validate(encoded_frame, encoded_length, message_id,
                             &decoded, &stream)) {
        if (queue != NULL && queue->initialized) {
            ++queue->stats.rejected;
        }
        return RADAR_TELEMETRY_QUEUE_PUSH_REJECTED;
    }

    radar_telemetry_queue_stream_stats_t *stream_counter =
        stream_stats(queue, stream);
    if (stream_counter == NULL) {
        ++queue->stats.rejected;
        return RADAR_TELEMETRY_QUEUE_PUSH_REJECTED;
    }

    if (stream == RADAR_TELEMETRY_QUEUE_CLASS_WHEEL) {
        if (queue->wheel_count >= queue->storage.wheel_capacity) {
            ++stream_counter->dropped;
            return RADAR_TELEMETRY_QUEUE_PUSH_REJECTED;
        }

        radar_telemetry_entry_t *destination =
            &queue->storage.wheel_entries[queue->wheel_head];
        (void)memmove(destination->data, encoded_frame, encoded_length);
        destination->length = (uint16_t)encoded_length;
        destination->message_id = message_id;
        destination->ingress_timestamp_ms = ingress_timestamp_ms;
        queue->wheel_head = (queue->wheel_head + 1U) %
                            queue->storage.wheel_capacity;
        ++queue->wheel_count;
        ++stream_counter->accepted;
        refresh_depth_stats(queue);
        return RADAR_TELEMETRY_QUEUE_PUSH_ACCEPTED;
    }

    const unsigned int observation =
        stream == RADAR_TELEMETRY_QUEUE_CLASS_CHASSIS
            ? RADAR_TELEMETRY_OBSERVATION_CHASSIS
            : stream == RADAR_TELEMETRY_QUEUE_CLASS_ATTITUDE
                  ? RADAR_TELEMETRY_OBSERVATION_ATTITUDE
                  : stream == RADAR_TELEMETRY_QUEUE_CLASS_IMU_LSM303
                        ? RADAR_TELEMETRY_OBSERVATION_IMU_LSM303
                        : RADAR_TELEMETRY_OBSERVATION_IMU_BMI323;
    radar_telemetry_entry_t *destination = observation_entry(queue, observation);
    if (destination == NULL) {
        ++queue->stats.rejected;
        return RADAR_TELEMETRY_QUEUE_PUSH_REJECTED;
    }
    const bool overwritten = observation_pending(queue, observation);
    if (overwritten) {
        ++stream_counter->overwritten;
    }
    (void)memmove(destination->data, encoded_frame, encoded_length);
    destination->length = (uint16_t)encoded_length;
    destination->message_id = message_id;
    destination->ingress_timestamp_ms = ingress_timestamp_ms;
    if (observation == RADAR_TELEMETRY_OBSERVATION_CHASSIS) {
        queue->chassis_pending = true;
    } else if (observation == RADAR_TELEMETRY_OBSERVATION_ATTITUDE) {
        queue->attitude_pending = true;
    } else if (observation == RADAR_TELEMETRY_OBSERVATION_IMU_LSM303) {
        queue->imu_pending[0] = true;
    } else {
        queue->imu_pending[1] = true;
    }
    ++stream_counter->accepted;
    refresh_depth_stats(queue);
    return overwritten ? RADAR_TELEMETRY_QUEUE_PUSH_OVERWRITTEN
                       : RADAR_TELEMETRY_QUEUE_PUSH_ACCEPTED;
}

/**
 * @brief  校验并复制一条 SRPv4 遥测帧，向旧调用方返回 bool 接受结果。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] queue 已成功初始化的队列；函数按精确结果更新队列统计。
 * @param  message_id 支持的遥测消息 ID，必须与完整线缆帧 type 一致。
 * @param  encoded_frame 非 NULL 的完整 SRPv4 线缆帧；成功时复制，不保留指针。
 * @param  encoded_length 完整帧长度，不超过 RADAR_TELEMETRY_QUEUE_MAX_FRAME_SIZE。
 * @param  ingress_timestamp_ms S3 接收时间，单位 ms，原样保存在队列项。
 * @return ACCEPTED 或 OVERWRITTEN 映射为 true；REJECTED 映射为 false。
 * 调用方式：兼容历史 bool 调用方；需要区分 latest-only 覆盖时直接调用 push_ex()。
 * 线程约束：继承 push_ex() 的解码/复制开销且无内部锁；与 pop/stats 并发时须持有同一外层 mutex，禁止 ISR。
 * 所有权约束：成功时队列拥有帧字节副本，源缓冲在函数返回后可立即复用。
 */
bool radar_telemetry_queue_push(
    radar_telemetry_queue_t *queue,
    uint16_t message_id,
    const uint8_t *encoded_frame,
    size_t encoded_length,
    uint32_t ingress_timestamp_ms)
{
    return radar_telemetry_queue_push_ex(queue, message_id, encoded_frame,
                                         encoded_length,
                                         ingress_timestamp_ms) !=
           RADAR_TELEMETRY_QUEUE_PUSH_REJECTED;
}

/**
 * @brief  按 wheel 优先且有界公平的策略弹出下一条待发送帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] queue 已成功初始化的队列。
 * @param[out] entry 非 NULL；成功时复制完整 entry 并消费对应 FIFO/槽位。
 * @return 成功弹出一条为 true；无数据、未初始化或参数无效时为 false。
 * 调用方式：单一上行任务调用；最多连续发送 WHEEL_BURST_MAX 条 wheel 后轮转一个 observation。
 * 线程约束：无内部锁并复制完整帧；必须与 push/has_pending/get_stats 用同一外层 mutex，禁止 ISR。
 */
bool radar_telemetry_queue_pop(radar_telemetry_queue_t *queue,
                               radar_telemetry_entry_t *entry)
{
    if (queue == NULL || entry == NULL || !queue->initialized) {
        return false;
    }

    const bool observations_waiting =
        queue->chassis_pending || queue->attitude_pending ||
        queue->imu_pending[0] ||
        queue->imu_pending[1];
    if (queue->wheel_count != 0U &&
        (!observations_waiting ||
         queue->wheel_burst < RADAR_TELEMETRY_QUEUE_WHEEL_BURST_MAX)) {
        return pop_wheel(queue, entry);
    }
    if (observations_waiting && pop_observation(queue, entry)) {
        return true;
    }
    return pop_wheel(queue, entry);
}

/**
 * @brief  查询任一 wheel、chassis、attitude 或 IMU 槽位是否有待发送帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  queue 队列实例；可为 NULL。
 * @return 队列已初始化且至少一个流有数据时为 true，否则为 false。
 * 调用方式：上行任务决定是否继续 burst；true 后仍应以 pop 返回值为准。
 * 线程约束：无锁快照、不阻塞；跨任务读取必须与 push/pop 使用同一外层 mutex。
 */
bool radar_telemetry_queue_has_pending(
    const radar_telemetry_queue_t *queue)
{
    return queue != NULL && queue->initialized &&
           (queue->wheel_count != 0U || queue->chassis_pending ||
            queue->attitude_pending ||
            queue->imu_pending[0] || queue->imu_pending[1]);
}

/**
 * @brief  复制各遥测流接受、覆盖、丢弃计数及深度水位快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  queue 可为 NULL 或未初始化；此时输出全零。
 * @param[out] stats 可为 NULL；非 NULL 时先清零再复制快照。
 * @return 无。
 * 调用方式：低频上行健康日志读取；accepted 只表示本地入队，不表示 TCP/ROS2 已接收。
 * 线程约束：无内部锁；与 push/pop 并发时必须由外层 mutex 保护一致快照。
 */
void radar_telemetry_queue_get_stats(
    const radar_telemetry_queue_t *queue,
    radar_telemetry_queue_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    (void)memset(stats, 0, sizeof(*stats));
    if (queue != NULL && queue->initialized) {
        *stats = queue->stats;
    }
}
