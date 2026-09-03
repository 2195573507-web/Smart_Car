#include <stdio.h>
#include <string.h>

#include "radar_telemetry_queue.h"

/* 遥测队列主机测试；创建人：待确认（当前维护人：Zhiqin）。 */
#include "srp_crc.h"
#include "srp_codec.h"
#include "srp_wire.h"

/**
 * @brief 检查 host 测试条件，失败时打印表达式与位置并从当前 int 测试函数返回 1。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param condition 仅求值一次的布尔表达式。
 * @return 宏本身无独立返回值；condition 为假时使包含它的 int 函数立即返回 1。
 * 调用方式：只在返回 int 的 telemetry host helper/test 中作为语句使用，不能用于 void 函数或表达式位置。
 * 线程约束：单线程主机测试；失败路径写 stderr，不使用 assert/RTOS/ISR，也不具备跨线程日志原子性。
 */
#define TEST_ASSERT(condition) do { \
    if (!(condition)) { \
        (void)fprintf(stderr, "assertion failed: %s at %s:%d\n", \
                      #condition, __FILE__, __LINE__); \
        return 1; \
    } \
} while (0)

/**
 * @brief 为指定遥测消息构造语义合法的 payload，并编码为完整 SRP v4 线缆帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param message_id 目标 SRP 消息 ID；支持 wheel/chassis/attitude/IMU，其他值生成一字节测试 payload。
 * @param sequence 写入 SRP 帧头的序号，并用于 chassis 时间戳。
 * @param sensor_id IMU 消息的传感器 ID，其他消息忽略。
 * @param marker 写入连续 float 字段的基准值。
 * @param[out] wire 至少可写 SRP_MAX_FRAME_SIZE 字节的调用方缓冲，不得为 NULL。
 * @param[out] wire_length 非 NULL；成功时写完整帧长。
 * @return 空输出参数返回 -1；否则原样返回 srp_encode() 状态，0 表示编码成功。
 * 调用方式：各队列测试先构造完整帧，再立即 push；函数不保留输出指针。
 * 线程约束：单线程 host 栈缓冲构造；payload 为局部数组且在 encode 返回前有效，不访问 UART/RTOS。
 */
static int make_frame(uint16_t message_id,
                      uint8_t sequence,
                      uint8_t sensor_id,
                      float marker,
                      uint8_t *wire,
                      uint16_t *wire_length)
{
    uint8_t payload[SRP_MAX_PAYLOAD] = {0};
    uint16_t payload_length = 0U;

    if (wire == NULL || wire_length == NULL) {
        return -1;
    }

    switch (message_id) {
    case SRP_MSG_ID_WHEEL_SPEED_STATUS:
        payload_length = SRP_PAYLOAD_WHEEL_SPEED_STATUS_SIZE;
        for (size_t index = 0U; index < 4U; ++index) {
            srp_wire_write_f32_le(&payload[index * sizeof(float)],
                                   marker + (float)index);
        }
        break;

    case SRP_MSG_ID_CHASSIS_STATE:
        payload_length = SRP_PAYLOAD_CHASSIS_STATE_SIZE;
        payload[0] = SRP_CHASSIS_STATE_SCHEMA;
        payload[1] = SRP_CHASSIS_STATE_FLAG_ODOMETRY_VALID |
                     SRP_CHASSIS_STATE_FLAG_ATTITUDE_READY;
        srp_wire_write_u32_le(&payload[4], (uint32_t)sequence * 10U);
        for (size_t index = 0U; index < 4U; ++index) {
            srp_wire_write_f32_le(&payload[8U + index * sizeof(float)],
                                   marker + (float)index);
        }
        break;

    case SRP_MSG_ID_ATTITUDE:
        payload_length = SRP_PAYLOAD_DUAL_AHRS_SIZE;
        payload[0] = SRP_DUAL_AHRS_SCHEMA;
        for (size_t index = 0U; index < 17U; ++index) {
            srp_wire_write_f32_le(&payload[12U + index * sizeof(float)],
                                   marker + (float)index);
        }
        break;

    case SRP_MSG_ID_IMU_TELEMETRY:
        payload_length = SRP_PAYLOAD_IMU_TELEMETRY_SIZE;
        payload[0] = sensor_id;
        payload[1] = SRP_IMU_TELEMETRY_FLAG_ONLINE;
        for (size_t index = 0U; index < 6U; ++index) {
            srp_wire_write_f32_le(&payload[6U + index * sizeof(float)],
                                   marker + (float)index);
        }
        break;

    default:
        payload_length = 1U;
        payload[0] = 0xA5U;
        break;
    }

    const srp_frame_t frame = {
        .priority = SRP_PRIORITY_TELEMETRY,
        .type = (uint8_t)message_id,
        .sequence = sequence,
        .flags = SRP_FLAG_STREAM_DATA,
        .length = payload_length,
        .payload = payload,
    };
    return srp_encode(&frame, wire, SRP_MAX_FRAME_SIZE,
                           wire_length);
}

/**
 * @brief 将调用方提供的 wheel/chassis/attitude/IMU 存储绑定到一个遥测队列实例。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param[out] queue 待初始化队列，不得为 NULL。
 * @param wheel_entries wheel FIFO 外部数组。
 * @param wheel_capacity wheel_entries 元素数。
 * @param chassis_entry chassis latest-only 外部槽。
 * @param attitude_entry attitude latest-only 外部槽。
 * @param imu_entries 至少两个元素的 LSM303/BMI323 外部槽数组。
 * @return 初始化成功返回 0，radar_telemetry_queue_init() 拒绝存储时返回 1。
 * 调用方式：每个测试在首次 push/pop 前调用，所有外部存储生命周期覆盖该测试。
 * 线程约束：单线程主机初始化；队列不拥有这些数组且本 helper 不创建 mutex。
 */
static int init_queue(radar_telemetry_queue_t *queue,
                      radar_telemetry_entry_t *wheel_entries,
                      size_t wheel_capacity,
                      radar_telemetry_entry_t *chassis_entry,
                      radar_telemetry_entry_t *attitude_entry,
                      radar_telemetry_entry_t *imu_entries)
{
    const radar_telemetry_queue_storage_t storage = {
        .wheel_entries = wheel_entries,
        .wheel_capacity = wheel_capacity,
        .chassis_entry = chassis_entry,
        .attitude_entry = attitude_entry,
        .imu_entries = imu_entries,
    };
    return radar_telemetry_queue_init(queue, &storage) ? 0 : 1;
}

/**
 * @brief 验证队列 entry 可容纳最大 SRP 帧，并拒绝不受支持的最大 LOG 帧且累计 rejected。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 全部 TEST_ASSERT 通过返回 0；首个失败条件打印后返回 1。
 * 调用方式：由 main() 调用；构造 SRP_MAX_PAYLOAD 的 LOG 帧但不把它作为支持的 telemetry 入队。
 * 线程约束：单线程 host 大缓冲测试；不验证任务栈、PSRAM、BLE/TCP 或设备端最大帧传输。
 */
static int test_maximum_frame_contract(void)
{
    radar_telemetry_queue_t queue;
    radar_telemetry_entry_t wheel[1];
    radar_telemetry_entry_t chassis;
    radar_telemetry_entry_t attitude;
    radar_telemetry_entry_t imu[2];
    uint8_t wire[SRP_MAX_FRAME_SIZE];
    uint8_t payload[SRP_MAX_PAYLOAD] = {0};
    uint16_t wire_length = 0U;
    radar_telemetry_queue_stats_t stats;

    TEST_ASSERT(sizeof(wire) == SRP_MAX_FRAME_SIZE);
    TEST_ASSERT(sizeof(wheel[0].data) == SRP_MAX_FRAME_SIZE);
    TEST_ASSERT(init_queue(&queue, wheel, 1U, &chassis, &attitude, imu) == 0);

    const srp_frame_t frame = {
        .priority = SRP_PRIORITY_LOG,
        .type = SRP_MSG_ID_LOG,
        .sequence = 0x55U,
        .flags = SRP_FLAG_STREAM_DATA,
        .length = SRP_MAX_PAYLOAD,
        .payload = payload,
    };
    TEST_ASSERT(srp_encode(&frame, wire, sizeof(wire), &wire_length) == 0);
    TEST_ASSERT(wire_length == SRP_HEADER_SIZE + SRP_MAX_PAYLOAD + SRP_TRAILER_SIZE);
    TEST_ASSERT(!radar_telemetry_queue_push(&queue, SRP_MSG_ID_LOG, wire,
                                            wire_length, 10U));
    radar_telemetry_queue_get_stats(&queue, &stats);
    TEST_ASSERT(stats.rejected == 1U);
    TEST_ASSERT(stats.depth == 0U);
    return 0;
}

/**
 * @brief 验证 wheel 有界 FIFO 保序、满队拒绝新项以及 accepted/dropped/水位统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 全部 TEST_ASSERT 通过返回 0；任一条件失败打印并返回 1。
 * 调用方式：由 main() 调用；容量 2 中推入序号 1/2，再验证序号 3 被拒绝并按序 pop。
 * 线程约束：单线程主机队列测试，不覆盖生产者零等待 mutex 竞争或上行任务并发消费。
 */
static int test_wheel_fifo_order_and_full_rejection(void)
{
    radar_telemetry_queue_t queue;
    radar_telemetry_entry_t wheel[2];
    radar_telemetry_entry_t chassis;
    radar_telemetry_entry_t attitude;
    radar_telemetry_entry_t imu[2];
    radar_telemetry_entry_t output;
    uint8_t wire[SRP_MAX_FRAME_SIZE];
    uint16_t wire_length = 0U;
    radar_telemetry_queue_stats_t stats;

    TEST_ASSERT(init_queue(&queue, wheel, 2U, &chassis, &attitude, imu) == 0);
    for (uint8_t sequence = 1U; sequence <= 2U; ++sequence) {
        TEST_ASSERT(make_frame(SRP_MSG_ID_WHEEL_SPEED_STATUS, sequence, 0U,
                               (float)sequence, wire, &wire_length) == 0);
        TEST_ASSERT(radar_telemetry_queue_push(
                        &queue, SRP_MSG_ID_WHEEL_SPEED_STATUS, wire,
                        wire_length, (uint32_t)sequence * 10U));
    }
    TEST_ASSERT(make_frame(SRP_MSG_ID_WHEEL_SPEED_STATUS, 3U, 0U, 3.0f,
                           wire, &wire_length) == 0);
    TEST_ASSERT(radar_telemetry_queue_push_ex(
                    &queue, SRP_MSG_ID_WHEEL_SPEED_STATUS, wire,
                    wire_length, 30U) ==
                RADAR_TELEMETRY_QUEUE_PUSH_REJECTED);
    radar_telemetry_queue_get_stats(&queue, &stats);
    TEST_ASSERT(stats.wheel.accepted == 2U);
    TEST_ASSERT(stats.wheel.dropped == 1U);
    TEST_ASSERT(stats.wheel.depth == 2U);
    TEST_ASSERT(stats.wheel.high_watermark == 2U);

    TEST_ASSERT(radar_telemetry_queue_pop(&queue, &output));
    TEST_ASSERT(output.message_id == SRP_MSG_ID_WHEEL_SPEED_STATUS);
    TEST_ASSERT(output.data[5] == 1U);
    TEST_ASSERT(output.ingress_timestamp_ms == 10U);
    TEST_ASSERT(radar_telemetry_queue_pop(&queue, &output));
    TEST_ASSERT(output.data[5] == 2U);
    TEST_ASSERT(!radar_telemetry_queue_has_pending(&queue));
    return 0;
}

/**
 * @brief 验证 attitude latest-only 槽用新样本覆盖旧样本并保留最新元数据。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 全部 TEST_ASSERT 通过返回 0；覆盖统计或弹出内容不符时返回 1。
 * 调用方式：由 main() 调用；连续 push 序号 10/11，随后只 pop 最新序号 11。
 * 线程约束：单线程 host 测试；不覆盖跨任务覆盖时的外层 mutex 或真实姿态有效性。
 */
static int test_attitude_latest_only(void)
{
    radar_telemetry_queue_t queue;
    radar_telemetry_entry_t wheel[1];
    radar_telemetry_entry_t chassis;
    radar_telemetry_entry_t attitude;
    radar_telemetry_entry_t imu[2];
    radar_telemetry_entry_t output;
    uint8_t wire[SRP_MAX_FRAME_SIZE];
    uint16_t wire_length = 0U;
    radar_telemetry_queue_stats_t stats;

    TEST_ASSERT(init_queue(&queue, wheel, 1U, &chassis, &attitude, imu) == 0);
    TEST_ASSERT(make_frame(SRP_MSG_ID_ATTITUDE, 10U, 0U, 10.0f, wire,
                           &wire_length) == 0);
    TEST_ASSERT(radar_telemetry_queue_push_ex(
                    &queue, SRP_MSG_ID_ATTITUDE, wire,
                    wire_length, 100U) ==
                RADAR_TELEMETRY_QUEUE_PUSH_ACCEPTED);
    TEST_ASSERT(make_frame(SRP_MSG_ID_ATTITUDE, 11U, 0U, 11.0f, wire,
                           &wire_length) == 0);
    TEST_ASSERT(radar_telemetry_queue_push_ex(
                    &queue, SRP_MSG_ID_ATTITUDE, wire,
                    wire_length, 110U) ==
                RADAR_TELEMETRY_QUEUE_PUSH_OVERWRITTEN);
    radar_telemetry_queue_get_stats(&queue, &stats);
    TEST_ASSERT(stats.attitude.accepted == 2U);
    TEST_ASSERT(stats.attitude.overwritten == 1U);
    TEST_ASSERT(stats.attitude.depth == 1U);
    TEST_ASSERT(stats.attitude.high_watermark == 1U);
    TEST_ASSERT(radar_telemetry_queue_pop(&queue, &output));
    TEST_ASSERT(output.message_id == SRP_MSG_ID_ATTITUDE);
    TEST_ASSERT(output.data[5] == 11U);
    TEST_ASSERT(output.ingress_timestamp_ms == 110U);
    return 0;
}

/**
 * @brief 验证 chassis latest-only 覆盖，以及未知 flags 和负累计距离的语义拒绝。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 全部 TEST_ASSERT 通过返回 0；内容、覆盖或两次 rejected 统计不符时返回 1。
 * 调用方式：由 main() 调用；对变异 payload 重新计算 CRC，确保拒绝来自业务语义而非 CRC。
 * 线程约束：单线程白盒 host 测试；不验证 CM7 里程计生成频率、freshness 或车辆位姿。
 */
static int test_chassis_latest_only_and_semantic_validation(void)
{
    radar_telemetry_queue_t queue;
    radar_telemetry_entry_t wheel[1];
    radar_telemetry_entry_t chassis;
    radar_telemetry_entry_t attitude;
    radar_telemetry_entry_t imu[2];
    radar_telemetry_entry_t output;
    uint8_t wire[SRP_MAX_FRAME_SIZE];
    uint16_t wire_length = 0U;
    radar_telemetry_queue_stats_t stats;

    TEST_ASSERT(init_queue(&queue, wheel, 1U, &chassis, &attitude, imu) == 0);
    TEST_ASSERT(make_frame(SRP_MSG_ID_CHASSIS_STATE, 30U, 0U, 10.0f,
                           wire, &wire_length) == 0);
    TEST_ASSERT(radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_CHASSIS_STATE, wire, wire_length, 300U));
    TEST_ASSERT(make_frame(SRP_MSG_ID_CHASSIS_STATE, 31U, 0U, 20.0f,
                           wire, &wire_length) == 0);
    TEST_ASSERT(radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_CHASSIS_STATE, wire, wire_length, 310U));
    radar_telemetry_queue_get_stats(&queue, &stats);
    TEST_ASSERT(stats.chassis.accepted == 2U);
    TEST_ASSERT(stats.chassis.overwritten == 1U);
    TEST_ASSERT(stats.chassis.depth == 1U);
    TEST_ASSERT(radar_telemetry_queue_pop(&queue, &output));
    TEST_ASSERT(output.message_id == SRP_MSG_ID_CHASSIS_STATE);
    TEST_ASSERT(output.data[5] == 31U);
    TEST_ASSERT(srp_wire_read_f32_le(&output.data[16]) == 20.0f);

    TEST_ASSERT(make_frame(SRP_MSG_ID_CHASSIS_STATE, 32U, 0U, 30.0f,
                           wire, &wire_length) == 0);
    wire[9] |= 0x80U;
    {
        const uint16_t crc = srp_crc16_ccitt_false(
            &wire[2], 6U + SRP_PAYLOAD_CHASSIS_STATE_SIZE);
        wire[wire_length - 4U] = (uint8_t)crc;
        wire[wire_length - 3U] = (uint8_t)(crc >> 8U);
    }
    TEST_ASSERT(!radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_CHASSIS_STATE, wire, wire_length, 320U));

    TEST_ASSERT(make_frame(SRP_MSG_ID_CHASSIS_STATE, 33U, 0U, 40.0f,
                           wire, &wire_length) == 0);
    srp_wire_write_f32_le(&wire[28], -1.0f);
    {
        const uint16_t crc = srp_crc16_ccitt_false(
            &wire[2], 6U + SRP_PAYLOAD_CHASSIS_STATE_SIZE);
        wire[wire_length - 4U] = (uint8_t)crc;
        wire[wire_length - 3U] = (uint8_t)(crc >> 8U);
    }
    TEST_ASSERT(!radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_CHASSIS_STATE, wire, wire_length, 330U));
    radar_telemetry_queue_get_stats(&queue, &stats);
    TEST_ASSERT(stats.rejected == 2U);
    return 0;
}

/**
 * @brief 验证 LSM303 与 BMI323 使用独立 latest-only 槽，且只覆盖同一传感器旧样本。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 全部 TEST_ASSERT 通过返回 0；分流、覆盖、深度或弹出顺序不符时返回 1。
 * 调用方式：由 main() 调用；依次 push LSM303、BMI323、更新后的 LSM303，再消费两项。
 * 线程约束：单线程 host 队列测试；不读取真实 IMU、标定状态或传感器时间戳。
 */
static int test_independent_imu_latest_slots(void)
{
    radar_telemetry_queue_t queue;
    radar_telemetry_entry_t wheel[1];
    radar_telemetry_entry_t chassis;
    radar_telemetry_entry_t attitude;
    radar_telemetry_entry_t imu[2];
    radar_telemetry_entry_t output;
    uint8_t wire[SRP_MAX_FRAME_SIZE];
    uint16_t wire_length = 0U;
    radar_telemetry_queue_stats_t stats;

    TEST_ASSERT(init_queue(&queue, wheel, 1U, &chassis, &attitude, imu) == 0);
    TEST_ASSERT(make_frame(SRP_MSG_ID_IMU_TELEMETRY, 20U,
                           SRP_IMU_SENSOR_LSM303, 20.0f, wire,
                           &wire_length) == 0);
    TEST_ASSERT(radar_telemetry_queue_push(&queue, SRP_MSG_ID_IMU_TELEMETRY,
                                           wire, wire_length, 200U));
    TEST_ASSERT(make_frame(SRP_MSG_ID_IMU_TELEMETRY, 21U,
                           SRP_IMU_SENSOR_BMI323, 21.0f, wire,
                           &wire_length) == 0);
    TEST_ASSERT(radar_telemetry_queue_push(&queue, SRP_MSG_ID_IMU_TELEMETRY,
                                           wire, wire_length, 210U));
    TEST_ASSERT(make_frame(SRP_MSG_ID_IMU_TELEMETRY, 22U,
                           SRP_IMU_SENSOR_LSM303, 22.0f, wire,
                           &wire_length) == 0);
    TEST_ASSERT(radar_telemetry_queue_push(&queue, SRP_MSG_ID_IMU_TELEMETRY,
                                           wire, wire_length, 220U));

    radar_telemetry_queue_get_stats(&queue, &stats);
    TEST_ASSERT(stats.imu_lsm303.accepted == 2U);
    TEST_ASSERT(stats.imu_lsm303.overwritten == 1U);
    TEST_ASSERT(stats.imu_bmi323.accepted == 1U);
    TEST_ASSERT(stats.imu_bmi323.overwritten == 0U);
    TEST_ASSERT(stats.depth == 2U);
    TEST_ASSERT(radar_telemetry_queue_pop(&queue, &output));
    TEST_ASSERT(output.data[5] == 22U);
    TEST_ASSERT(output.data[8] == SRP_IMU_SENSOR_LSM303);
    TEST_ASSERT(radar_telemetry_queue_pop(&queue, &output));
    TEST_ASSERT(output.data[5] == 21U);
    TEST_ASSERT(output.data[8] == SRP_IMU_SENSOR_BMI323);
    TEST_ASSERT(!radar_telemetry_queue_has_pending(&queue));
    return 0;
}

/**
 * @brief 验证 wheel 最多连续弹出 4 项后让出给 observation，并按轮转顺序服务 chassis/attitude。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 全部 TEST_ASSERT 通过返回 0；任何公平顺序或序号断言失败时返回 1。
 * 调用方式：由 main() 调用；先排入 attitude/chassis 和 8 条 wheel，再按预期次序逐项 pop。
 * 线程约束：单线程确定性调度测试；不覆盖持续生产、锁竞争或网络慢消费者下的时序。
 */
static int test_wheel_priority_is_bounded_fair(void)
{
    radar_telemetry_queue_t queue;
    radar_telemetry_entry_t wheel[8];
    radar_telemetry_entry_t chassis;
    radar_telemetry_entry_t attitude;
    radar_telemetry_entry_t imu[2];
    radar_telemetry_entry_t output;
    uint8_t wire[SRP_MAX_FRAME_SIZE];
    uint16_t wire_length = 0U;
    uint8_t sequence;

    TEST_ASSERT(init_queue(&queue, wheel, 8U, &chassis, &attitude, imu) == 0);
    TEST_ASSERT(make_frame(SRP_MSG_ID_ATTITUDE, 100U, 0U, 100.0f, wire,
                           &wire_length) == 0);
    TEST_ASSERT(radar_telemetry_queue_push(&queue, SRP_MSG_ID_ATTITUDE, wire,
                                           wire_length, 1000U));
    TEST_ASSERT(make_frame(SRP_MSG_ID_CHASSIS_STATE, 101U, 0U, 10.0f, wire,
                           &wire_length) == 0);
    TEST_ASSERT(radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_CHASSIS_STATE, wire, wire_length, 1001U));
    for (sequence = 1U; sequence <= 8U; ++sequence) {
        TEST_ASSERT(make_frame(SRP_MSG_ID_WHEEL_SPEED_STATUS, sequence, 0U,
                               (float)sequence, wire, &wire_length) == 0);
        TEST_ASSERT(radar_telemetry_queue_push(
                        &queue, SRP_MSG_ID_WHEEL_SPEED_STATUS, wire,
                        wire_length, (uint32_t)sequence));
    }

    for (sequence = 1U; sequence <= 4U; ++sequence) {
        TEST_ASSERT(radar_telemetry_queue_pop(&queue, &output));
        TEST_ASSERT(output.message_id == SRP_MSG_ID_WHEEL_SPEED_STATUS);
        TEST_ASSERT(output.data[5] == sequence);
    }
    TEST_ASSERT(radar_telemetry_queue_pop(&queue, &output));
    TEST_ASSERT(output.message_id == SRP_MSG_ID_CHASSIS_STATE);
    TEST_ASSERT(output.data[5] == 101U);
    for (sequence = 5U; sequence <= 8U; ++sequence) {
        TEST_ASSERT(radar_telemetry_queue_pop(&queue, &output));
        TEST_ASSERT(output.message_id == SRP_MSG_ID_WHEEL_SPEED_STATUS);
        TEST_ASSERT(output.data[5] == sequence);
    }
    TEST_ASSERT(radar_telemetry_queue_pop(&queue, &output));
    TEST_ASSERT(output.message_id == SRP_MSG_ID_ATTITUDE);
    TEST_ASSERT(output.data[5] == 100U);
    return 0;
}

/**
 * @brief 验证空帧、消息类型不匹配、CRC、未知 IMU ID、截断和 ACK flag 遥测均被拒绝。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 全部 TEST_ASSERT 通过返回 0；六次拒绝计数、深度或 pending 状态不符时返回 1。
 * 调用方式：由 main() 调用；对需要绕过 CRC 的头变异重新计算校验，再检查业务约束。
 * 线程约束：单线程 host 负向测试；不覆盖所有 NaN/Inf 组合、锁失败或 ISR 输入。
 */
static int test_invalid_inputs_are_rejected(void)
{
    radar_telemetry_queue_t queue;
    radar_telemetry_entry_t wheel[2];
    radar_telemetry_entry_t chassis;
    radar_telemetry_entry_t attitude;
    radar_telemetry_entry_t imu[2];
    uint8_t wire[SRP_MAX_FRAME_SIZE];
    uint16_t wire_length = 0U;
    radar_telemetry_queue_stats_t stats;

    TEST_ASSERT(init_queue(&queue, wheel, 2U, &chassis, &attitude, imu) == 0);
    TEST_ASSERT(!radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_WHEEL_SPEED_STATUS, NULL, 0U, 0U));
    TEST_ASSERT(make_frame(SRP_MSG_ID_WHEEL_SPEED_STATUS, 1U, 0U, 1.0f,
                           wire, &wire_length) == 0);
    TEST_ASSERT(!radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_ATTITUDE, wire, wire_length, 0U));
    wire[wire_length - 1U] ^= 0x01U;
    TEST_ASSERT(!radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_WHEEL_SPEED_STATUS, wire,
                    wire_length, 0U));
    TEST_ASSERT(make_frame(SRP_MSG_ID_IMU_TELEMETRY, 2U, 0x03U, 2.0f, wire,
                           &wire_length) == 0);
    TEST_ASSERT(!radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_IMU_TELEMETRY, wire, wire_length, 0U));
    TEST_ASSERT(make_frame(SRP_MSG_ID_WHEEL_SPEED_STATUS, 3U, 0U, 3.0f,
                           wire, &wire_length) == 0);
    TEST_ASSERT(!radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_WHEEL_SPEED_STATUS, wire,
                    wire_length - 1U, 0U));
    TEST_ASSERT(make_frame(SRP_MSG_ID_WHEEL_SPEED_STATUS, 4U, 0U, 4.0f,
                           wire, &wire_length) == 0);
    wire[4] = SRP_FLAG_ACK_REQUIRED; /* Telemetry must not request ACK. */
    {
        const uint16_t crc = srp_crc16_ccitt_false(
            &wire[2], 6U + SRP_PAYLOAD_WHEEL_SPEED_STATUS_SIZE);
        wire[wire_length - 4U] = (uint8_t)crc;
        wire[wire_length - 3U] = (uint8_t)(crc >> 8U);
    }
    TEST_ASSERT(!radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_WHEEL_SPEED_STATUS, wire,
                    wire_length, 0U));
    radar_telemetry_queue_get_stats(&queue, &stats);
    TEST_ASSERT(stats.rejected == 6U);
    TEST_ASSERT(stats.depth == 0U);
    TEST_ASSERT(!radar_telemetry_queue_has_pending(&queue));
    return 0;
}

/**
 * @brief 顺序运行遥测队列容量、分流、覆盖、公平和拒绝场景。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 全部子测试返回 0 时返回 0；首个失败子测试使 main 返回 1。
 * 调用方式：由 radar/tests/run_host_tests.sh 编译并直接运行；无命令行参数。
 * 线程约束：单进程单线程，不创建外层 mutex/FreeRTOS 任务，不证明 TCP/ROS2 或设备遥测链路通过。
 */
int main(void)
{
    if (test_maximum_frame_contract() != 0 ||
        test_wheel_fifo_order_and_full_rejection() != 0 ||
        test_attitude_latest_only() != 0 ||
        test_chassis_latest_only_and_semantic_validation() != 0 ||
        test_independent_imu_latest_slots() != 0 ||
        test_wheel_priority_is_bounded_fair() != 0 ||
        test_invalid_inputs_are_rejected() != 0) {
        return 1;
    }
    return 0;
}
