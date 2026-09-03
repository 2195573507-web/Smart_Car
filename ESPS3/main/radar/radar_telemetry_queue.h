#ifndef S3_RADAR_TELEMETRY_QUEUE_H
#define S3_RADAR_TELEMETRY_QUEUE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "srp_registry.h"

/*
 * S3 遥测有界队列。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 队列只拥有完整 SRPv4 线缆帧的副本，负责有限公平调度，不负责 UART/BLE 发送。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* 队列传递完整 SRPv4 线缆帧；容量直接引用共享协议上限，避免帧跨越
 * service/uplink 边界时被静默截断。 */
#define RADAR_TELEMETRY_QUEUE_MAX_FRAME_SIZE SRP_MAX_FRAME_SIZE
#define RADAR_TELEMETRY_QUEUE_IMU_SLOT_COUNT 2U
#define RADAR_TELEMETRY_QUEUE_WHEEL_BURST_MAX 4U

/** SRPv4 遥测在本地有界队列中的存储/调度类别。 */
typedef enum {
    RADAR_TELEMETRY_QUEUE_CLASS_WHEEL = 0, /**< 轮速状态：有界 FIFO，满时拒绝新帧。 */
    RADAR_TELEMETRY_QUEUE_CLASS_CHASSIS,   /**< 底盘状态：单槽 latest-only，覆盖旧待发值。 */
    RADAR_TELEMETRY_QUEUE_CLASS_ATTITUDE,  /**< DualAHRS 姿态：单槽 latest-only。 */
    RADAR_TELEMETRY_QUEUE_CLASS_IMU_LSM303,/**< LSM303 遥测：独立单槽 latest-only。 */
    RADAR_TELEMETRY_QUEUE_CLASS_IMU_BMI323,/**< BMI323 遥测：独立单槽 latest-only。 */
    RADAR_TELEMETRY_QUEUE_CLASS_COUNT      /**< 类别数量哨兵，不是可入队消息类别。 */
} radar_telemetry_queue_class_t;

/** 一条待上行 SRPv4 完整线缆帧；entry 自身拥有 data 字节副本。 */
typedef struct {
    uint8_t data[RADAR_TELEMETRY_QUEUE_MAX_FRAME_SIZE]; /**< 完整 SRPv4 帧副本，仅前 length 字节有效。 */
    uint16_t length; /**< data 有效长度，单位 byte，不超过 SRP_MAX_FRAME_SIZE。 */
    uint16_t message_id; /**< 入队时核验一致的 SRP_MSG_ID_*；当前合法值均为 8 位。 */
    uint32_t ingress_timestamp_ms; /**< S3 收到帧的单调时间戳，单位 ms。 */
} radar_telemetry_entry_t;

/** 单个遥测流的累计计数与深度；只说明本地队列结果。 */
typedef struct {
    uint32_t accepted; /**< 校验并复制入槽的累计帧数；latest-only 覆盖也计入。 */
    uint32_t overwritten; /**< latest-only 槽已有待发值时被新值替换的累计次数。 */
    uint32_t dropped; /**< 因有界 FIFO 已满而拒绝的累计帧数；当前主要用于 wheel。 */
    size_t depth; /**< 当前待发送条目数。 */
    size_t high_watermark; /**< 初始化以来该流 depth 的历史最大值。 */
} radar_telemetry_queue_stream_stats_t;

/** 全部遥测流的队列统计快照；accepted 不等同于网络或 ROS2 接收成功。 */
typedef struct {
    radar_telemetry_queue_stream_stats_t wheel; /**< 轮速 FIFO 统计。 */
    radar_telemetry_queue_stream_stats_t chassis; /**< 底盘状态 latest-only 统计。 */
    radar_telemetry_queue_stream_stats_t attitude; /**< DualAHRS latest-only 统计。 */
    radar_telemetry_queue_stream_stats_t imu_lsm303; /**< LSM303 latest-only 统计。 */
    radar_telemetry_queue_stream_stats_t imu_bmi323; /**< BMI323 latest-only 统计。 */
    uint32_t rejected; /**< 参数、SRP、schema/数值或内部分类非法的累计帧数。 */
    size_t depth; /**< 所有流当前待发条目总数。 */
    size_t high_watermark; /**< 初始化以来总 depth 的历史最大值。 */
} radar_telemetry_queue_stats_t;

/** 队列所需外部存储描述；所有指针均借用，队列不分配或释放。 */
typedef struct {
    radar_telemetry_entry_t *wheel_entries; /**< 至少 wheel_capacity 项的轮速 FIFO 数组。 */
    size_t wheel_capacity; /**< wheel_entries 元素数，必须大于 0。 */
    radar_telemetry_entry_t *chassis_entry; /**< 单个底盘状态 latest-only 槽。 */
    radar_telemetry_entry_t *attitude_entry; /**< 单个 DualAHRS latest-only 槽。 */
    radar_telemetry_entry_t *imu_entries; /**< 至少两项：索引 0 为 LSM303，索引 1 为 BMI323。 */
} radar_telemetry_queue_storage_t;

/* 队列状态刻意保持为不含 RTOS 原语的普通 C 对象；多个执行上下文访问
 * push/pop/stats 时必须由调用方串行化。 */
/** 不含锁的队列运行状态；外部存储指针按值保存，所有访问须由调用方串行化。 */
typedef struct {
    radar_telemetry_queue_storage_t storage; /**< init 时复制的借用外部存储描述。 */
    size_t wheel_head; /**< 下一轮速帧写入索引。 */
    size_t wheel_tail; /**< 最旧待发轮速帧索引。 */
    size_t wheel_count; /**< 当前待发轮速帧数，范围 0..wheel_capacity。 */
    bool chassis_pending; /**< chassis_entry 是否含未发送的新快照。 */
    bool attitude_pending; /**< attitude_entry 是否含未发送的新快照。 */
    bool imu_pending[RADAR_TELEMETRY_QUEUE_IMU_SLOT_COUNT]; /**< 两路 IMU 槽的待发标志，索引语义同 storage。 */
    uint8_t next_observation; /**< 下一次 latest-only 公平轮转起始索引，范围 0..3。 */
    uint8_t wheel_burst; /**< 上次 observation 后连续弹出的 wheel 数；在 UINT8_MAX 饱和。 */
    bool initialized; /**< 外部存储已校验绑定标志；false 时 push/pop 拒绝。 */
    radar_telemetry_queue_stats_t stats; /**< 队列拥有的累计计数和当前深度统计。 */
} radar_telemetry_queue_t;

/** 单次 push 的精确结果；旧 bool API 仍把 ACCEPTED/OVERWRITTEN 都映射为 true。 */
typedef enum {
    RADAR_TELEMETRY_QUEUE_PUSH_REJECTED = 0, /**< 参数、协议、schema 或容量原因拒绝。 */
    RADAR_TELEMETRY_QUEUE_PUSH_ACCEPTED, /**< 新帧进入空槽或有界 FIFO。 */
    RADAR_TELEMETRY_QUEUE_PUSH_OVERWRITTEN /**< latest-only 新帧替换旧待发帧。 */
} radar_telemetry_queue_push_result_t;

/**
 * @brief 校验队列存储并建立初始状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[out] queue 非 NULL；成功时清零状态并复制 storage 指针/容量描述。
 * @param  storage 非 NULL；wheel_entries、chassis_entry、attitude_entry、
 *                 imu_entries 均必须有效，wheel_capacity 必须大于 0。
 *                 imu_entries 至少包含两个元素。
 * @return true 表示存储绑定成功；参数/槽位无效返回 false。函数不分配或清零 entry 存储。
 * 调用方式：S3 上行任务创建前调用一次；所有外部存储生命周期必须覆盖 queue 使用期。
 * 线程约束：纯内存初始化、无 RTOS 原语；初始化期间禁止并发访问。
 * 数据布局：imu_entries[0] 对应 LSM303，imu_entries[1] 对应 BMI323。
 */
bool radar_telemetry_queue_init(
    radar_telemetry_queue_t *queue,
    const radar_telemetry_queue_storage_t *storage);

/**
 * @brief 校验并复制一条 SRPv4 遥测帧入队。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  queue 已成功初始化的队列。
 * @param  message_id 仅支持 WHEEL_SPEED_STATUS、CHASSIS_STATE、ATTITUDE、
 *                    IMU_TELEMETRY，且必须与帧 type 一致。
 * @param  encoded_frame 非 NULL 的完整 SRPv4 线缆帧；成功时复制，不保留指针。
 * @param  encoded_length 完整帧长度，不超过 SRP_MAX_FRAME_SIZE。
 * @param  ingress_timestamp_ms S3 接收时间，单位 ms，原样保存在 entry。
 * @return true 表示接受。wheel 队列满时拒绝新样本；底盘状态、姿态和两路 IMU 采用 latest-only，
 *         待发槽已有数据时覆盖旧样本并仍返回 true。协议/schema/NaN/Inf 无效时返回 false。
 * 调用方式：仅从已完成 SRP 解码边界的 telemetry sink 调用；本函数会再次解码并校验 payload。
 * 线程约束：执行整帧复制且无内部锁；服务任务与上行任务并发时必须使用同一外层 mutex，禁止 ISR 调用。
 */
bool radar_telemetry_queue_push(
    radar_telemetry_queue_t *queue,
    uint16_t message_id,
    const uint8_t *encoded_frame,
    size_t encoded_length,
    uint32_t ingress_timestamp_ms);

/**
 * @brief 与 radar_telemetry_queue_push() 相同，但区分普通接受、latest 覆盖和拒绝。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[in,out] queue 已成功初始化的队列；函数按结果更新队列及对应流统计。
 * @param  message_id 仅支持 WHEEL_SPEED_STATUS、CHASSIS_STATE、ATTITUDE、
 *                    IMU_TELEMETRY，且必须与帧 type 一致。
 * @param  encoded_frame 非 NULL 的完整 SRPv4 线缆帧；接受时复制，不保留指针。
 * @param  encoded_length 完整帧长度，不超过 RADAR_TELEMETRY_QUEUE_MAX_FRAME_SIZE。
 * @param  ingress_timestamp_ms S3 接收时间，单位 ms，原样写入队列项。
 * @return REJECTED、ACCEPTED 或 OVERWRITTEN；不改变队列和 wire 语义。
 * 调用方式：需要可观测性的 sink 使用本接口；历史调用方继续使用 bool 包装。
 * 线程约束：执行 SRPv4 解码、浮点校验和整帧复制且无内部锁；调用方必须持有同一外层 mutex，禁止 ISR。
 * 所有权约束：队列仅保留 encoded_frame 前 encoded_length 字节的副本，源缓冲返回后可立即复用。
 */
radar_telemetry_queue_push_result_t radar_telemetry_queue_push_ex(
    radar_telemetry_queue_t *queue,
    uint16_t message_id,
    const uint8_t *encoded_frame,
    size_t encoded_length,
    uint32_t ingress_timestamp_ms);

/**
 * @brief 按有界公平策略取出下一条帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  queue 已成功初始化的队列。
 * @param[out] entry 非 NULL；成功时复制完整 entry 并消费对应槽位。
 * @return true 表示弹出一条帧；无数据、未初始化或参数无效返回 false。
 * 调用方式：单一上行任务调用。最多连续发送 RADAR_TELEMETRY_QUEUE_WHEEL_BURST_MAX 条
 *           wheel 后轮转一个待发 observation，避免底盘状态/姿态/IMU 饥饿。
 * 线程约束：无内部锁；与 push/has_pending/stats 的并发访问必须由外层串行化，禁止 ISR 调用。
 */
bool radar_telemetry_queue_pop(radar_telemetry_queue_t *queue,
                               radar_telemetry_entry_t *entry);

/**
 * @brief  查询任一 wheel/chassis/attitude/IMU 槽位是否存在待发送帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  queue 队列实例；可为 NULL。
 * @return 已初始化且至少一个流有数据时为 true，否则 false。
 * 调用方式：上行任务用于决定是否继续 burst；返回 true 后 pop 仍可能因并发消费而失败。
 * 线程约束：无锁快照；跨任务使用时必须与 push/pop 采用相同外层同步。
 */
bool radar_telemetry_queue_has_pending(
    const radar_telemetry_queue_t *queue);

/**
 * @brief  复制各遥测流接受/覆盖/丢弃计数及深度水位。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  queue 可为 NULL或未初始化；此时输出全零。
 * @param[out] stats 可为 NULL；非 NULL 时先清零再复制快照。
 * @return 无。
 * 调用方式：低频健康日志读取；accepted 只表示进入本地队列，不代表 TCP/ROS2 已接收。
 * 线程约束：无内部锁；与 push/pop 并发时须由外层 mutex 保护一致快照。
 */
void radar_telemetry_queue_get_stats(
    const radar_telemetry_queue_t *queue,
    radar_telemetry_queue_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* S3_RADAR_TELEMETRY_QUEUE_H */
