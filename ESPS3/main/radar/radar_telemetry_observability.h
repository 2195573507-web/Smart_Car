#ifndef S3_RADAR_TELEMETRY_OBSERVABILITY_H
#define S3_RADAR_TELEMETRY_OBSERVABILITY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "radar_telemetry_queue.h"
#include "radar_uplink_tx.h"

#ifdef __cplusplus
extern "C" {
#endif

/** S3 雷达遥测从 service sink 到上行发送边界的累计可观测统计。 */
typedef struct {
    uint32_t telemetry_sink_calls; /**< telemetry sink 被调用的总次数。 */
    uint32_t sink_wheel; /**< sink 收到轮速状态帧的次数。 */
    uint32_t sink_attitude; /**< sink 收到姿态帧的次数。 */
    uint32_t sink_imu; /**< sink 收到两类 IMU 帧的合计次数。 */
    uint32_t sink_chassis; /**< sink 收到底盘状态帧的次数。 */
    uint32_t telemetry_lock_drops; /**< 因外层遥测锁不可用而丢弃的次数。 */
    uint32_t queue_accepted; /**< 队列普通接受新帧的次数。 */
    uint32_t queue_rejected; /**< 队列拒绝帧的次数。 */
    uint32_t queue_overwritten; /**< latest-only 槽覆盖旧帧的次数。 */
    uint32_t telemetry_packets_prepared; /**< 已封装为待发送上行包的次数。 */
    uint32_t telemetry_type2_sent; /**< 类型 2 遥测包完整发送成功次数。 */
    uint32_t telemetry_stale_drops; /**< 超过时效限制而在上行前丢弃的次数。 */
    uint32_t telemetry_send_failures; /**< 上行发送失败或永久错误次数。 */
} radar_telemetry_observability_stats_t;

/**
 * @brief  清零一份雷达遥测可观测统计对象。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[out] stats 待初始化对象；可为 NULL，非 NULL 时整体清零。
 * @return 无；stats 为 NULL 时静默不动作。
 * 调用方式：上行模块完成队列准备后、开始接收 telemetry sink 回调前调用一次。
 * 线程约束：纯内存写入且无内部锁；初始化期间禁止其他上下文并发读写，禁止在 ISR 中与任务共享访问。
 * 所有权约束：对象存储始终由调用方拥有，本函数不分配或保留指针。
 */
void radar_telemetry_observability_init(
    radar_telemetry_observability_stats_t *stats);

/**
 * @brief  记录一次 telemetry sink 调用，并按受支持的 SRPv4 消息类型分类计数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[in,out] stats 已初始化的统计对象；可为 NULL。
 * @param  message_id 收到的 SRPv4 消息 ID；未知类型只增加 sink 总次数，不增加分类次数。
 * @return 无；stats 为 NULL 时不动作，各计数达到 UINT32_MAX 后保持饱和。
 * 调用方式：telemetry sink 每次收到候选帧时调用，是否随后成功入队由 queue result 接口另行记录。
 * 线程约束：无内部锁；同一 stats 被任务并发访问时必须由调用方统一串行化，禁止无保护的 ISR/任务共享。
 */
void radar_telemetry_observability_note_sink_call(
    radar_telemetry_observability_stats_t *stats,
    uint16_t message_id);

/**
 * @brief  记录一次遥测队列外层锁未取得导致的丢弃。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[in,out] stats 已初始化的统计对象；可为 NULL。
 * @return 无；stats 为 NULL 时不动作，计数在 UINT32_MAX 饱和。
 * 调用方式：telemetry sink、pop 或统计快照的零等待 mutex 失败路径调用。
 * 线程约束：函数自身无锁、不阻塞；调用方须用与其他统计更新相同的临界区保护共享对象。
 */
void radar_telemetry_observability_note_lock_drop(
    radar_telemetry_observability_stats_t *stats);

/**
 * @brief  按队列 push 精确结果更新接受、覆盖或拒绝计数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[in,out] stats 已初始化的统计对象；可为 NULL。
 * @param  result 队列 push 结果；OVERWRITTEN 同时增加 accepted 和 overwritten，
 *                ACCEPTED 只增加 accepted，其他值均按 rejected 统计。
 * @return 无；stats 为 NULL 时不动作，所有计数均在 UINT32_MAX 饱和。
 * 调用方式：telemetry sink 完成 radar_telemetry_queue_push_ex() 后调用一次。
 * 线程约束：无内部锁、不阻塞；共享 stats 的所有更新与快照必须由调用方使用同一临界区串行化。
 */
void radar_telemetry_observability_note_queue_result(
    radar_telemetry_observability_stats_t *stats,
    radar_telemetry_queue_push_result_t result);

/**
 * @brief  记录一次类型 2 SRPv4 遥测外层包已成功封装并进入待发送状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[in,out] stats 已初始化的统计对象；可为 NULL。
 * @return 无；stats 为 NULL 时不动作，计数在 UINT32_MAX 饱和。
 * 调用方式：上行任务成功编码完整类型 2 包后调用；该计数不表示 TCP 已发送或对端已接收。
 * 线程约束：无内部锁、不阻塞；共享对象必须由调用方统一加锁，禁止无保护的并发更新。
 */
void radar_telemetry_observability_note_packet_prepared(
    radar_telemetry_observability_stats_t *stats);

/**
 * @brief  按类型 2 包发送状态记录完整发送成功或永久失败。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[in,out] stats 已初始化的统计对象；可为 NULL。
 * @param  result radar_uplink_tx_send() 结果；COMPLETE 增加 sent，FAILED 增加 failure，WAIT 不计数。
 * @return 无；stats 为 NULL 时不动作，计数在 UINT32_MAX 饱和。
 * 调用方式：上行任务每次推进待发类型 2 包后调用；sent 仅表示本地整包 send 完成，不证明 ROS2 消费。
 * 线程约束：无内部锁、不阻塞；共享 stats 的调用必须由外层临界区串行化。
 */
void radar_telemetry_observability_note_type2_tx_result(
    radar_telemetry_observability_stats_t *stats,
    radar_uplink_tx_result_t result);

/**
 * @brief  记录一条超过上行时效上限而被丢弃的遥测队列项。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[in,out] stats 已初始化的统计对象；可为 NULL。
 * @return 无；stats 为 NULL 时不动作，计数在 UINT32_MAX 饱和。
 * 调用方式：上行任务弹出 entry 并判定其帧龄超限时调用；不改变队列自身统计。
 * 线程约束：无内部锁、不阻塞；共享 stats 必须由调用方使用统一临界区保护。
 */
void radar_telemetry_observability_note_stale_drop(
    radar_telemetry_observability_stats_t *stats);

/**
 * @brief  复制一份可观测统计快照，或在源对象缺失时输出全零快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  stats 源统计对象；可为 NULL，NULL 时将有效 snapshot 清零。
 * @param[out] snapshot 调用方拥有的输出对象；可为 NULL，NULL 时整体不动作。
 * @return 无；仅执行结构体复制，不重置源计数。
 * 调用方式：低频日志或诊断读取前调用；需要一致快照时由上层临界区包围本函数。
 * 线程约束：无内部锁；不得与对同一 stats 的无保护写入并发，返回后 snapshot 可由调用方独占使用。
 */
void radar_telemetry_observability_snapshot(
    const radar_telemetry_observability_stats_t *stats,
    radar_telemetry_observability_stats_t *snapshot);

/**
 * @brief  将关键遥测计数格式化为适合 BLE 日志通道的有界单行文本。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  stats 待格式化的只读统计快照；不得为 NULL。
 * @param[out] output 调用方提供的字符缓冲；不得为 NULL，成功时包含 NUL 结尾字符串。
 * @param  output_capacity output 总容量，单位 byte，必须大于 0。
 * @return 文本完整写入返回 true；参数无效、格式化失败或缓冲不足返回 false。
 * 调用方式：先取得一致 snapshot，再由低频诊断路径调用；每个输出计数上限显示为 999999。
 * 线程约束：不修改 stats 且无内部锁；调用 snprintf，禁止 ISR/硬实时路径调用。
 * 所有权约束：不保留 stats/output 指针；false 时 output 内容可能是截断字符串，不得作为完整日志发送。
 */
bool radar_telemetry_observability_format_short_log(
    const radar_telemetry_observability_stats_t *stats,
    char *output,
    size_t output_capacity);

#ifdef __cplusplus
}
#endif

#endif /* S3_RADAR_TELEMETRY_OBSERVABILITY_H */
