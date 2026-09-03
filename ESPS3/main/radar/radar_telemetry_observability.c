#include "radar_telemetry_observability.h"

#include <inttypes.h>
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define RADAR_TELEMETRY_SHORT_LOG_COUNTER_MAX UINT32_C(999999)

/**
 * @brief  对单个 uint32_t 计数器执行 UINT32_MAX 饱和递增。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] value 待更新计数器；可为 NULL。
 * @return 无；指针为空或当前值已为 UINT32_MAX 时不动作。
 * 调用方式：本模块所有累计计数更新的内部公共辅助函数。
 * 线程约束：单次普通内存读改写、无原子性和内部锁；共享计数必须由上层临界区保护。
 */
static void saturating_increment(uint32_t *value)
{
    if (value != NULL && *value != UINT32_MAX) {
        ++(*value);
    }
}

/**
 * @brief  清零一份雷达遥测可观测统计对象。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[out] stats 待初始化对象；可为 NULL，非 NULL 时整体清零。
 * @return 无；stats 为 NULL 时静默不动作。
 * 调用方式：上行模块准备完成、telemetry sink 开始工作前调用一次。
 * 线程约束：调用 memset 且无内部锁；初始化阶段须串行执行，禁止与统计更新或快照并发。
 * 所有权约束：不分配内存、不保留指针，对象生命周期由调用方管理。
 */
void radar_telemetry_observability_init(
    radar_telemetry_observability_stats_t *stats)
{
    if (stats != NULL) {
        (void)memset(stats, 0, sizeof(*stats));
    }
}

/**
 * @brief  增加 telemetry sink 总调用次数，并按受支持的 SRPv4 消息类型分类。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] stats 已初始化统计对象；可为 NULL。
 * @param  message_id SRPv4 消息 ID；未知类型只累计总次数。
 * @return 无；stats 为 NULL 时不动作，相关计数在 UINT32_MAX 饱和。
 * 调用方式：telemetry sink 接到候选帧后、尝试队列锁和入队之前调用。
 * 线程约束：无内部锁；radar_uplink.c 通过 portMUX 临界区串行化共享对象，裸调用方须提供等价保护。
 */
void radar_telemetry_observability_note_sink_call(
    radar_telemetry_observability_stats_t *stats,
    uint16_t message_id)
{
    if (stats == NULL) {
        return;
    }

    saturating_increment(&stats->telemetry_sink_calls);
    switch (message_id) {
    case SRP_MSG_ID_WHEEL_SPEED_STATUS:
        saturating_increment(&stats->sink_wheel);
        break;
    case SRP_MSG_ID_ATTITUDE:
        saturating_increment(&stats->sink_attitude);
        break;
    case SRP_MSG_ID_IMU_TELEMETRY:
        saturating_increment(&stats->sink_imu);
        break;
    case SRP_MSG_ID_CHASSIS_STATE:
        saturating_increment(&stats->sink_chassis);
        break;
    default:
        break;
    }
}

/**
 * @brief  对 telemetry 外层队列锁竞争丢弃计数执行饱和递增。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] stats 已初始化统计对象；可为 NULL。
 * @return 无；stats 为 NULL 时不动作，计数在 UINT32_MAX 饱和。
 * 调用方式：queue push/pop/stats 的零等待 mutex 未取得时调用一次。
 * 线程约束：无内部锁、不阻塞；共享对象须由调用方临界区保护。
 */
void radar_telemetry_observability_note_lock_drop(
    radar_telemetry_observability_stats_t *stats)
{
    if (stats != NULL) {
        saturating_increment(&stats->telemetry_lock_drops);
    }
}

/**
 * @brief  将队列精确 push 结果转换为接受、覆盖与拒绝累计计数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] stats 已初始化统计对象；可为 NULL。
 * @param  result ACCEPTED 增加 accepted；OVERWRITTEN 增加 accepted 与 overwritten；
 *                REJECTED 或未知枚举值增加 rejected。
 * @return 无；stats 为 NULL 时不动作，所有计数在 UINT32_MAX 饱和。
 * 调用方式：telemetry sink 完成一次 radar_telemetry_queue_push_ex() 尝试后调用。
 * 线程约束：无内部锁、不阻塞；共享 stats 必须由统一临界区串行化。
 */
void radar_telemetry_observability_note_queue_result(
    radar_telemetry_observability_stats_t *stats,
    radar_telemetry_queue_push_result_t result)
{
    if (stats == NULL) {
        return;
    }

    if (result == RADAR_TELEMETRY_QUEUE_PUSH_ACCEPTED) {
        saturating_increment(&stats->queue_accepted);
    } else if (result == RADAR_TELEMETRY_QUEUE_PUSH_OVERWRITTEN) {
        saturating_increment(&stats->queue_accepted);
        saturating_increment(&stats->queue_overwritten);
    } else {
        saturating_increment(&stats->queue_rejected);
    }
}

/**
 * @brief  记录一次类型 2 遥测包成功完成外层封装。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] stats 已初始化统计对象；可为 NULL。
 * @return 无；stats 为 NULL 时不动作，计数在 UINT32_MAX 饱和。
 * 调用方式：上行任务把已弹出的 SRPv4 entry 编码为完整类型 2 包后调用。
 * 线程约束：无内部锁、不阻塞；共享对象由调用方临界区保护。
 * 语义边界：prepared 不等于 TCP 完整发送，也不证明 Windows/ROS2 已接收。
 */
void radar_telemetry_observability_note_packet_prepared(
    radar_telemetry_observability_stats_t *stats)
{
    if (stats != NULL) {
        saturating_increment(&stats->telemetry_packets_prepared);
    }
}

/**
 * @brief  根据类型 2 包发送状态累计完整发送成功或永久失败。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] stats 已初始化统计对象；可为 NULL。
 * @param  result COMPLETE 增加 sent，FAILED 增加 failure，WAIT 保持两者不变。
 * @return 无；stats 为 NULL 时不动作，计数在 UINT32_MAX 饱和。
 * 调用方式：上行任务每次调用 radar_uplink_tx_send() 推进类型 2 待发包后调用。
 * 线程约束：无内部锁、不阻塞；共享 stats 必须由外层临界区保护。
 * 语义边界：sent 仅表示本地整包 send 完成，不代表对端应用层已消费。
 */
void radar_telemetry_observability_note_type2_tx_result(
    radar_telemetry_observability_stats_t *stats,
    radar_uplink_tx_result_t result)
{
    if (stats == NULL) {
        return;
    }

    if (result == RADAR_UPLINK_TX_COMPLETE) {
        saturating_increment(&stats->telemetry_type2_sent);
    } else if (result == RADAR_UPLINK_TX_FAILED) {
        saturating_increment(&stats->telemetry_send_failures);
    }
}

/**
 * @brief  对超过上行时效限制的遥测丢弃计数执行饱和递增。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] stats 已初始化统计对象；可为 NULL。
 * @return 无；stats 为 NULL 时不动作，计数在 UINT32_MAX 饱和。
 * 调用方式：上行任务弹出 telemetry entry 后判定帧龄超限时调用。
 * 线程约束：无内部锁、不阻塞；共享对象必须由调用方临界区串行化。
 */
void radar_telemetry_observability_note_stale_drop(
    radar_telemetry_observability_stats_t *stats)
{
    if (stats != NULL) {
        saturating_increment(&stats->telemetry_stale_drops);
    }
}

/**
 * @brief  复制统计对象，源对象为空时生成全零快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  stats 源统计对象；可为 NULL。
 * @param[out] snapshot 调用方拥有的输出对象；可为 NULL，NULL 时整体不动作。
 * @return 无；不重置或修改有效的源对象。
 * 调用方式：低频日志格式化前获取瞬时副本；radar_uplink.c 在 portMUX 临界区内调用。
 * 线程约束：结构体复制本身无锁；若 stats 可被并发更新，调用方必须在整个复制期间持有同一锁。
 */
void radar_telemetry_observability_snapshot(
    const radar_telemetry_observability_stats_t *stats,
    radar_telemetry_observability_stats_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    if (stats == NULL) {
        (void)memset(snapshot, 0, sizeof(*snapshot));
    } else {
        *snapshot = *stats;
    }
}

/**
 * @brief  将显示用计数限制在 BLE 短日志的六位十进制范围内。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  value 原始 uint32_t 累计值。
 * @return value 不超过 999999 时原样返回，否则返回 999999。
 * 调用方式：radar_telemetry_observability_format_short_log() 格式化每个字段前调用。
 * 线程约束：纯值计算、可重入、不阻塞，可在任务上下文调用。
 */
static uint32_t short_log_counter(uint32_t value)
{
    return value > RADAR_TELEMETRY_SHORT_LOG_COUNTER_MAX
               ? RADAR_TELEMETRY_SHORT_LOG_COUNTER_MAX
               : value;
}

/**
 * @brief  将关键遥测计数格式化为有界的单行 BLE 诊断文本。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  stats 只读统计快照；不得为 NULL。
 * @param[out] output 调用方提供的字符缓冲；不得为 NULL。
 * @param  output_capacity output 总容量，单位 byte，必须大于 0。
 * @return snprintf 完整写入且未截断时为 true；参数无效、格式失败或容量不足时为 false。
 * 调用方式：上行低频可观测日志取得一致 snapshot 后调用；各显示计数最多为 999999。
 * 线程约束：无内部锁并调用 snprintf；禁止 ISR 或硬实时路径，同一 output 不得被并发写入。
 * 所有权约束：不保留输入/输出指针；false 时 output 可能包含截断内容，调用方不得发送该内容。
 */
bool radar_telemetry_observability_format_short_log(
    const radar_telemetry_observability_stats_t *stats,
    char *output,
    size_t output_capacity)
{
    int written;

    if (stats == NULL || output == NULL || output_capacity == 0U) {
        return false;
    }
    written = snprintf(
        output, output_capacity,
        "TELEM in=%" PRIu32 " c=%" PRIu32 " q=%" PRIu32
        " rej=%" PRIu32 " lock=%" PRIu32 " tx2=%" PRIu32
        " stale=%" PRIu32 " fail=%" PRIu32,
        short_log_counter(stats->telemetry_sink_calls),
        short_log_counter(stats->sink_chassis),
        short_log_counter(stats->queue_accepted),
        short_log_counter(stats->queue_rejected),
        short_log_counter(stats->telemetry_lock_drops),
        short_log_counter(stats->telemetry_type2_sent),
        short_log_counter(stats->telemetry_stale_drops),
        short_log_counter(stats->telemetry_send_failures));
    return written > 0 && (size_t)written < output_capacity;
}
