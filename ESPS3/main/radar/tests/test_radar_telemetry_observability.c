#include "radar_telemetry_observability.h"

#include <assert.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/**
 * @brief 验证 sink 总调用计数覆盖所有消息，而四类遥测消息分别递增对应分类计数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；任一 assert 失败时终止测试进程。
 * 调用方式：由 main() 调用；依次记录 wheel、attitude、IMU、chassis 和不分类的 LOG 消息。
 * 线程约束：单线程局部统计测试；不覆盖生产侧外层锁、SRP 接收任务或 UART2 数据流。
 */
static void test_sink_message_id_counters(void)
{
    radar_telemetry_observability_stats_t stats;

    radar_telemetry_observability_init(&stats);
    radar_telemetry_observability_note_sink_call(
        &stats, SRP_MSG_ID_WHEEL_SPEED_STATUS);
    radar_telemetry_observability_note_sink_call(
        &stats, SRP_MSG_ID_ATTITUDE);
    radar_telemetry_observability_note_sink_call(
        &stats, SRP_MSG_ID_IMU_TELEMETRY);
    radar_telemetry_observability_note_sink_call(
        &stats, SRP_MSG_ID_CHASSIS_STATE);
    radar_telemetry_observability_note_sink_call(&stats, SRP_MSG_ID_LOG);

    assert(stats.telemetry_sink_calls == 5U);
    assert(stats.sink_wheel == 1U);
    assert(stats.sink_attitude == 1U);
    assert(stats.sink_imu == 1U);
    assert(stats.sink_chassis == 1U);
}

/**
 * @brief 验证 accepted、overwritten 和 rejected 三类入队结果的累计统计语义。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；accepted=2、overwritten=1 或 rejected=1 不成立时 assert 终止进程。
 * 调用方式：由 main() 调用；初始化局部 stats 后各记录一次队列结果。
 * 线程约束：单线程纯统计测试；不创建真实遥测队列，也不覆盖生产者锁竞争。
 */
static void test_queue_result_counters(void)
{
    radar_telemetry_observability_stats_t stats;

    radar_telemetry_observability_init(&stats);
    radar_telemetry_observability_note_queue_result(
        &stats, RADAR_TELEMETRY_QUEUE_PUSH_ACCEPTED);
    radar_telemetry_observability_note_queue_result(
        &stats, RADAR_TELEMETRY_QUEUE_PUSH_OVERWRITTEN);
    radar_telemetry_observability_note_queue_result(
        &stats, RADAR_TELEMETRY_QUEUE_PUSH_REJECTED);

    assert(stats.queue_accepted == 2U);
    assert(stats.queue_overwritten == 1U);
    assert(stats.queue_rejected == 1U);
}

/**
 * @brief 验证 type-2 上行仅在 COMPLETE 时计为发送成功，WAIT 不计数，FAILED 只累计失败。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；prepared、sent 或 failure 计数语义不符时 assert 终止进程。
 * 调用方式：由 main() 调用；记录两次 packet prepared，并依次注入 WAIT、FAILED、COMPLETE。
 * 线程约束：单线程局部统计测试；不建立 TCP 连接，也不证明类型 2 包到达服务端或 ROS2。
 */
static void test_type2_completion_semantics(void)
{
    radar_telemetry_observability_stats_t stats;

    radar_telemetry_observability_init(&stats);
    radar_telemetry_observability_note_packet_prepared(&stats);
    radar_telemetry_observability_note_packet_prepared(&stats);
    radar_telemetry_observability_note_type2_tx_result(
        &stats, RADAR_UPLINK_TX_WAIT);
    assert(stats.telemetry_type2_sent == 0U);
    assert(stats.telemetry_send_failures == 0U);

    radar_telemetry_observability_note_type2_tx_result(
        &stats, RADAR_UPLINK_TX_FAILED);
    assert(stats.telemetry_type2_sent == 0U);
    assert(stats.telemetry_send_failures == 1U);

    radar_telemetry_observability_note_type2_tx_result(
        &stats, RADAR_UPLINK_TX_COMPLETE);
    assert(stats.telemetry_packets_prepared == 2U);
    assert(stats.telemetry_type2_sent == 1U);
    assert(stats.telemetry_send_failures == 1U);
}

typedef struct {
    size_t call_count;
} type2_send_context_t;

/**
 * @brief 模拟非阻塞 send：首次部分写入、第二次以 EAGAIN 暂停，后续写完请求长度。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param[in,out] context 非 NULL 的 type2_send_context_t，调用时累计 call_count。
 * @param[in] data 本次待发送数据的借用只读指针，不得为 NULL，函数不保留。
 * @param length 本次待发送字节数；测试路径保证可安全转换为 int。
 * @return 第一次返回 length/2，第二次设置 errno=EAGAIN 并返回 -1，第三次起返回完整 length。
 * 调用方式：仅作为 radar_uplink_tx_send() 的同步 send callback，用于驱动 partial/WAIT/恢复路径。
 * 线程约束：单线程测试 stub；会修改共享 context 和 errno，不可并发复用同一上下文。
 */
static int type2_partial_then_wait_send(void *context,
                                        const uint8_t *data,
                                        size_t length)
{
    type2_send_context_t *send_context = context;

    assert(send_context != NULL);
    assert(data != NULL);
    ++send_context->call_count;
    if (send_context->call_count == 1U) {
        assert(length >= 2U);
        return (int)(length / 2U);
    }
    if (send_context->call_count == 2U) {
        errno = EAGAIN;
        return -1;
    }
    return (int)length;
}

/**
 * @brief 验证一个 type-2 包在部分发送和 EAGAIN 后保留 offset，并在重试完整发送时只计一次成功。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；WAIT/COMPLETE、offset 或成功计数不符时 assert 终止进程。
 * 调用方式：由 main() 调用；对同一 tx_state 和 packet 连续调用两次 radar_uplink_tx_send()。
 * 线程约束：单线程纯内存集成测试；packet、tx_state 与 callback context 均由本函数独占，不访问 socket。
 */
static void test_type2_full_packet_integration(void)
{
    static const uint8_t packet[] = {1U, 2U, 3U, 4U, 5U, 6U};
    radar_telemetry_observability_stats_t stats;
    radar_uplink_tx_state_t tx_state = {0};
    type2_send_context_t send_context = {0};
    radar_uplink_tx_result_t result;

    radar_telemetry_observability_init(&stats);
    result = radar_uplink_tx_send(&tx_state, packet, sizeof(packet),
                                  type2_partial_then_wait_send,
                                  &send_context);
    assert(result == RADAR_UPLINK_TX_WAIT);
    assert(tx_state.offset > 0U && tx_state.offset < sizeof(packet));
    radar_telemetry_observability_note_type2_tx_result(&stats, result);
    assert(stats.telemetry_type2_sent == 0U);

    result = radar_uplink_tx_send(&tx_state, packet, sizeof(packet),
                                  type2_partial_then_wait_send,
                                  &send_context);
    assert(result == RADAR_UPLINK_TX_COMPLETE);
    assert(tx_state.offset == sizeof(packet));
    radar_telemetry_observability_note_type2_tx_result(&stats, result);
    assert(stats.telemetry_type2_sent == 1U);
}

/**
 * @brief 验证短日志的固定字段格式、极大计数的有界输出，以及容量不足时明确失败。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；格式字符串、最大长度或短缓冲返回语义不符时 assert 终止进程。
 * 调用方式：由 main() 调用；先检查代表性统计的精确文本，再以全 0xFF 统计测试 97/92 字节容量。
 * 线程约束：单线程局部缓冲格式化测试；不覆盖 ESP 日志锁、并发快照或串口/TCP 输出。
 */
static void test_short_log_is_bounded(void)
{
    radar_telemetry_observability_stats_t stats;
    char output[97];

    radar_telemetry_observability_init(&stats);
    stats.telemetry_sink_calls = 120U;
    stats.sink_chassis = 20U;
    stats.queue_accepted = 118U;
    stats.queue_rejected = 2U;
    stats.telemetry_type2_sent = 55U;
    assert(radar_telemetry_observability_format_short_log(
        &stats, output, sizeof(output)));
    assert(strcmp(output,
                  "TELEM in=120 c=20 q=118 rej=2 lock=0 tx2=55 stale=0 fail=0") == 0);

    (void)memset(&stats, 0xFF, sizeof(stats));
    assert(radar_telemetry_observability_format_short_log(
        &stats, output, sizeof(output)));
    assert(strlen(output) <= 96U);
    assert(!radar_telemetry_observability_format_short_log(
        &stats, output, 92U));
}

/**
 * @brief 验证全部 13 个可观测计数在 UINT32_MAX 时继续记录事件仍保持饱和，并可原样快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；结构尺寸或任一计数快照不是 UINT32_MAX 时 assert 终止进程。
 * 调用方式：由 main() 调用；白盒填满 stats 后覆盖各 note API，再按 13 个 uint32_t 检查 snapshot。
 * 线程约束：单线程白盒 host 测试；不证明无锁并发快照的一致性或 32 位 MCU 上的同步边界。
 */
static void test_saturating_counters(void)
{
    radar_telemetry_observability_stats_t stats;
    radar_telemetry_observability_stats_t snapshot;

    stats = (radar_telemetry_observability_stats_t){
        .telemetry_sink_calls = UINT32_MAX,
        .sink_wheel = UINT32_MAX,
        .sink_attitude = UINT32_MAX,
        .sink_imu = UINT32_MAX,
        .sink_chassis = UINT32_MAX,
        .telemetry_lock_drops = UINT32_MAX,
        .queue_accepted = UINT32_MAX,
        .queue_rejected = UINT32_MAX,
        .queue_overwritten = UINT32_MAX,
        .telemetry_packets_prepared = UINT32_MAX,
        .telemetry_type2_sent = UINT32_MAX,
        .telemetry_stale_drops = UINT32_MAX,
        .telemetry_send_failures = UINT32_MAX,
    };

    radar_telemetry_observability_note_sink_call(
        &stats, SRP_MSG_ID_WHEEL_SPEED_STATUS);
    radar_telemetry_observability_note_sink_call(&stats, SRP_MSG_ID_ATTITUDE);
    radar_telemetry_observability_note_sink_call(
        &stats, SRP_MSG_ID_IMU_TELEMETRY);
    radar_telemetry_observability_note_sink_call(&stats, SRP_MSG_ID_CHASSIS_STATE);
    radar_telemetry_observability_note_lock_drop(&stats);
    radar_telemetry_observability_note_queue_result(
        &stats, RADAR_TELEMETRY_QUEUE_PUSH_OVERWRITTEN);
    radar_telemetry_observability_note_queue_result(
        &stats, RADAR_TELEMETRY_QUEUE_PUSH_REJECTED);
    radar_telemetry_observability_note_packet_prepared(&stats);
    radar_telemetry_observability_note_type2_tx_result(
        &stats, RADAR_UPLINK_TX_COMPLETE);
    radar_telemetry_observability_note_type2_tx_result(
        &stats, RADAR_UPLINK_TX_FAILED);
    radar_telemetry_observability_note_stale_drop(&stats);
    radar_telemetry_observability_snapshot(&stats, &snapshot);

    uint32_t values[13];
    _Static_assert(sizeof(values) == sizeof(snapshot),
                   "observability stats must contain 13 uint32 counters");
    (void)memcpy(values, &snapshot, sizeof(values));
    for (size_t index = 0U;
         index < sizeof(snapshot) / sizeof(values[0]);
         ++index) {
        assert(values[index] == UINT32_MAX);
    }
}

/**
 * @brief 顺序执行雷达遥测可观测计数、type-2 完成语义、短日志边界和饱和统计测试。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 全部断言通过返回 0；任一 assert 失败会中止进程且不会打印 PASS。
 * 调用方式：由 main/radar/tests/run_host_tests.sh 编译并运行普通版与 sanitizer 版；无命令行参数。
 * 线程约束：单进程单线程 host 测试，不访问 UART1/GPIO44、Wi-Fi、TCP 或真实雷达。
 */
int main(void)
{
    test_sink_message_id_counters();
    test_queue_result_counters();
    test_type2_completion_semantics();
    test_type2_full_packet_integration();
    test_short_log_is_bounded();
    test_saturating_counters();
    (void)printf("radar telemetry observability tests: PASS\n");
    return 0;
}
