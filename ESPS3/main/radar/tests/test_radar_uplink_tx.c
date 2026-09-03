#include <assert.h>
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "radar_uplink_tx.h"

/* 上行分片发送主机测试；创建人：待确认（当前维护人：Zhiqin）。 */

typedef struct {
    const int *results;
    size_t result_count;
    size_t result_index;
    uint8_t received[32];
    size_t received_length;
} mock_sender_t;

/**
 * @brief 按预置结果脚本模拟非阻塞 send，并复制正数返回对应的发送前缀。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param context 指向测试拥有的 mock_sender_t，不得为 NULL。
 * @param data 本次待发送片段的借用指针；正数结果时同步复制 result 字节。
 * @param length data 当前剩余长度，必须不小于脚本给出的正数结果。
 * @return 当前脚本项；负值时同时把 errno 设为 EAGAIN，正值表示已复制的字节数。
 * 调用方式：作为 radar_uplink_tx_send() 的 send_fn，同一 sender 按 result_index 顺序消费脚本。
 * 线程约束：单线程 host mock，修改 context 状态且不可并发；断言失败会终止测试进程，不保留 data。
 */
static int mock_send(void *context, const uint8_t *data, size_t length)
{
    mock_sender_t *sender = context;
    assert(sender->result_index < sender->result_count);
    const int result = sender->results[sender->result_index++];
    if (result < 0) {
        errno = EAGAIN;
        return result;
    }
    assert((size_t)result <= length);
    assert(sender->received_length + (size_t)result <= sizeof(sender->received));
    memcpy(&sender->received[sender->received_length], data, (size_t)result);
    sender->received_length += (size_t)result;
    return result;
}

/**
 * @brief 验证部分写后 EAGAIN 保留 offset，下一次调用从正确位置续发并重组原包。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；WAIT/COMPLETE、offset、重试数或最终字节不符时 assert 终止。
 * 调用方式：由 main() 调用；同一 packet/state/sender 连续调用两次发送状态机。
 * 线程约束：单线程主机状态机测试；所有缓冲和 mock context 在本函数返回前有效，不使用真实 socket。
 */
static void test_partial_write_and_eagain_preserve_packet_offset(void)
{
    static const int results[] = {3, -1, 7};
    static const uint8_t packet[] = {0U, 1U, 2U, 3U, 4U,
                                     5U, 6U, 7U, 8U, 9U};
    mock_sender_t sender = {
        .results = results,
        .result_count = sizeof(results) / sizeof(results[0]),
    };
    radar_uplink_tx_state_t state;
    radar_uplink_tx_reset(&state);

    assert(radar_uplink_tx_send(&state,
                                packet,
                                sizeof(packet),
                                mock_send,
                                &sender) == RADAR_UPLINK_TX_WAIT);
    assert(state.offset == 3U);
    assert(state.retry_count == 1U);
    assert(state.wrote_partial);
    assert(sender.received_length == 3U);

    assert(radar_uplink_tx_send(&state,
                                packet,
                                sizeof(packet),
                                mock_send,
                                &sender) == RADAR_UPLINK_TX_COMPLETE);
    assert(state.offset == sizeof(packet));
    assert(state.retry_count == 1U);
    assert(state.wrote_partial == false);
    assert(sender.received_length == sizeof(packet));
    assert(memcmp(sender.received, packet, sizeof(packet)) == 0);
}

/**
 * @brief 模拟不可重试的 EPIPE 发送失败。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param context mock 上下文，当前忽略，允许 NULL。
 * @param data 待发数据借用指针，当前不读取。
 * @param length 待发字节数，当前不读取。
 * @return 固定返回 -1，并把 errno 设置为 EPIPE。
 * 调用方式：仅作为永久失败测试的 radar_uplink_tx_send() 回调。
 * 线程约束：单线程 host mock；会修改线程局部/进程 errno，不保留任何指针。
 */
static int failing_send(void *context, const uint8_t *data, size_t length)
{
    (void)context;
    (void)data;
    (void)length;
    errno = EPIPE;
    return -1;
}

/**
 * @brief 验证不可重试 send 错误被报告为 FAILED 且发送 offset 不前进。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；结果或 offset 断言失败时终止测试进程。
 * 调用方式：由 main() 调用，使用 failing_send() 对单字节包执行一次状态机。
 * 线程约束：单线程纯 host mock，不覆盖真实 socket 关闭、重连或 errno 竞争。
 */
static void test_permanent_send_failure_is_reported(void)
{
    static const uint8_t packet[] = {1U};
    radar_uplink_tx_state_t state;
    radar_uplink_tx_reset(&state);

    assert(radar_uplink_tx_send(&state,
                                packet,
                                sizeof(packet),
                                failing_send,
                                NULL) == RADAR_UPLINK_TX_FAILED);
    assert(state.offset == 0U);
}

/**
 * @brief 模拟每次只成功写入一个字节并累计调用次数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param context 指向调用方拥有的 size_t 计数器，不得为 NULL。
 * @param data 待发片段借用指针，当前不读取。
 * @param length 当前剩余长度，必须大于 0，否则 assert 终止。
 * @return 固定返回 1，表示一次写入一个字节。
 * 调用方式：仅作为发送调用预算测试的 send_fn，多次同步调用同一计数器。
 * 线程约束：单线程 host mock，无锁修改 context，不保留 data。
 */
static int one_byte_send(void *context, const uint8_t *data, size_t length)
{
    size_t *calls = context;
    (void)data;
    assert(length > 0U);
    ++*calls;
    return 1;
}

/**
 * @brief 验证单轮 send 调用受 RADAR_UPLINK_TX_MAX_SEND_CALLS 限制，后续调用可完成剩余包。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 返回值：无（void）；调用预算、WAIT/COMPLETE、offset 或 retry 断言失败时终止进程。
 * 调用方式：由 main() 调用；对 32 字节包连续运行两轮 one_byte_send 状态机。
 * 线程约束：单线程 host 测试，不验证 FreeRTOS 调度让步、socket 可写事件或网络吞吐。
 */
static void test_send_call_budget_is_bounded(void)
{
    static const uint8_t packet[32] = {0};
    size_t calls = 0U;
    radar_uplink_tx_state_t state;
    radar_uplink_tx_reset(&state);

    assert(radar_uplink_tx_send(&state,
                                packet,
                                sizeof(packet),
                                one_byte_send,
                                &calls) == RADAR_UPLINK_TX_WAIT);
    assert(calls == RADAR_UPLINK_TX_MAX_SEND_CALLS);
    assert(state.offset == RADAR_UPLINK_TX_MAX_SEND_CALLS);
    assert(state.retry_count == 1U);

    assert(radar_uplink_tx_send(&state,
                                packet,
                                sizeof(packet),
                                one_byte_send,
                                &calls) == RADAR_UPLINK_TX_COMPLETE);
    assert(calls == sizeof(packet));
    assert(state.offset == sizeof(packet));
}

/**
 * @brief 顺序执行上行非阻塞分片发送状态机的主机 mock 测试。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * 传入参数：无。
 * @return 全部断言通过返回 0；任一 assert 失败会终止测试进程。
 * 调用方式：由 radar/tests/run_host_tests.sh 编译并直接运行。
 * 线程约束：单进程单线程，不创建 TCP socket 或上行任务；只验证纯发送状态机。
 */
int main(void)
{
    test_partial_write_and_eagain_preserve_packet_offset();
    test_permanent_send_failure_is_reported();
    test_send_call_budget_is_bounded();
    return 0;
}
