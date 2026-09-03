#include "radar_uplink_tx.h"

/* 非阻塞上行发送实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <errno.h>

/**
 * @brief  清零一个上行包的发送偏移、等待计数和部分写标志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[out] state 可为 NULL；非 NULL 时三个状态字段全部重置。
 * @return 无。
 * 调用方式：开始新包或明确放弃旧包时调用；WAIT 后继续同一包时不得 reset，避免重复发送前缀。
 * 线程约束：纯内存操作、无锁、不阻塞；同一 state 只能由单一发送任务拥有。
 */
void radar_uplink_tx_reset(radar_uplink_tx_state_t *state)
{
    if (state != NULL) {
        state->offset = 0U;
        state->retry_count = 0U;
        state->wrote_partial = false;
    }
}

/**
 * @brief  在有限 send 调用预算内推进一条包的非阻塞分片发送。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] state 非 NULL 的单包状态，进入时 offset 不得大于 length。
 * @param  data 非 NULL 的完整包；返回 WAIT 后其地址和内容必须保持不变。
 * @param  length 完整包字节数，必须大于 0。
 * @param  send_fn 非 NULL 的非阻塞写回调；正数不得超过请求长度，0 或超长返回视为失败，
 *                 负数仅在 errno 为 EAGAIN、EWOULDBLOCK 或 EINTR 时视为可重试。
 * @param  context 原样传给 send_fn，状态机不拥有该对象。
 * @return COMPLETE 表示全部写完；WAIT 表示可重试错误或本轮达到最多写调用数且 offset 已保留；
 *         FAILED 表示参数、回调返回契约或传输错误。
 * 调用方式：上行任务对同一 data/state 重复调用直至 COMPLETE/FAILED；本函数不限制累计 retry_count，
 *           断线和是否继续重试由外层状态机决定。
 * 线程约束：单任务、不可重入、无内部锁；send_fn 必须非阻塞并正确保留 errno，禁止 ISR 调用。
 */
radar_uplink_tx_result_t radar_uplink_tx_send(
    radar_uplink_tx_state_t *state,
    const uint8_t *data,
    size_t length,
    radar_uplink_send_fn_t send_fn,
    void *context)
{
    if (state == NULL || data == NULL || length == 0U || send_fn == NULL ||
        state->offset > length) {
        return RADAR_UPLINK_TX_FAILED;
    }

    state->wrote_partial = false;
    size_t send_calls = 0U;
    while (state->offset < length &&
           send_calls < RADAR_UPLINK_TX_MAX_SEND_CALLS) {
        ++send_calls;
        const size_t remaining = length - state->offset;
        const int written = send_fn(context, &data[state->offset], remaining);
        if (written > 0) {
            if ((size_t)written > remaining) {
                return RADAR_UPLINK_TX_FAILED;
            }
            if ((size_t)written < remaining) {
                state->wrote_partial = true;
            }
            state->offset += (size_t)written;
            continue;
        }
        if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK ||
                            errno == EINTR)) {
            ++state->retry_count;
            return RADAR_UPLINK_TX_WAIT;
        }
        return RADAR_UPLINK_TX_FAILED;
    }
    if (state->offset < length) {
        ++state->retry_count;
        return RADAR_UPLINK_TX_WAIT;
    }
    return RADAR_UPLINK_TX_COMPLETE;
}
