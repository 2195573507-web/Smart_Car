#ifndef S3_RADAR_UPLINK_TX_H
#define S3_RADAR_UPLINK_TX_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 非阻塞 TCP 分片发送状态机；创建人：待确认（当前维护人：Zhiqin）。 */
#define RADAR_UPLINK_TX_MAX_SEND_CALLS 16U

/**
 * @brief  非阻塞传输写回调类型。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  context radar_uplink_tx_send() 传入的原样上下文，可为 NULL。
 * @param  data 当前未发送片段，只在本次回调期间借用。
 * @param  length 本次最多可写字节数。
 * @return 1..length 表示实际写入；负值且 errno 为 EAGAIN/EWOULDBLOCK/EINTR 表示可重试；
 *         其他负值以及 0 都被当前状态机视为失败。不得返回大于 length 的值。
 * 调用方式：通常包装非阻塞 socket send()；回调必须正确保留 errno 供调用者判定。
 * 线程约束：由单一上行任务同步调用，不得无限阻塞、递归调用发送状态机或保存 data 指针。
 */
typedef int (*radar_uplink_send_fn_t)(void *context,
                                      const uint8_t *data,
                                      size_t length);

/** 单次非阻塞发送推进结果；WAIT 保留 offset，FAILED 由外层决定断线/丢包。 */
typedef enum {
    RADAR_UPLINK_TX_COMPLETE = 0, /**< 完整 data 已由 send 回调接受，offset 等于 length。 */
    RADAR_UPLINK_TX_WAIT,         /**< 遇到可重试错误或本轮调用预算耗尽，稍后续传。 */
    RADAR_UPLINK_TX_FAILED        /**< 参数、回调返回契约或不可重试 socket 错误。 */
} radar_uplink_tx_result_t;

/** 单个上行包的续传状态；由一个发送任务拥有，WAIT 期间必须绑定同一 data。 */
typedef struct {
    size_t offset; /**< 已被 send 回调接受的包前缀字节数，范围 0..length。 */
    uint32_t retry_count; /**< 返回 WAIT 的累计次数；32 位自然回绕，外层负责重试上限。 */
    bool wrote_partial; /**< 本次推进是否发生正数但小于剩余长度的部分写；下次调用会重置。 */
} radar_uplink_tx_state_t;

/**
 * @brief  清零一个上行包的偏移、等待计数和部分写标志。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[out] state 可为 NULL；非 NULL 时三个字段全部重置。
 * @return 无。
 * 调用方式：开始发送每条新包前调用；WAIT 后继续同一包时不要 reset，否则会重复发送前缀。
 * 线程约束：纯内存操作、无锁；同一个 state 只能由单一发送任务拥有。
 */
void radar_uplink_tx_reset(radar_uplink_tx_state_t *state);

/**
 * @brief 非阻塞推进一次分片发送。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[in,out] state 非 NULL 的单包状态；offset 不得大于 length。
 * @param  data 非 NULL 的完整包；只要结果为 WAIT，内容和地址就必须保持不变。
 * @param  length 完整包长度，必须大于 0。
 * @param  send_fn 非 NULL 的非阻塞写回调。
 * @param  context 原样传给 send_fn，不由状态机拥有。
 * @return COMPLETE 表示 offset 已到 length；WAIT 表示背压/中断或本轮达到最多 16 次写调用，
 *         offset 保留且 retry_count 增加；FAILED 表示参数或传输契约错误。
 * 调用方式：上行任务对同一包反复调用直至 COMPLETE/FAILED；本函数不限制总 retry_count，
 *           断线和最大重试策略由外层负责。FAILED 后不要换 data 继续沿用旧 offset。
 * 线程约束：单任务、不可重入、无内部锁；send_fn 必须非阻塞，禁止 ISR 调用。
 */
radar_uplink_tx_result_t radar_uplink_tx_send(
    radar_uplink_tx_state_t *state,
    const uint8_t *data,
    size_t length,
    radar_uplink_send_fn_t send_fn,
    void *context);

#endif
