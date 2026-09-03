#include "s3_ble_log_tx.h"

#include <limits.h>
#include <string.h>

_Static_assert(S3_BLE_LOG_TX_NORMAL_CAPACITY > 0U,
               "normal log queue must not be empty");
_Static_assert(S3_BLE_LOG_TX_CRITICAL_CAPACITY > 0U,
               "critical log queue must not be empty");
_Static_assert(S3_BLE_LOG_TX_NORMAL_CAPACITY <= UINT8_MAX,
               "normal log queue index must fit in uint8_t");
_Static_assert(S3_BLE_LOG_TX_CRITICAL_CAPACITY <= UINT8_MAX,
               "critical log queue index must fit in uint8_t");
_Static_assert(S3_BLE_LOG_TX_CRITICAL_BURST_MAX > 0U,
               "critical burst limit must not be zero");

/**
 * @brief 对内部 32 位统计执行饱和加一，保持 UINT32_MAX 不回绕。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param value 非 NULL 可写统计指针；函数不保存该指针。
 * @return 无。
 * 调用方式：仅由本文件的队列、拥塞、发送完成和中止路径调用。
 * 线程约束：函数自身不加锁；调用方须已独占 tx 或持有其统一外部锁，禁止直接并发调用。
 */
static void saturating_increment(uint32_t *value)
{
    if (*value != UINT32_MAX) {
        ++(*value);
    }
}

/**
 * @brief 刷新两个等待队列的当前深度并维护历史高水位。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param tx 非 NULL 可写状态对象；active 帧不计入深度。
 * @return 无。
 * 调用方式：完成入队或从等待队列选出 active 帧后调用。
 * 线程约束：函数内部不加锁；调用方须已独占对象或持有统一外部锁，禁止 ISR 并发访问。
 */
static void update_depth(s3_ble_log_tx_t *tx)
{
    const uint32_t depth = (uint32_t)tx->normal_count +
                           (uint32_t)tx->critical_count;

    tx->stats.current_depth = depth;
    if (depth > tx->stats.high_watermark) {
        tx->stats.high_watermark = depth;
    }
}

/**
 * @brief 中止当前 active 帧，并在已准备或已完成部分分片时累计 partial_drop。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param tx 非 NULL 可写状态对象；等待队列以及在途 token/length 不会在此函数清除。
 * @return 无；无 active 帧时保持状态不变。
 * 调用方式：断连或关闭 FFE3 CCC 时调用；迟到的 complete 仍可凭保留 token 返回 ABORTED。
 * 线程约束：函数内部不加锁；调用方须持有统一外部锁或串行独占对象，禁止 ISR 并发访问。
 */
static void abort_active_frame(s3_ble_log_tx_t *tx)
{
    if (!tx->active_valid) {
        return;
    }

    if (tx->active_offset > 0U || tx->chunk_in_flight) {
        saturating_increment(&tx->stats.partial_drop);
    }
    tx->active_valid = false;
    tx->active_offset = 0U;
}

/**
 * @brief 将完整帧复制进一个固定环形队列，队满时先覆盖最旧帧并累计丢弃。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param entries 非 NULL 环形队列存储，由 tx 对象拥有。
 * @param capacity 队列容量，必须大于 0 且索引可由 uint8_t 表示。
 * @param head 非 NULL 下一写入索引指针。
 * @param tail 非 NULL 最旧帧索引指针。
 * @param count 非 NULL 当前帧数指针。
 * @param drop_count 非 NULL 同级队列覆盖丢弃的饱和累计计数。
 * @param frame 调用方只读帧；函数返回前有效即可，内容会复制到 entries。
 * @param length 要复制的有效字节数；上层已保证不超过单帧存储。
 * @return 无。
 * 调用方式：仅由 s3_ble_log_tx_enqueue() 在参数校验完成后按优先级调用。
 * 线程约束：内部不加锁且执行有界 memcpy；调用方须独占状态对象，禁止并发和 ISR 调用。
 */
static void enqueue_frame(s3_ble_log_tx_frame_t *entries,
                          uint8_t capacity,
                          uint8_t *head,
                          uint8_t *tail,
                          uint8_t *count,
                          uint32_t *drop_count,
                          const uint8_t *frame,
                          uint16_t length)
{
    if (*count >= capacity) {
        *tail = (uint8_t)((*tail + 1U) % capacity);
        --(*count);
        saturating_increment(drop_count);
    }

    entries[*head].length = length;
    (void)memcpy(entries[*head].data, frame, length);
    *head = (uint8_t)((*head + 1U) % capacity);
    ++(*count);
}

/**
 * @brief 按“关键优先、连续四帧后让普通帧一次”的规则选择下一完整帧为 active。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param tx 非 NULL 可写状态对象；调用前应不存在有效 active 帧。
 * @return 成功取出一帧返回 true；两个等待队列均空时返回 false。
 * 调用方式：prepare 在 active 无效时调用；成功会复制完整帧、递减对应等待队列并刷新深度。
 * 线程约束：函数内部不加锁；调用方须持有统一外部锁或串行独占对象，禁止 ISR 并发访问。
 */
static bool pop_next_frame(s3_ble_log_tx_t *tx)
{
    const bool take_critical =
        tx->critical_count > 0U &&
        (tx->normal_count == 0U ||
         tx->critical_burst_count < S3_BLE_LOG_TX_CRITICAL_BURST_MAX);

    if (take_critical) {
        tx->active = tx->critical[tx->critical_tail];
        tx->critical_tail = (uint8_t)((tx->critical_tail + 1U) %
                                      S3_BLE_LOG_TX_CRITICAL_CAPACITY);
        --tx->critical_count;
        if (tx->critical_burst_count < S3_BLE_LOG_TX_CRITICAL_BURST_MAX) {
            ++tx->critical_burst_count;
        }
    } else if (tx->normal_count > 0U) {
        tx->active = tx->normal[tx->normal_tail];
        tx->normal_tail = (uint8_t)((tx->normal_tail + 1U) %
                                    S3_BLE_LOG_TX_NORMAL_CAPACITY);
        --tx->normal_count;
        tx->critical_burst_count = 0U;
    } else if (tx->critical_count > 0U) {
        tx->active = tx->critical[tx->critical_tail];
        tx->critical_tail = (uint8_t)((tx->critical_tail + 1U) %
                                      S3_BLE_LOG_TX_CRITICAL_CAPACITY);
        --tx->critical_count;
        if (tx->critical_burst_count < S3_BLE_LOG_TX_CRITICAL_BURST_MAX) {
            ++tx->critical_burst_count;
        }
    } else {
        tx->critical_burst_count = 0U;
        return false;
    }

    tx->active_valid = true;
    tx->active_offset = 0U;
    update_depth(tx);
    return true;
}

/**
 * @brief 生成下一个非零在途 token，32 位回绕后从 1 继续。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param tx 非 NULL 可写状态对象。
 * @return 新生成的非零 token。
 * 调用方式：每次成功准备 notification 分片时调用一次，用于拒绝迟到或重复完成事件。
 * 线程约束：函数内部不加锁；调用方须独占对象，且只允许唯一 TX worker 推进 token。
 */
static uint32_t next_nonzero_token(s3_ble_log_tx_t *tx)
{
    ++tx->next_token;
    if (tx->next_token == 0U) {
        tx->next_token = 1U;
    }
    return tx->next_token;
}

/**
 * @brief 将 BLE 日志发送状态、双队列和累计统计全部清零。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param tx 待初始化的可写对象；允许 NULL，NULL 时不执行操作。
 * @return 无。
 * 调用方式：由 BLE 初始化路径在创建 worker 和首次入队前调用；重调会丢弃对象内现有状态。
 * 线程约束：函数内部不加锁；调用时须独占对象，禁止与生产者、worker 或 GATT 回调并发，禁止 ISR。
 */
void s3_ble_log_tx_init(s3_ble_log_tx_t *tx)
{
    if (tx != NULL) {
        (void)memset(tx, 0, sizeof(*tx));
    }
}

/**
 * @brief 校验并复制一条完整帧到普通或关键固定环形队列。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param tx 已初始化的可写状态对象，不得为 NULL。
 * @param priority 普通或关键队列优先级，其他值无效。
 * @param frame 调用方拥有的只读帧；成功时复制，不转移所有权。
 * @param length 有效字节数，范围为 1..S3_BLE_LOG_TX_MAX_FRAME_SIZE。
 * @return 参数有效且完成入队返回 true；参数非法返回 false；队满覆盖同级最旧帧仍返回 true。
 * 调用方式：生产者编码完整 SmartCarLog 帧后调用，随后由外层唤醒唯一 TX worker。
 * 线程约束：内部不加锁、不等待；固件调用点须持有统一短临界区，禁止硬件 ISR。
 */
bool s3_ble_log_tx_enqueue(s3_ble_log_tx_t *tx,
                           s3_ble_log_tx_priority_t priority,
                           const uint8_t *frame,
                           uint16_t length)
{
    if (tx == NULL || frame == NULL || length == 0U ||
        length > S3_BLE_LOG_TX_MAX_FRAME_SIZE ||
        priority > S3_BLE_LOG_TX_PRIORITY_CRITICAL) {
        return false;
    }

    if (priority == S3_BLE_LOG_TX_PRIORITY_CRITICAL) {
        enqueue_frame(tx->critical, S3_BLE_LOG_TX_CRITICAL_CAPACITY,
                      &tx->critical_head, &tx->critical_tail,
                      &tx->critical_count, &tx->stats.drop_critical,
                      frame, length);
    } else {
        enqueue_frame(tx->normal, S3_BLE_LOG_TX_NORMAL_CAPACITY,
                      &tx->normal_head, &tx->normal_tail,
                      &tx->normal_count, &tx->stats.drop_normal,
                      frame, length);
    }
    saturating_increment(&tx->stats.queued);
    update_depth(tx);
    return true;
}

/**
 * @brief 更新连接门；断连时中止 active 帧、关闭 CCC 并清除拥塞状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param tx 已初始化的可写状态对象；允许 NULL，NULL 时静默返回。
 * @param connected 当前 GATT 连接状态；false 不会清空两个等待队列。
 * @return 无。
 * 调用方式：GATT connect/disconnect 事件在更新外层状态时调用；迟到 complete 由 token 状态处理。
 * 线程约束：内部不加锁；须与 worker 和生产者使用同一外部临界区，禁止 ISR。
 */
void s3_ble_log_tx_set_connected(s3_ble_log_tx_t *tx, bool connected)
{
    if (tx == NULL) {
        return;
    }

    if (!connected) {
        abort_active_frame(tx);
        tx->ccc_enabled = false;
        tx->congested = false;
    } else if (!tx->connected) {
        tx->congested = false;
    }
    tx->connected = connected;
}

/**
 * @brief 更新 FFE3 CCC 通知门；关闭时中止 active 帧但保留等待队列。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param tx 已初始化的可写状态对象；允许 NULL，NULL 时静默返回。
 * @param enabled true 允许 prepare，false 暂停发送并中止 active 帧。
 * @return 无。
 * 调用方式：处理 FFE3 CCC 写事件后调用，再由外层唤醒 worker 重新评估发送门。
 * 线程约束：内部不加锁；须由统一外部临界区保护，禁止 ISR。
 */
void s3_ble_log_tx_set_ccc_enabled(s3_ble_log_tx_t *tx, bool enabled)
{
    if (tx == NULL) {
        return;
    }

    if (!enabled) {
        abort_active_frame(tx);
    }
    tx->ccc_enabled = enabled;
}

/**
 * @brief 更新 GATT 拥塞门，并对每次非 NULL 调用累计一个 congest_events。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param tx 已初始化的可写状态对象；允许 NULL，NULL 时不更新也不计数。
 * @param congested true 暂停后续 prepare，false 恢复；active 和等待队列均保留。
 * @return 无。
 * 调用方式：匹配当前连接的 GATT CONGEST 事件调用；解除后由外层唤醒 worker。
 * 线程约束：内部不加锁；须由统一外部临界区保护，禁止 ISR。
 */
void s3_ble_log_tx_set_congested(s3_ble_log_tx_t *tx, bool congested)
{
    if (tx == NULL) {
        return;
    }

    saturating_increment(&tx->stats.congest_events);
    tx->congested = congested;
}

/**
 * @brief 在发送门允许时选取 active 帧并复制其下一个 notification 分片。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param tx 已初始化的可写状态对象，不得为 NULL。
 * @param max_chunk_size 本次最大分片长度，单位 byte；0 无效，通常为 ATT_MTU-3。
 * @param chunk 调用方拥有的可写输出；仅 READY 时其 token、偏移、长度、末片标志和 data 有效。
 * @return 返回 READY、EMPTY、PAUSED、BUSY 或 INVALID_ARG；READY 时创建唯一在途 token。
 * 调用方式：唯一 FFE3 TX worker 在 GATT 提交前调用，并为 READY 结果配对一次 complete。
 * 线程约束：内部不加锁；须在统一外部临界区内调用，复制完成后 chunk 可在锁外提交，禁止 ISR。
 */
s3_ble_log_tx_prepare_result_t s3_ble_log_tx_prepare_chunk(
    s3_ble_log_tx_t *tx,
    uint16_t max_chunk_size,
    s3_ble_log_tx_chunk_t *chunk)
{
    uint16_t remaining;

    if (tx == NULL || chunk == NULL || max_chunk_size == 0U) {
        return S3_BLE_LOG_TX_PREPARE_INVALID_ARG;
    }
    if (!tx->connected || !tx->ccc_enabled || tx->congested) {
        return S3_BLE_LOG_TX_PREPARE_PAUSED;
    }
    if (tx->chunk_in_flight) {
        return S3_BLE_LOG_TX_PREPARE_BUSY;
    }
    if (!tx->active_valid && !pop_next_frame(tx)) {
        return S3_BLE_LOG_TX_PREPARE_EMPTY;
    }

    remaining = (uint16_t)(tx->active.length - tx->active_offset);
    chunk->length = remaining < max_chunk_size ? remaining : max_chunk_size;
    chunk->frame_offset = tx->active_offset;
    chunk->frame_end = chunk->length == remaining;
    chunk->token = next_nonzero_token(tx);
    (void)memcpy(chunk->data, &tx->active.data[tx->active_offset],
                 chunk->length);

    tx->chunk_in_flight = true;
    tx->in_flight_length = chunk->length;
    tx->in_flight_token = chunk->token;
    return S3_BLE_LOG_TX_PREPARE_READY;
}

/**
 * @brief 校验在途 token，并按 GATT 提交结果推进偏移、完成整帧或放弃 active 帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param tx 已初始化的可写状态对象，不得为 NULL。
 * @param token prepare 返回的非零 token；0、迟到或不匹配值不消费当前在途状态。
 * @param send_succeeded true 计入 sent_chunks，且 active 仍有效时推进；false 计入 send_fail 并放弃有效 active 帧，
 *                       仅 active_offset 已有成功前缀时增加 partial_drop。
 * @return 返回 MORE、FRAME_DONE、FRAME_DROPPED、ABORTED 或 STALE；具体状态见公开枚举。
 * 调用方式：唯一 worker 在一次 send_indicate() 返回后调用；底层提交成功仍不代表手机已收到。
 * 线程约束：内部不加锁；须与 prepare、断连和 CCC 更新使用同一外部临界区，禁止 ISR。
 */
s3_ble_log_tx_complete_result_t s3_ble_log_tx_complete_chunk(
    s3_ble_log_tx_t *tx,
    uint32_t token,
    bool send_succeeded)
{
    uint16_t completed_length;

    if (tx == NULL || !tx->chunk_in_flight ||
        token == 0U || token != tx->in_flight_token) {
        return S3_BLE_LOG_TX_COMPLETE_STALE;
    }

    completed_length = tx->in_flight_length;
    if (send_succeeded) {
        saturating_increment(&tx->stats.sent_chunks);
    } else {
        saturating_increment(&tx->stats.send_fail);
    }
    tx->chunk_in_flight = false;
    tx->in_flight_length = 0U;
    tx->in_flight_token = 0U;

    if (!tx->active_valid) {
        return S3_BLE_LOG_TX_COMPLETE_ABORTED;
    }
    if (!send_succeeded) {
        if (tx->active_offset > 0U) {
            saturating_increment(&tx->stats.partial_drop);
        }
        tx->active_valid = false;
        tx->active_offset = 0U;
        return S3_BLE_LOG_TX_COMPLETE_FRAME_DROPPED;
    }

    tx->active_offset = (uint16_t)(tx->active_offset +
                                   completed_length);
    if (tx->active_offset >= tx->active.length) {
        tx->active_valid = false;
        tx->active_offset = 0U;
        saturating_increment(&tx->stats.sent_frames);
        return S3_BLE_LOG_TX_COMPLETE_FRAME_DONE;
    }
    return S3_BLE_LOG_TX_COMPLETE_MORE;
}

/**
 * @brief 复制本地队列和 GATT 提交统计快照，读取不清零。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（函数契约补充）。
 * @param tx 已初始化的只读状态对象；允许 NULL，NULL 时不写输出。
 * @param stats 调用方拥有的可写输出；允许 NULL，NULL 时不执行复制。
 * @return 无；任一参数为 NULL 时保持输出不变。
 * 调用方式：由低频诊断路径读取；current_depth 不含 active，所有发送计数只代表本地提交状态。
 * 线程约束：内部不加锁；固件调用须在统一外部临界区内取得一致快照，host test 可串行调用，禁止 ISR。
 */
void s3_ble_log_tx_get_stats(const s3_ble_log_tx_t *tx,
                             s3_ble_log_tx_stats_t *stats)
{
    if (tx != NULL && stats != NULL) {
        *stats = tx->stats;
    }
}
