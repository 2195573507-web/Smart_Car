#include "radar_frame_fifo.h"

/* 雷达原始帧 FIFO 实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <string.h>

/**
 * @brief  在调用方提供的条目数组上初始化完整雷达帧 FIFO。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[out] fifo 非 NULL 的 FIFO 状态；成功时整体清零并绑定 entries。
 * @param  entries 非 NULL、至少包含 capacity 个元素的外部存储；函数不接管释放责任。
 * @param  capacity entries 的元素个数，必须大于 0。
 * @return true 表示初始化完成；指针为空或容量为 0 时返回 false 且不修改 fifo。
 * 调用方式：雷达接收任务启动前调用一次，fifo 与 entries 生命周期必须覆盖后续全部访问。
 * 线程约束：纯内存操作、不阻塞、无内部锁；初始化期间禁止 push/pop/stats 并发访问。
 */
bool radar_frame_fifo_init(radar_frame_fifo_t *fifo,
                           radar_frame_fifo_entry_t *entries,
                           size_t capacity)
{
    if (fifo == NULL || entries == NULL || capacity == 0U) {
        return false;
    }

    memset(fifo, 0, sizeof(*fifo));
    fifo->entries = entries;
    fifo->capacity = capacity;
    return true;
}

/**
 * @brief  复制一条完整雷达帧到 FIFO，队满时先丢弃最旧项。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  fifo 已成功初始化的 FIFO。
 * @param  data 非 NULL 的源帧；函数同步复制 length 字节且不保留指针。
 * @param  length 帧长度，范围为 1..RADAR_PARSER_MAX_FRAME_SIZE。
 * @param  sequence 调用方生成的帧序号，原样保存。
 * @param  timestamp_ms 调用方采集时间，单位 ms，原样保存。
 * @return true 表示新帧已复制；参数或状态无效返回 false。满队列丢最旧项后仍返回 true。
 * 调用方式：解析器校验完整帧后调用；本函数不再次验证帧头、长度字段或校验和。
 * 线程约束：复制完整帧且无内部锁；生产者与消费者并发时必须由外层 mutex 串行化，禁止 ISR 调用。
 */
bool radar_frame_fifo_push(radar_frame_fifo_t *fifo,
                           const uint8_t *data,
                           size_t length,
                           uint32_t sequence,
                           uint32_t timestamp_ms)
{
    if (fifo == NULL || fifo->entries == NULL || fifo->capacity == 0U ||
        data == NULL || length == 0U ||
        length > RADAR_PARSER_MAX_FRAME_SIZE) {
        return false;
    }

    if (fifo->count == fifo->capacity) {
        fifo->tail = (fifo->tail + 1U) % fifo->capacity;
        --fifo->count;
        ++fifo->dropped_oldest_count;
    }

    radar_frame_fifo_entry_t *entry = &fifo->entries[fifo->head];
    memcpy(entry->data, data, length);
    entry->length = length;
    entry->sequence = sequence;
    entry->timestamp_ms = timestamp_ms;
    fifo->head = (fifo->head + 1U) % fifo->capacity;
    ++fifo->count;
    if (fifo->count > fifo->high_watermark) {
        fifo->high_watermark = fifo->count;
    }
    return true;
}

/**
 * @brief  将最旧完整帧复制到调用方缓冲并从 FIFO 消费。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  fifo 已成功初始化的 FIFO。
 * @param[out] data 非 NULL、至少可写 capacity 字节的输出缓冲。
 * @param  capacity data 的字节容量，必须大于 0。
 * @param[out] length 必须非 NULL；成功时写帧长，空队列写 0，短缓冲写所需长度。
 * @param[out] sequence 可为 NULL；成功时写队首序号。
 * @param[out] timestamp_ms 可为 NULL；成功时写队首采集时间，单位 ms。
 * @return true 表示已复制并消费；参数错误、空队列或短缓冲返回 false，后两者不消费队首。
 * 调用方式：上行消费者调用；false 时可根据 length 区分空队列与需要扩容的短缓冲。
 * 线程约束：复制完整帧且无内部锁；必须与 push/get_stats 使用同一外层 mutex，禁止 ISR 调用。
 */
bool radar_frame_fifo_pop(radar_frame_fifo_t *fifo,
                          uint8_t *data,
                          size_t capacity,
                          size_t *length,
                          uint32_t *sequence,
                          uint32_t *timestamp_ms)
{
    if (fifo == NULL || fifo->entries == NULL || fifo->capacity == 0U ||
        data == NULL || length == NULL || capacity == 0U) {
        return false;
    }
    if (fifo->count == 0U) {
        *length = 0U;
        return false;
    }

    const radar_frame_fifo_entry_t *entry = &fifo->entries[fifo->tail];
    if (entry->length > capacity) {
        *length = entry->length;
        return false;
    }

    memcpy(data, entry->data, entry->length);
    *length = entry->length;
    if (sequence != NULL) {
        *sequence = entry->sequence;
    }
    if (timestamp_ms != NULL) {
        *timestamp_ms = entry->timestamp_ms;
    }
    fifo->tail = (fifo->tail + 1U) % fifo->capacity;
    --fifo->count;
    return true;
}

/**
 * @brief  复制 FIFO 容量、当前深度、历史水位和丢最旧计数快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  fifo 可为 NULL；NULL 时输出保持为全零快照。
 * @param[out] stats 可为 NULL；非 NULL 时总会先整体清零再复制可用统计。
 * @return 无。
 * 调用方式：低频健康日志读取；统计只反映本地 FIFO，不证明 TCP/ROS2 已接收。
 * 线程约束：纯内存读取、无内部锁；与 push/pop 并发时必须由调用方加同一 mutex。
 */
void radar_frame_fifo_get_stats(const radar_frame_fifo_t *fifo,
                                radar_frame_fifo_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    memset(stats, 0, sizeof(*stats));
    if (fifo != NULL) {
        stats->capacity = fifo->capacity;
        stats->count = fifo->count;
        stats->high_watermark = fifo->high_watermark;
        stats->dropped_oldest_count = fifo->dropped_oldest_count;
    }
}
