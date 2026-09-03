#ifndef S3_RADAR_FRAME_FIFO_H
#define S3_RADAR_FRAME_FIFO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "radar_parser.h"

/*
 * 雷达完整帧有界 FIFO。
 * 创建人：待确认（当前维护人：Zhiqin）。
 * 只复制已校验的原始帧，不解析测量字段，也不直接访问 UART 或网络。
 */

/* 256 条完整帧为短时 TCP 阻塞提供有界 PSRAM 缓冲。 */
#define RADAR_FRAME_FIFO_DEPTH 256U

/** FIFO 中一条完整雷达原始帧；结构体自身拥有 data 字节副本。 */
typedef struct {
    uint8_t data[RADAR_PARSER_MAX_FRAME_SIZE]; /**< 已校验原始帧存储，仅前 length 字节有效。 */
    size_t length; /**< data 有效长度，单位 byte，范围 1..RADAR_PARSER_MAX_FRAME_SIZE。 */
    uint32_t sequence; /**< 调用方分配的原始帧序号，按 32 位自然回绕。 */
    uint32_t timestamp_ms; /**< UART 接收侧采集时间，单位单调 ms。 */
} radar_frame_fifo_entry_t;

/** 有界环形 FIFO 状态；entries 存储由调用方拥有并覆盖整个使用期。 */
typedef struct {
    radar_frame_fifo_entry_t *entries; /**< 借用的外部条目数组，FIFO 不分配或释放。 */
    size_t capacity; /**< entries 元素个数，必须大于 0。 */
    size_t head; /**< 下一次 push 的零基写入索引。 */
    size_t tail; /**< 当前最旧待取条目的零基索引。 */
    size_t count; /**< 当前待取条目数，范围 0..capacity。 */
    size_t high_watermark; /**< 初始化以来 count 的历史最大值。 */
    uint32_t dropped_oldest_count; /**< 队满时为接收新帧而丢弃最旧项的累计次数。 */
} radar_frame_fifo_t;

/** 对外只读 FIFO 统计快照；只反映本地缓存，不证明 TCP/ROS2 已接收。 */
typedef struct {
    size_t capacity; /**< FIFO 可容纳的条目数。 */
    size_t count; /**< 读取快照时的当前待取条目数。 */
    size_t high_watermark; /**< 初始化以来的历史最大深度。 */
    uint32_t dropped_oldest_count; /**< 因队满被覆盖丢弃的最旧帧累计数。 */
} radar_frame_fifo_stats_t;

/**
 * @brief 在调用方提供的数组上初始化 FIFO。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param[out] fifo 非 NULL 的 FIFO 状态对象；成功时整体清零并绑定 entries。
 * @param  entries 非 NULL、至少包含 capacity 个元素的调用方存储；函数不接管释放责任。
 * @param  capacity entries 的元素个数，必须大于 0；不强制等于 RADAR_FRAME_FIFO_DEPTH。
 * @return true 表示绑定完成；任一指针为空或 capacity 为 0 时返回 false。
 * 调用方式：雷达接收任务启动前调用；fifo 和 entries 生命周期必须覆盖全部 push/pop。
 * 线程约束：纯内存操作、不阻塞、无内部锁；初始化期间不得有并发访问。
 */
bool radar_frame_fifo_init(radar_frame_fifo_t *fifo,
                           radar_frame_fifo_entry_t *entries,
                           size_t capacity);

/**
 * @brief 压入一条完整且已校验的雷达帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  fifo 已成功初始化的 FIFO。
 * @param  data 非 NULL 的帧起始地址；函数同步复制 length 字节，不保留指针。
 * @param  length 帧长，范围 1..RADAR_PARSER_MAX_FRAME_SIZE。
 * @param  sequence 调用方生成的帧序号，原样保存。
 * @param  timestamp_ms 调用方采集时间，单位 ms，原样保存。
 * @return true 表示新帧已复制；参数无效返回 false。满队列会先丢最旧帧但仍返回 true。
 * 调用方式：仅向队列放入已经过 radar_parser 校验的完整原始帧；本函数不会再次校验协议。
 * 线程约束：无内部锁；UART 接收与上行消费者并发时必须由外层 mutex 串行化，禁止 ISR 调用大帧复制。
 */
bool radar_frame_fifo_push(radar_frame_fifo_t *fifo,
                           const uint8_t *data,
                           size_t length,
                           uint32_t sequence,
                           uint32_t timestamp_ms);

/**
 * @brief 取出最旧帧并复制到输出缓冲区。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  fifo 已成功初始化的 FIFO。
 * @param[out] data 非 NULL、至少可写 capacity 字节的输出缓冲。
 * @param  capacity data 容量，必须大于 0。
 * @param[out] length 必须非 NULL；成功时为帧长，空队列时为 0，短缓冲时为所需容量。
 * @param[out] sequence 可为 NULL；成功时返回队首序号。
 * @param[out] timestamp_ms 可为 NULL；成功时返回队首采集时间。
 * @return true 表示已复制并消费队首；参数错误、空队列或短缓冲返回 false，后两者不消费队首。
 * 调用方式：上行任务调用；false 后先检查 length，必要时提供更大缓冲再重试。
 * 线程约束：无内部锁；必须与 push/stats 的并发访问在外层串行化，禁止 ISR 调用大帧复制。
 */
bool radar_frame_fifo_pop(radar_frame_fifo_t *fifo,
                          uint8_t *data,
                          size_t capacity,
                          size_t *length,
                          uint32_t *sequence,
                          uint32_t *timestamp_ms);

/**
 * @brief  复制 FIFO 容量、当前深度、水位和丢最旧计数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（接口契约补充）。
 * @param  fifo 可为 NULL；NULL 时输出全零快照。
 * @param[out] stats 可为 NULL；非 NULL 时总会先清零再复制可用状态。
 * @return 无。
 * 调用方式：健康日志低频读取；统计是瞬时快照，不证明网络消费者已收到帧。
 * 线程约束：无内部锁；与 push/pop 并发时调用方必须加同一外层 mutex。
 */
void radar_frame_fifo_get_stats(const radar_frame_fifo_t *fifo,
                                radar_frame_fifo_stats_t *stats);

#endif
