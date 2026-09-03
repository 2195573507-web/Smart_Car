#include "radar_parser.h"

/* YDLIDAR 原始流解析实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <stdio.h>
#include <string.h>

/**
 * @brief  将相对队尾的逻辑偏移换算为环形缓冲物理下标。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  parser 非 NULL、已初始化的解析器；函数直接读取 tail。
 * @param  offset 相对 tail 的字节偏移，调用方保证属于当前候选范围。
 * @return 对 RADAR_PARSER_RING_BUFFER_SIZE 取模后的物理数组下标。
 * 调用方式：仅由解析器内部 peek/copy/扫描路径调用，不独立校验参数。
 * 线程约束：纯计算、可重入、不阻塞；同一 parser 的并发修改仍须由外层禁止。
 */
static size_t ring_index(const radar_parser_t *parser, size_t offset)
{
    return (parser->tail + offset) % RADAR_PARSER_RING_BUFFER_SIZE;
}

/**
 * @brief  从连续两字节读取一个小端 16 位整数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  data 必须指向至少 2 个可读字节，函数不做空指针或边界检查。
 * @return 解码后的 uint16_t 值。
 * 调用方式：默认 X3/X3PRO 校验器在完成候选帧长度检查后读取角度、校验和及采样字段。
 * 线程约束：纯只读计算、可重入、不阻塞。
 */
static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/**
 * @brief  按 X3/X3PRO 规则校验候选帧布局和 XOR 校验和。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  frame 候选完整帧；非 NULL 且至少包含 RADAR_X3PRO_HEADER_BYTES 字节。
 * @param  length 候选帧总长度，用于反推每个样本为 2 或 3 字节。
 * @param  context 兼容校验回调签名的上下文，当前默认实现忽略，可为 NULL。
 * @return 帧头、LSN、采样布局和 XOR 校验均合法时为 true，否则为 false。
 * 调用方式：由独立完整帧验证和流解析候选验证同步调用；不检查 FSA/LSA 的协议检查位。
 * 线程约束：只读、可重入、不阻塞；计算量随样本数增长，不应在 ISR 中执行。
 */
static bool radar_parser_default_checksum(const uint8_t *frame,
                                          size_t length,
                                          void *context)
{
    (void)context;

    if (frame == NULL || length < RADAR_X3PRO_HEADER_BYTES ||
        frame[0] != RADAR_X3PRO_HEADER_BYTE_0 ||
        frame[1] != RADAR_X3PRO_HEADER_BYTE_1) {
        return false;
    }

    const size_t sample_count = frame[3];
    if (sample_count == 0U || sample_count > RADAR_X3PRO_MAX_SAMPLES) {
        return false;
    }

    const size_t sample_payload_length = length - RADAR_X3PRO_HEADER_BYTES;
    if (sample_payload_length % sample_count != 0U) {
        return false;
    }

    const size_t sample_bytes = sample_payload_length / sample_count;
    if (sample_bytes != RADAR_X3PRO_SAMPLE_BYTES &&
        sample_bytes != RADAR_X3PRO_MAX_SAMPLE_BYTES) {
        return false;
    }

    uint16_t checksum = 0x55AAU;
    checksum ^= read_le16(&frame[4]);
    for (size_t index = 0U; index < sample_count; ++index) {
        const size_t offset = RADAR_X3PRO_HEADER_BYTES +
                              (index * sample_bytes);
        if (sample_bytes == RADAR_X3PRO_SAMPLE_BYTES) {
            checksum ^= read_le16(&frame[offset]);
        } else {
            checksum ^= frame[offset];
            checksum ^= read_le16(&frame[offset + 1U]);
        }
    }
    checksum ^= (uint16_t)(((uint16_t)frame[3] << 8U) | frame[2]);
    checksum ^= read_le16(&frame[6]);

    return checksum == read_le16(&frame[8]);
}

/**
 * @brief  使用默认 X3/X3PRO 规则独立验证一条完整原始帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  frame 非 NULL 的完整帧缓冲，函数不保留指针。
 * @param  length 必须精确匹配 LSN 推导的 2/3 字节采样帧长度。
 * @param[out] sample_bytes 可为 NULL；成功时写入检测到的每样本 2 或 3 字节。
 * @return 帧头、LSN、FSA/LSA 检查位、长度和默认 XOR 均通过时为 true，否则为 false。
 * 调用方式：S3RD 封装、兼容解码和主机测试复验完整帧；不会使用 parser 实例的自定义校验器。
 * 线程约束：纯只读计算、可重入、不阻塞；失败时 sample_bytes 保持调用前值，大帧校验不适合 ISR。
 */
bool radar_parser_validate_frame(const uint8_t *frame,
                                 size_t length,
                                 size_t *sample_bytes)
{
    if (frame == NULL || length < RADAR_X3PRO_HEADER_BYTES ||
        frame[0] != RADAR_X3PRO_HEADER_BYTE_0 ||
        frame[1] != RADAR_X3PRO_HEADER_BYTE_1) {
        return false;
    }

    const size_t sample_count = frame[RADAR_X3PRO_LENGTH_OFFSET];
    if (sample_count == 0U || sample_count > RADAR_X3PRO_MAX_SAMPLES ||
        (frame[4] & 0x01U) == 0U || (frame[6] & 0x01U) == 0U) {
        return false;
    }

    const size_t payload_length = length - RADAR_X3PRO_HEADER_BYTES;
    if (payload_length % sample_count != 0U) {
        return false;
    }

    const size_t detected_sample_bytes = payload_length / sample_count;
    if (detected_sample_bytes != RADAR_X3PRO_SAMPLE_BYTES &&
        detected_sample_bytes != RADAR_X3PRO_MAX_SAMPLE_BYTES) {
        return false;
    }
    if (!radar_parser_default_checksum(frame, length, NULL)) {
        return false;
    }

    if (sample_bytes != NULL) {
        *sample_bytes = detected_sample_bytes;
    }
    return true;
}

/**
 * @brief  读取环形缓冲中相对队尾偏移处的一个字节而不消费。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  parser 非 NULL、已初始化的解析器。
 * @param  offset 必须小于当前 parser->size，函数不做范围检查。
 * @return 指定逻辑位置的字节值。
 * 调用方式：仅由同一解析循环的帧头搜索、字段检查和候选复制调用。
 * 线程约束：纯读取、不阻塞；调用期间同一 parser 不得被其他上下文修改。
 */
static uint8_t ring_peek(const radar_parser_t *parser, size_t offset)
{
    return parser->buffer[ring_index(parser, offset)];
}

/**
 * @brief  从环形缓冲队尾消费指定数量的字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] parser 非 NULL、已初始化的解析器。
 * @param  count 要丢弃的字节数；大于等于当前 size 时直接清空未处理流。
 * @return 无。
 * 调用方式：解析循环在成功消费、噪声重同步或候选失败后调用；不会修改累计统计。
 * 线程约束：修改 head/tail/size、无内部锁；只允许 parser 的单一 feed owner 调用。
 */
static void ring_drop(radar_parser_t *parser, size_t count)
{
    if (count >= parser->size) {
        parser->tail = parser->head;
        parser->size = 0U;
        return;
    }

    parser->tail = (parser->tail + count) % RADAR_PARSER_RING_BUFFER_SIZE;
    parser->size -= count;
}

/**
 * @brief  向解析器环形缓冲追加一个字节，满时覆盖式丢弃最旧字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] parser 非 NULL、已初始化的解析器。
 * @param  byte 待追加的原始 UART 字节。
 * @return 无；缓冲已满时先丢最旧字节并递增 overflow_count。
 * 调用方式：radar_parser_feed() 按输入顺序逐字节调用；不单独触发帧解析回调。
 * 线程约束：修改 parser、无内部锁；只允许单一任务串行调用，禁止 ISR/并发 feed。
 */
static void ring_push(radar_parser_t *parser, uint8_t byte)
{
    if (parser->size == RADAR_PARSER_RING_BUFFER_SIZE) {
        parser->tail = (parser->tail + 1U) % RADAR_PARSER_RING_BUFFER_SIZE;
        --parser->size;
        ++parser->stats.overflow_count;
    }

    parser->buffer[parser->head] = byte;
    parser->head = (parser->head + 1U) % RADAR_PARSER_RING_BUFFER_SIZE;
    ++parser->size;
}

/**
 * @brief  在当前环形缓冲可用区中查找最早的 AA 55 帧头。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  parser 解析器实例；NULL 或少于 2 字节时直接失败。
 * @param[out] offset 非 NULL；成功时写相对 tail 的帧头偏移，失败时保持调用前值。
 * @return 找到完整两字节帧头时为 true，否则为 false。
 * 调用方式：process_frames() 每轮解析候选前调用，用结果决定丢噪声或等待后续字节。
 * 线程约束：只读线性扫描、不阻塞；扫描期间同一 parser 不得被并发修改。
 */
static bool find_header(const radar_parser_t *parser, size_t *offset)
{
    if (parser == NULL || offset == NULL || parser->size < 2U) {
        return false;
    }

    for (size_t index = 0U; index + 1U < parser->size; ++index) {
        if (ring_peek(parser, index) == RADAR_X3PRO_HEADER_BYTE_0 &&
            ring_peek(parser, index + 1U) == RADAR_X3PRO_HEADER_BYTE_1) {
            *offset = index;
            return true;
        }
    }
    return false;
}

/**
 * @brief  将环形缓冲队尾开始的候选帧复制为连续字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  parser 非 NULL，且当前 size 至少为 frame_length。
 * @param[out] frame 非 NULL、至少可写 frame_length 字节的连续输出缓冲。
 * @param  frame_length 要复制的候选帧字节数。
 * @return 无。
 * 调用方式：candidate_checksum_valid() 在校验回调前把环形候选复制到栈缓冲；不消费源数据。
 * 线程约束：无内部锁并执行逐字节复制；只允许解析任务调用，不适合 ISR。
 */
static void copy_frame(const radar_parser_t *parser,
                       uint8_t *frame,
                       size_t frame_length)
{
    for (size_t index = 0U; index < frame_length; ++index) {
        frame[index] = ring_peek(parser, index);
    }
}

/**
 * @brief  复制一个候选帧并调用当前配置的完整性校验器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  parser 非 NULL，且缓冲中至少有 frame_length 字节。
 * @param[out] frame 非 NULL、容量至少为 frame_length；函数先写入候选帧。
 * @param  frame_length 候选的完整字节数，不超过 RADAR_PARSER_MAX_FRAME_SIZE。
 * @return 当前校验器接受候选时为 true，否则为 false；校验器为空时使用独立默认验证。
 * 调用方式：process_frames() 依次尝试 2 字节和 3 字节采样布局；自定义回调在本调用栈同步执行。
 * 线程约束：与 feed 相同、不可重入；校验器不得保存 frame 指针、递归 feed 或无限阻塞。
 */
static bool candidate_checksum_valid(const radar_parser_t *parser,
                                     uint8_t *frame,
                                     size_t frame_length)
{
    copy_frame(parser, frame, frame_length);
    return parser->checksum_validator == NULL
               ? radar_parser_validate_frame(frame, frame_length, NULL)
               : parser->checksum_validator(frame,
                                            frame_length,
                                            parser->checksum_context);
}

/**
 * @brief  从环形流中连续提取、校验并同步分发所有当前可确定的完整帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] parser 非 NULL、已初始化的解析器；函数会消费字节并更新累计统计。
 * @param  callback 可为 NULL；非 NULL 时每条有效帧同步调用一次。
 * @param  context 原样传给 callback，解析器不拥有该对象。
 * @return 无；遇到半帧时保留数据等待下次 feed，噪声/无效候选按实现有界丢弃并重同步。
 * 调用方式：radar_parser_feed() 完成输入追加后调用；回调收到的 frame 位于本函数栈上，返回即失效。
 * 线程约束：使用 RADAR_PARSER_MAX_FRAME_SIZE 字节栈缓冲并可能处理多帧，禁止 ISR、并发或递归调用。
 */
static void process_frames(radar_parser_t *parser,
                           radar_frame_callback_t callback,
                           void *context)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];

    while (parser->size >= 2U) {
        size_t header_offset = 0U;
        if (!find_header(parser, &header_offset)) {
            /* Retain one trailing AA only: the next feed may complete an
             * AA 55 header, while all other noise can be discarded now. */
            const bool retain_header_prefix =
                ring_peek(parser, parser->size - 1U) == RADAR_X3PRO_HEADER_BYTE_0;
            const size_t drop_count = parser->size -
                                      (retain_header_prefix ? 1U : 0U);
            if (drop_count > 0U) {
                ++parser->stats.header_resync_count;
                ring_drop(parser, drop_count);
            }
            return;
        }

        if (header_offset > 0U) {
            ++parser->stats.header_resync_count;
            ring_drop(parser, header_offset);
        }

        /* CT and LSN are the first two fields after the two-byte header. */
        if (parser->size <= RADAR_X3PRO_LENGTH_OFFSET) {
            return;
        }

        size_t sample_count = ring_peek(parser, RADAR_X3PRO_LENGTH_OFFSET);
        if (sample_count == 0U || sample_count > RADAR_X3PRO_MAX_SAMPLES) {
            ++parser->stats.invalid_frame_count;
            ring_drop(parser, 1U);
            continue;
        }

        if (parser->size < RADAR_X3PRO_HEADER_BYTES) {
            return;
        }

        /* FSA and LSA carry the protocol check bit in bit 0. */
        if ((ring_peek(parser, 4U) & 0x01U) == 0U ||
            (ring_peek(parser, 6U) & 0x01U) == 0U) {
            ++parser->stats.invalid_frame_count;
            ++parser->stats.header_resync_count;
            ring_drop(parser, 1U);
            continue;
        }

        const size_t distance_frame_length = RADAR_X3PRO_HEADER_BYTES +
                                             (sample_count * RADAR_X3PRO_SAMPLE_BYTES);
        const size_t intensity_frame_length = RADAR_X3PRO_HEADER_BYTES +
                                              (sample_count * RADAR_X3PRO_MAX_SAMPLE_BYTES);
        size_t frame_length = 0U;
        size_t accepted_sample_bytes = 0U;

        if (parser->sample_bytes == RADAR_X3PRO_SAMPLE_BYTES_AUTO) {
            if (parser->size < distance_frame_length) {
                return;
            }
            if (candidate_checksum_valid(parser, frame, distance_frame_length)) {
                frame_length = distance_frame_length;
                accepted_sample_bytes = RADAR_X3PRO_SAMPLE_BYTES;
            } else {
                /* A failed two-byte candidate may be the prefix of an
                 * intensity frame. Do not resynchronise until the full
                 * three-byte candidate is available and has also failed. */
                if (parser->size < intensity_frame_length) {
                    return;
                }
                if (candidate_checksum_valid(parser, frame, intensity_frame_length)) {
                    frame_length = intensity_frame_length;
                    accepted_sample_bytes = RADAR_X3PRO_MAX_SAMPLE_BYTES;
                }
            }
        } else {
            accepted_sample_bytes = parser->sample_bytes;
            frame_length = RADAR_X3PRO_HEADER_BYTES +
                           (sample_count * accepted_sample_bytes);
            if (frame_length > RADAR_PARSER_MAX_FRAME_SIZE) {
                ++parser->stats.invalid_frame_count;
                ring_drop(parser, 1U);
                continue;
            }
            if (parser->size < frame_length) {
                return;
            }
            if (!candidate_checksum_valid(parser, frame, frame_length)) {
                frame_length = 0U;
                accepted_sample_bytes = 0U;
            }
        }

        if (accepted_sample_bytes == 0U) {
            ++parser->stats.checksum_error_count;
            ++parser->stats.header_resync_count;
            /* Keep searching from the next byte after this candidate header. */
            ring_drop(parser, 1U);
            continue;
        }

        ++parser->stats.valid_frame_count;
        parser->stats.last_sample_bytes = (uint8_t)accepted_sample_bytes;
        if (accepted_sample_bytes == RADAR_X3PRO_SAMPLE_BYTES) {
            ++parser->stats.valid_distance_frame_count;
        } else {
            ++parser->stats.valid_intensity_frame_count;
        }
        if (callback != NULL) {
            callback(frame, frame_length, context);
        }
        ring_drop(parser, frame_length);
    }
}

/**
 * @brief  清零解析器流/统计并选择自动采样布局和默认 XOR 校验器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[out] parser 可为 NULL；非 NULL 时整体重置，既有统计和自定义校验器会丢失。
 * @return 无。
 * 调用方式：雷达 UART 接收任务启动前调用；parser 必须在整个接收期保持有效。
 * 线程约束：整体 memset、无内部锁；初始化期间禁止 feed/get_stats 等并发访问，禁止 ISR 调用。
 */
void radar_parser_init(radar_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->sample_bytes = RADAR_X3PRO_SAMPLE_BYTES_AUTO;
    parser->checksum_validator = radar_parser_default_checksum;
}

/**
 * @brief  丢弃当前未完成字节流并重新等待帧头，保留配置和累计统计。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] parser 可为 NULL；非 NULL 时只清 head、tail 和 size。
 * @return 无。
 * 调用方式：UART FIFO 溢出或驱动缓冲丢失后调用，防止在数据缺口两侧拼接伪帧。
 * 线程约束：无内部锁；必须与 feed 串行并由雷达 UART 任务调用，禁止 ISR 直接访问同一实例。
 */
void radar_parser_reset_stream(radar_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }

    parser->head = 0U;
    parser->tail = 0U;
    parser->size = 0U;
}

/**
 * @brief  安装设备变体校验器，或以 NULL 恢复默认 X3/X3PRO XOR 校验器。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] parser 解析器实例；NULL 时不动作。
 * @param  validator 自定义同步回调；NULL 表示恢复默认实现。
 * @param  context 原样借给自定义回调，解析器不接管所有权；默认实现忽略它。
 * @return 无。
 * 调用方式：初始化后、喂入字节前设置；运行期变更前应先 reset_stream()，本函数不会清半帧。
 * 线程约束：无内部锁；不得与 feed 并发，context 生命周期必须覆盖后续所有校验调用。
 */
void radar_parser_set_checksum_validator(
    radar_parser_t *parser,
    radar_parser_checksum_validator_t validator,
    void *context)
{
    if (parser == NULL) {
        return;
    }

    parser->checksum_validator = validator == NULL ? radar_parser_default_checksum : validator;
    parser->checksum_context = context;
}

/**
 * @brief  选择自动、2 字节距离或 3 字节强度采样布局。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] parser 非 NULL、已初始化的解析器。
 * @param  sample_bytes 仅允许 AUTO(0)、RADAR_X3PRO_SAMPLE_BYTES(2) 或 MAX(3)。
 * @return 配置更新成功为 true；实例为空或值不支持时为 false，原配置保持不变。
 * 调用方式：真实设备格式未冻结时保持 AUTO；运行期切换前先 reset_stream()，避免半帧跨布局。
 * 线程约束：无内部锁；不得与 feed 并发，禁止 ISR 调用同一实例。
 */
bool radar_parser_set_sample_bytes(radar_parser_t *parser, size_t sample_bytes)
{
    if (parser == NULL ||
        (sample_bytes != RADAR_X3PRO_SAMPLE_BYTES_AUTO &&
         sample_bytes != RADAR_X3PRO_SAMPLE_BYTES &&
         sample_bytes != RADAR_X3PRO_MAX_SAMPLE_BYTES)) {
        return false;
    }

    parser->sample_bytes = sample_bytes;
    return true;
}

/**
 * @brief  复制有效帧、校验错误、重同步和环形溢出统计快照。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  parser 可为 NULL；NULL 时输出全零。
 * @param[out] stats 可为 NULL；非 NULL 时先清零再复制统计。
 * @return 无。
 * 调用方式：雷达 UART 任务低频输出健康日志；统计不证明测量字段已解码或主机已收到。
 * 线程约束：无内部锁；当前由 parser owner 串行读取，跨任务读取时调用方需保护一致快照。
 */
void radar_parser_get_stats(const radar_parser_t *parser,
                            radar_parser_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    memset(stats, 0, sizeof(*stats));
    if (parser != NULL) {
        *stats = parser->stats;
    }
}

/**
 * @brief  将连续 UART 字节追加到环形缓冲并同步提取所有可用完整帧。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param[in,out] parser 非 NULL、已初始化的解析器。
 * @param  data 输入字节；length 大于 0 时必须非 NULL，返回前完成读取。
 * @param  length 输入字节数，可为 0。
 * @param  callback 可为 NULL；非 NULL 时每条有效帧同步调用一次。
 * @param  context 原样传给 callback，解析器不拥有该对象。
 * @return 无；参数无效时静默返回，环形满时丢最旧字节并增加 overflow_count。
 * 调用方式：单一雷达 UART 任务按接收块调用；回调必须在返回前复制临时帧。
 * 线程约束：会修改 parser 并使用较大栈缓冲，不可重入、不可并发，禁止 ISR 调用。
 */
void radar_parser_feed(radar_parser_t *parser,
                       const uint8_t *data,
                       size_t length,
                       radar_frame_callback_t callback,
                       void *context)
{
    if (parser == NULL || (data == NULL && length > 0U)) {
        return;
    }

    for (size_t index = 0U; index < length; ++index) {
        ring_push(parser, data[index]);
    }
    process_frames(parser, callback, context);
}

/**
 * @brief  保留测量点解码接口；字段未冻结前不输出任何推测的角度、距离或质量。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  frame 候选完整帧，可为 NULL。
 * @param  length frame 的字节长度。
 * @param[out] measurement 输出对象；非 NULL 时首先整体清零。
 * @return 当前实现固定返回 false，包括基本帧头有效的情况；输出不得用于构造 LaserScan。
 * 调用方式：仅保留 ABI/调用点兼容，待真实抓包与主机协议联合冻结后再实现字段转换。
 * 线程约束：纯内存操作、可重入、不阻塞；该预留入口不应从 ISR 调用。
 */
bool radar_parser_parse_measurement(const uint8_t *frame,
                                    size_t length,
                                    radar_measurement_t *measurement)
{
    if (measurement != NULL) {
        memset(measurement, 0, sizeof(*measurement));
    }

    if (frame == NULL || measurement == NULL ||
        length < RADAR_X3PRO_HEADER_BYTES ||
        frame[0] != RADAR_X3PRO_HEADER_BYTE_0 ||
        frame[1] != RADAR_X3PRO_HEADER_BYTE_1) {
        return false;
    }

    /* Field conversion is intentionally deferred until the device variant is
     * confirmed from a captured X3PRO frame.  Do not emit made-up values. */
    return false;
}

/**
 * @brief  将字节序列格式化为以空格分隔的大写十六进制诊断字符串。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date   2026-08-31（函数契约补充）。
 * @param  data 输入字节；NULL 时输出空字符串。
 * @param  length 希望格式化的字节数。
 * @param[out] output 输出字符缓冲；NULL 时不动作。
 * @param  output_size output 容量；大于 0 时保证 NUL 结尾，空间不足时安全截断。
 * @return 无；snprintf 失败时输出空字符串，接口不报告实际截断位置。
 * 调用方式：仅用于有节流的诊断路径，生产高速流不应频繁格式化整帧。
 * 线程约束：使用调用方缓冲、可重入；snprintf 成本不适合 ISR 或硬实时回调。
 */
void radar_parser_format_hex(const uint8_t *data,
                             size_t length,
                             char *output,
                             size_t output_size)
{
    if (output == NULL || output_size == 0U) {
        return;
    }

    output[0] = '\0';
    if (data == NULL) {
        return;
    }

    size_t written = 0U;
    for (size_t index = 0U; index < length; ++index) {
        if (written >= output_size) {
            break;
        }

        int count = snprintf(output + written,
                             output_size - written,
                             index == 0U ? "%02X" : " %02X",
                             data[index]);
        if (count < 0) {
            output[0] = '\0';
            return;
        }
        if ((size_t)count >= output_size - written) {
            output[output_size - 1U] = '\0';
            return;
        }
        written += (size_t)count;
    }
}
