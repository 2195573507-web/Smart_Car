#include "srp_codec.h"

#include <string.h>

#include "srp_crc.h"

/* SRP v4 编解码与增量解析实现；创建人：待确认（当前维护人：Zhiqin）。 */

/**
 * @brief 从连续 2 字节读取一个小端无符号整数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param data 至少包含 2 字节的只读缓冲；调用方必须保证非 NULL 和容量有效。
 * @return 解码后的 uint16_t 数值。
 * 调用方式：仅在完整 SRP header/trailer 长度已校验后读取固定字段。
 * 线程约束：纯内存读取、可重入、不阻塞，不保留输入指针。
 */
static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

/**
 * @brief 从连续 4 字节读取一个小端无符号整数。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param data 至少包含 4 字节的只读缓冲；调用方必须保证非 NULL 和容量有效。
 * @return 解码后的 uint32_t 数值。
 * 调用方式：仅在完整 SRP header 长度已校验后读取逻辑 header。
 * 线程约束：纯内存读取、可重入、不阻塞，不保留输入指针。
 */
static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

/**
 * @brief 把 uint16_t 按小端序写入连续 2 字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param data 至少可写 2 字节的缓冲；调用方必须保证非 NULL 和容量有效。
 * @param value 待编码数值。
 * @return 无。
 * 调用方式：srp_encode() 写 magic、length、CRC 和 EOF 固定字段时调用。
 * 线程约束：纯内存写入、可重入、不阻塞；同一输出区域不得并发写。
 */
static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

/**
 * @brief 把 uint32_t 按小端序写入连续 4 字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param data 至少可写 4 字节的缓冲；调用方必须保证非 NULL 和容量有效。
 * @param value 待编码数值。
 * @return 无。
 * 调用方式：srp_encode() 写组合后的逻辑 header 时调用。
 * 线程约束：纯内存写入、可重入、不阻塞；同一输出区域不得并发写。
 */
static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

/**
 * @brief 校验逻辑帧的优先级、保留标志、payload 长度和指针组合。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param frame 待编码的只读逻辑帧；允许 NULL。
 * @return 全部约束满足时返回 1，否则返回 0；不校验业务消息 ID 或 payload 内容。
 * 调用方式：srp_encode() 在计算输出长度和访问 payload 前调用。
 * 线程约束：纯只读判断、可重入、不阻塞；frame 在调用期间保持有效。
 */
static int valid_header(const srp_frame_t *frame)
{
    return frame != NULL && frame->priority <= SRP_PRIORITY_LOG &&
           (frame->flags & SRP_FLAG_RESERVED_MASK) == 0U &&
           frame->length <= SRP_MAX_PAYLOAD &&
           (frame->length == 0U || frame->payload != NULL);
}

/** 依据 srp_codec.h 契约编码一条完整线缆帧。 */
int srp_encode(const srp_frame_t *frame, uint8_t *out, size_t capacity,
               uint16_t *out_length)
{
    size_t total;
    uint32_t header;
    uint16_t crc;

    if (!valid_header(frame) || out == NULL || out_length == NULL) {
        return SRP_CODEC_INVALID_ARGUMENT;
    }
    total = (size_t)SRP_HEADER_SIZE + frame->length + SRP_TRAILER_SIZE;
    if (capacity < total) {
        return SRP_CODEC_OVERFLOW;
    }
    write_u16_le(&out[0], SRP_MAGIC);
    write_u16_le(&out[2], frame->length);
    header = SRP_HDR_MAKE(frame->priority, frame->type, frame->sequence,
                          frame->flags);
    write_u32_le(&out[4], header);
    if (frame->length != 0U) {
        (void)memcpy(&out[8], frame->payload, frame->length);
    }
    crc = srp_crc16_ccitt_false(&out[2], 6U + frame->length);
    write_u16_le(&out[8U + frame->length], crc);
    write_u16_le(&out[10U + frame->length], SRP_EOF);
    *out_length = (uint16_t)total;
    return SRP_CODEC_OK;
}

/** 统一入口包装，保持所有传输层使用同一编码实现。 */
int srp_encode_frame(const srp_frame_t *frame, uint8_t *out, size_t capacity,
                     uint16_t *out_length)
{
    return srp_encode(frame, out, capacity, out_length);
}

/** 校验魔数、长度、header、CRC、EOF 并返回借用 payload 视图。 */
int srp_decode(const uint8_t *data, size_t length, srp_frame_t *frame)
{
    uint16_t payload_length;
    uint16_t expected_crc;
    uint16_t actual_crc;
    uint32_t header;
    size_t expected_length;

    if (data == NULL || frame == NULL || length < SRP_HEADER_SIZE + SRP_TRAILER_SIZE) {
        return SRP_CODEC_INVALID_ARGUMENT;
    }
    if (data[0] != SRP_MAGIC_BYTE0 || data[1] != SRP_MAGIC_BYTE1) {
        return SRP_CODEC_BAD_MAGIC;
    }
    payload_length = read_u16_le(&data[2]);
    if (payload_length > SRP_MAX_PAYLOAD) {
        return SRP_CODEC_INVALID_LENGTH;
    }
    expected_length = (size_t)SRP_HEADER_SIZE + payload_length + SRP_TRAILER_SIZE;
    if (length != expected_length) {
        return SRP_CODEC_INVALID_LENGTH;
    }
    header = read_u32_le(&data[4]);
    frame->priority = SRP_HDR_PRI(header);
    frame->type = SRP_HDR_TYPE(header);
    frame->sequence = SRP_HDR_SEQ(header);
    frame->flags = SRP_HDR_FLAGS(header);
    frame->length = payload_length;
    frame->payload = &data[8];
    if (frame->priority > SRP_PRIORITY_LOG ||
        (frame->flags & SRP_FLAG_RESERVED_MASK) != 0U) {
        return SRP_CODEC_INVALID_HEADER;
    }
    expected_crc = read_u16_le(&data[8U + payload_length]);
    actual_crc = srp_crc16_ccitt_false(&data[2], 6U + payload_length);
    if (expected_crc != actual_crc) {
        return SRP_CODEC_BAD_CRC;
    }
    if (data[10U + payload_length] != SRP_EOF_BYTE0 ||
        data[11U + payload_length] != SRP_EOF_BYTE1) {
        return SRP_CODEC_BAD_EOF;
    }
    return SRP_CODEC_OK;
}

/**
 * @brief 把增量 parser 恢复到等待首个 magic 字节的半帧状态。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param parser 非 NULL 的可写解析器；不清回调、context、累计计数或字节数组。
 * @return 无。
 * 调用方式：初始化、公开 reset 以及完成/拒绝一帧后调用。
 * 线程约束：无内部锁，只允许同一 parser 的单一接收 owner 调用。
 */
static void parser_reset(srp_parser_t *parser)
{
    parser->state = SRP_PARSER_WAIT_MAGIC0;
    parser->index = 0U;
    parser->expected_length = 0U;
}

/** 丢弃半帧并保留回调/累计诊断。 */
void srp_parser_reset(srp_parser_t *parser)
{
    if (parser != NULL) {
        parser_reset(parser);
    }
}

/**
 * @brief 在当前 parser 状态下同步上报一次解析错误。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param parser 非 NULL 解析器；bytes/index 作为借用错误片段传给回调。
 * @param error 错误类型。
 * @return 无；未注册 error_callback 时不动作，也不会自动 reset parser。
 * 调用方式：parser_consume_byte() 记录必要诊断字段后调用，再由对应分支决定 reset。
 * 线程约束：回调在 feed 调用栈同步执行；不得保留 bytes、递归 feed 同一 parser 或阻塞。
 */
static void parser_error(srp_parser_t *parser, srp_parser_error_t error)
{
    if (parser->error_callback != NULL) {
        parser->error_callback(error, parser->bytes, parser->index, parser->context);
    }
}

/**
 * @brief 保存最近一次 magic/header 拒绝时的 parser 状态和触发字节。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param parser 非 NULL 的可写解析器。
 * @param state 发生拒绝时的状态。
 * @param byte 导致拒绝或用于定位 header 的字节。
 * @return 无；只更新诊断快照，不增加错误计数或改变解析状态。
 * 调用方式：magic、长度和完整 header 校验失败路径在 error callback 前调用。
 * 线程约束：无锁状态写入，只允许 parser 单 owner 调用。
 */
static void parser_record_header_drop(srp_parser_t *parser,
                                      srp_parser_state_t state,
                                      uint8_t byte)
{
    parser->last_error_state = state;
    parser->last_drop_byte = byte;
}

/** 初始化增量解析器及其回调上下文。 */
void srp_parser_init(srp_parser_t *parser,
                     void (*frame_callback)(const srp_frame_t *, void *),
                     void (*error_callback)(srp_parser_error_t, const uint8_t *,
                                            size_t, void *),
                     void *context)
{
    if (parser == NULL) {
        return;
    }
    (void)memset(parser, 0, sizeof(*parser));
    parser->frame_callback = frame_callback;
    parser->error_callback = error_callback;
    parser->context = context;
    parser_reset(parser);
}

/**
 * @brief 向 SRP 增量状态机消费一个字节，并在完整帧或错误时同步触发回调。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param parser 已初始化且非 NULL 的可写解析器。
 * @param byte 本次输入字节。
 * @return 无；函数更新状态、索引和统计，完成或拒绝一帧后回到 magic 搜索。
 * 调用方式：仅由 srp_parser_feed() 按输入顺序逐字调用。
 * 线程约束：同一 parser 不可重入或并发；frame/error 回调在当前调用栈同步执行，
 *           frame->payload 借用 parser 内部 bytes，仅在回调期间有效。
 */
static void parser_consume_byte(srp_parser_t *parser, uint8_t byte)
{
    srp_frame_t frame;
    uint16_t payload_length;
    int result;

    if (parser->state == SRP_PARSER_WAIT_MAGIC0) {
        if (byte == SRP_MAGIC_BYTE0) {
            parser->bytes[0] = byte;
            parser->index = 1U;
            parser->state = SRP_PARSER_WAIT_MAGIC1;
        } else {
            parser_record_header_drop(parser, SRP_PARSER_WAIT_MAGIC0, byte);
            parser_error(parser, SRP_PARSER_ERROR_MAGIC);
        }
        return;
    }
    if (parser->state == SRP_PARSER_WAIT_MAGIC1) {
        if (byte == SRP_MAGIC_BYTE1) {
            parser->bytes[parser->index++] = byte;
            parser->state = SRP_PARSER_READ_HEADER;
        } else if (byte == SRP_MAGIC_BYTE0) {
            parser->bytes[0] = byte;
            parser->index = 1U;
        } else {
            parser_record_header_drop(parser, SRP_PARSER_WAIT_MAGIC1, byte);
            parser_error(parser, SRP_PARSER_ERROR_MAGIC);
            parser_reset(parser);
        }
        return;
    }
    if (parser->index >= SRP_MAX_FRAME_SIZE) {
        ++parser->length_error_count;
        parser_error(parser, SRP_PARSER_ERROR_OVERFLOW);
        parser_reset(parser);
        return;
    }
    parser->bytes[parser->index++] = byte;
    if (parser->state == SRP_PARSER_READ_HEADER && parser->index == SRP_HEADER_SIZE) {
        payload_length = read_u16_le(&parser->bytes[2]);
        if (payload_length > SRP_MAX_PAYLOAD) {
            ++parser->length_error_count;
            parser_record_header_drop(parser, SRP_PARSER_READ_HEADER, byte);
            parser_error(parser, SRP_PARSER_ERROR_LENGTH);
            parser_reset(parser);
            return;
        }
        parser->expected_length = (uint16_t)(SRP_HEADER_SIZE + payload_length +
                                             SRP_TRAILER_SIZE);
        parser->state = SRP_PARSER_READ_BODY;
    }
    if (parser->state == SRP_PARSER_READ_BODY &&
        parser->index == parser->expected_length) {
        result = srp_decode(parser->bytes, parser->index, &frame);
        if (result == SRP_CODEC_OK) {
            ++parser->frame_count;
            if (parser->frame_callback != NULL) {
                parser->frame_callback(&frame, parser->context);
            }
        } else {
            if (result == SRP_CODEC_BAD_CRC) {
                ++parser->crc_error_count;
                parser_error(parser, SRP_PARSER_ERROR_CRC);
            } else if (result == SRP_CODEC_BAD_EOF) {
                ++parser->eof_error_count;
                parser_error(parser, SRP_PARSER_ERROR_EOF);
            } else if (result == SRP_CODEC_INVALID_LENGTH) {
                ++parser->length_error_count;
                parser_error(parser, SRP_PARSER_ERROR_LENGTH);
            } else {
                /* Preserve the offending header byte for diagnostics. The
                 * frame is complete, but validation rejected it before CRC
                 * evaluation (for example, reserved flags or priority). */
                parser_record_header_drop(parser, SRP_PARSER_READ_HEADER,
                                          parser->bytes[7U]);
                parser_error(parser, SRP_PARSER_ERROR_HEADER);
            }
        }
        parser_reset(parser);
    }
}

/** 消费输入字节；完整帧/错误在任务上下文回调。 */
size_t srp_parser_feed(srp_parser_t *parser, const uint8_t *data, size_t length)
{
    if (parser == NULL || data == NULL) {
        return 0U;
    }
    for (size_t index = 0U; index < length; ++index) {
        parser_consume_byte(parser, data[index]);
    }
    return length;
}

/** 初始化 TLV 迭代器，不复制输入。 */
void srp_tlv_iter_init(srp_tlv_iter_t *iterator, const uint8_t *data,
                       size_t length)
{
    if (iterator == NULL) {
        return;
    }
    iterator->data = data;
    iterator->length = length;
    iterator->offset = 0U;
}

/** 读取一项 TLV 并推进 offset。 */
bool srp_tlv_next(srp_tlv_iter_t *iterator, uint8_t *tag, uint8_t *value_length,
                  const uint8_t **value)
{
    size_t end;

    if (iterator == NULL || tag == NULL || value_length == NULL || value == NULL ||
        iterator->data == NULL || iterator->offset + 2U > iterator->length) {
        return false;
    }
    *tag = iterator->data[iterator->offset];
    *value_length = iterator->data[iterator->offset + 1U];
    end = iterator->offset + 2U + *value_length;
    if (end > iterator->length) {
        iterator->offset = iterator->length;
        return false;
    }
    *value = &iterator->data[iterator->offset + 2U];
    iterator->offset = end;
    return true;
}
