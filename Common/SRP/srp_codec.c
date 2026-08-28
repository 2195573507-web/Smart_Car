#include "srp_codec.h"

#include <string.h>

#include "srp_crc.h"

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

static int valid_header(const srp_frame_t *frame)
{
    return frame != NULL && frame->priority <= SRP_PRIORITY_LOG &&
           (frame->flags & SRP_FLAG_RESERVED_MASK) == 0U &&
           frame->length <= SRP_MAX_PAYLOAD &&
           (frame->length == 0U || frame->payload != NULL);
}

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

int srp_encode_frame(const srp_frame_t *frame, uint8_t *out, size_t capacity,
                     uint16_t *out_length)
{
    return srp_encode(frame, out, capacity, out_length);
}

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

static void parser_reset(srp_parser_t *parser)
{
    parser->state = SRP_PARSER_WAIT_MAGIC0;
    parser->index = 0U;
    parser->expected_length = 0U;
}

void srp_parser_reset(srp_parser_t *parser)
{
    if (parser != NULL) {
        parser_reset(parser);
    }
}

static void parser_error(srp_parser_t *parser, srp_parser_error_t error)
{
    if (parser->error_callback != NULL) {
        parser->error_callback(error, parser->bytes, parser->index, parser->context);
    }
}

static void parser_record_header_drop(srp_parser_t *parser,
                                      srp_parser_state_t state,
                                      uint8_t byte)
{
    parser->last_error_state = state;
    parser->last_drop_byte = byte;
}

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
