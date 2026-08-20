#include "scbp_parser.h"

#include <string.h>

#include "scbp_crc.h"

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & UINT16_C(0x00FF));
    data[1] = (uint8_t)(value >> 8U);
}

static uint8_t valid_node(uint8_t node)
{
    return node >= SCBP_NODE_STM32H757 && node <= SCBP_NODE_BROADCAST;
}

static void parser_seek_sof0(scbp_parser_t *parser, uint8_t byte)
{
    parser->header_index = 0U;
    parser->body_index = 0U;
    parser->payload_length = 0U;
    if (byte == SCBP_CAN_SOF0) {
        parser->bytes[0] = byte;
        parser->state = SCBP_PARSER_WAIT_SOF1;
    } else {
        parser->state = SCBP_PARSER_WAIT_SOF0;
    }
}

static void parser_reset(scbp_parser_t *parser)
{
    parser_seek_sof0(parser, 0U);
}

static size_t parser_consume_byte(scbp_parser_t *parser, uint8_t byte);

/* HCS is checked before LEN is trusted. Replay every later header byte so an
 * overlapping 5A A5 start sequence is not discarded with the bad header. */
static void parser_replay_header_suffix(scbp_parser_t *parser)
{
    uint8_t suffix[SCBP_CAN_HEADER_SIZE - 1U];

    (void)memcpy(suffix, &parser->bytes[1], sizeof(suffix));
    parser_reset(parser);
    for (uint8_t index = 0U; index < sizeof(suffix); ++index) {
        (void)parser_consume_byte(parser, suffix[index]);
    }
}

static void parser_report(scbp_parser_t *parser, scbp_parser_error_t error,
                          size_t length)
{
    if (parser->error_callback != NULL) {
        parser->error_callback(error, parser->bytes, length, parser->context);
    }
}

static uint8_t parser_verify_hcs(scbp_parser_t *parser)
{
    const uint8_t hcs = scbp_crc8_itu(&parser->bytes[2], 4U);
    const uint16_t can_id = read_u16_le(&parser->bytes[2]);
    const uint8_t flags = parser->bytes[4];
    const uint8_t source = SCBP_CAN_ID_SOURCE(can_id);
    const uint8_t destination = SCBP_CAN_ID_DESTINATION(can_id);

    parser->state = SCBP_PARSER_VERIFY_HCS;
    if (hcs != parser->bytes[6]) {
        parser_report(parser, SCBP_PARSER_ERROR_HCS, SCBP_CAN_HEADER_SIZE);
        parser_replay_header_suffix(parser);
        return 0U;
    }
    if ((flags & SCBP_CAN_FLAG_RESERVED_MASK) != 0U) {
        parser_report(parser, SCBP_PARSER_ERROR_FLAGS, SCBP_CAN_HEADER_SIZE);
        parser_replay_header_suffix(parser);
        return 0U;
    }
    if (valid_node(source) == 0U || valid_node(destination) == 0U) {
        parser_report(parser, SCBP_PARSER_ERROR_NODE, SCBP_CAN_HEADER_SIZE);
        parser_replay_header_suffix(parser);
        return 0U;
    }

    parser->payload_length = parser->bytes[5];
    parser->body_index = 0U;
    parser->state = SCBP_PARSER_READ_PAYLOAD_AND_TAIL;
    return 1U;
}

static uint8_t parser_verify_fcs_eof(scbp_parser_t *parser)
{
    const uint16_t fcs_offset = (uint16_t)SCBP_CAN_HEADER_SIZE + parser->payload_length;
    const uint16_t received_fcs = read_u16_le(&parser->bytes[fcs_offset]);
    const uint16_t expected_fcs = scbp_crc16_modbus(&parser->bytes[SCBP_CAN_HEADER_SIZE],
                                                    parser->payload_length);
    const uint8_t eof0 = parser->bytes[fcs_offset + 2U];
    const uint8_t eof1 = parser->bytes[fcs_offset + 3U];
    const uint16_t frame_length = (uint16_t)(fcs_offset + SCBP_CAN_TRAILER_SIZE);

    parser->state = SCBP_PARSER_VERIFY_FCS_EOF;
    if (received_fcs != expected_fcs) {
        parser_report(parser, SCBP_PARSER_ERROR_FCS, frame_length);
        parser_seek_sof0(parser, eof1);
        return 0U;
    }
    if (eof0 != SCBP_CAN_EOF0 || eof1 != SCBP_CAN_EOF1) {
        parser_report(parser, SCBP_PARSER_ERROR_EOF, frame_length);
        parser_seek_sof0(parser, eof1);
        return 0U;
    }

    if (parser->frame_callback != NULL) {
        scbp_can_frame_t frame;
        frame.can_id = read_u16_le(&parser->bytes[2]);
        frame.flags = parser->bytes[4];
        frame.sequence = parser->bytes[7];
        frame.length = parser->payload_length;
        frame.payload = &parser->bytes[SCBP_CAN_HEADER_SIZE];
        parser->frame_callback(&frame, parser->context);
    }
    ++parser->frame_count;
    parser_reset(parser);
    return 1U;
}

void scbp_parser_init(scbp_parser_t *parser,
                      scbp_parser_frame_callback_t frame_callback,
                      scbp_parser_error_callback_t error_callback,
                      void *context)
{
    if (parser == NULL) {
        return;
    }
    (void)memset(parser, 0, sizeof(*parser));
    parser->frame_callback = frame_callback;
    parser->error_callback = error_callback;
    parser->context = context;
    parser->state = SCBP_PARSER_WAIT_SOF0;
}

static size_t parser_consume_byte(scbp_parser_t *parser, uint8_t byte)
{
    switch (parser->state) {
    case SCBP_PARSER_WAIT_SOF0:
        if (byte == SCBP_CAN_SOF0) {
            parser->bytes[0] = byte;
            parser->state = SCBP_PARSER_WAIT_SOF1;
        }
        break;

    case SCBP_PARSER_WAIT_SOF1:
        if (byte == SCBP_CAN_SOF1) {
            parser->bytes[1] = byte;
            parser->header_index = 2U;
            parser->state = SCBP_PARSER_READ_HEADER;
        } else if (byte == SCBP_CAN_SOF0) {
            parser->bytes[0] = byte;
        } else {
            parser_reset(parser);
        }
        break;

    case SCBP_PARSER_READ_HEADER:
        parser->bytes[parser->header_index] = byte;
        ++parser->header_index;
        if (parser->header_index == SCBP_CAN_HEADER_SIZE) {
            (void)parser_verify_hcs(parser);
        }
        break;

    case SCBP_PARSER_READ_PAYLOAD_AND_TAIL:
        parser->bytes[(uint16_t)SCBP_CAN_HEADER_SIZE + parser->body_index] = byte;
        ++parser->body_index;
        if (parser->body_index == (uint16_t)parser->payload_length +
                                  SCBP_CAN_TRAILER_SIZE) {
            return parser_verify_fcs_eof(parser);
        }
        break;

    case SCBP_PARSER_VERIFY_HCS:
    case SCBP_PARSER_VERIFY_FCS_EOF:
    default:
        parser_reset(parser);
        if (byte == SCBP_CAN_SOF0) {
            parser->bytes[0] = byte;
            parser->state = SCBP_PARSER_WAIT_SOF1;
        }
        break;
    }
    return 0U;
}

size_t scbp_parser_feed(scbp_parser_t *parser, const uint8_t *data, size_t length)
{
    size_t frames = 0U;

    if (parser == NULL || (data == NULL && length != 0U)) {
        return 0U;
    }

    for (size_t index = 0U; index < length; ++index) {
        frames += parser_consume_byte(parser, data[index]);
    }
    return frames;
}

int scbp_can_encode(const scbp_can_frame_t *frame, uint8_t *out, size_t capacity,
                    uint16_t *out_length)
{
    const uint8_t source = frame == NULL ? 0U : SCBP_CAN_ID_SOURCE(frame->can_id);
    const uint8_t destination = frame == NULL ? 0U : SCBP_CAN_ID_DESTINATION(frame->can_id);
    const size_t frame_length = frame == NULL ? 0U :
        (size_t)SCBP_CAN_HEADER_SIZE + frame->length + SCBP_CAN_TRAILER_SIZE;
    uint16_t fcs;

    if (frame == NULL || out == NULL || out_length == NULL ||
        (frame->payload == NULL && frame->length != 0U)) {
        return -1;
    }
    if ((frame->flags & SCBP_CAN_FLAG_RESERVED_MASK) != 0U ||
        valid_node(source) == 0U || valid_node(destination) == 0U ||
        capacity < frame_length) {
        return -2;
    }

    out[0] = SCBP_CAN_SOF0;
    out[1] = SCBP_CAN_SOF1;
    write_u16_le(&out[2], frame->can_id);
    out[4] = frame->flags;
    out[5] = frame->length;
    out[6] = scbp_crc8_itu(&out[2], 4U);
    out[7] = frame->sequence;
    if (frame->length != 0U) {
        (void)memcpy(&out[SCBP_CAN_HEADER_SIZE], frame->payload, frame->length);
    }
    fcs = scbp_crc16_modbus(&out[SCBP_CAN_HEADER_SIZE], frame->length);
    write_u16_le(&out[SCBP_CAN_HEADER_SIZE + frame->length], fcs);
    out[SCBP_CAN_HEADER_SIZE + frame->length + 2U] = SCBP_CAN_EOF0;
    out[SCBP_CAN_HEADER_SIZE + frame->length + 3U] = SCBP_CAN_EOF1;
    *out_length = (uint16_t)frame_length;
    return 0;
}

int scbp_can_decode(const uint8_t *data, size_t length, scbp_can_frame_t *frame)
{
    uint8_t payload_length;
    uint16_t can_id;
    size_t expected;
    uint16_t fcs_offset;

    if (data == NULL || frame == NULL || length < SCBP_CAN_HEADER_SIZE +
                                                     SCBP_CAN_TRAILER_SIZE) {
        return -1;
    }
    payload_length = data[5];
    can_id = read_u16_le(&data[2]);
    expected = (size_t)SCBP_CAN_HEADER_SIZE + payload_length +
               SCBP_CAN_TRAILER_SIZE;
    fcs_offset = (uint16_t)SCBP_CAN_HEADER_SIZE + payload_length;
    if (data[0] != SCBP_CAN_SOF0 || data[1] != SCBP_CAN_SOF1 ||
        length != expected) {
        return -2;
    }
    if (scbp_crc8_itu(&data[2], 4U) != data[6] ||
        (data[4] & SCBP_CAN_FLAG_RESERVED_MASK) != 0U ||
        valid_node(SCBP_CAN_ID_SOURCE(can_id)) == 0U ||
        valid_node(SCBP_CAN_ID_DESTINATION(can_id)) == 0U) {
        return -3;
    }
    if (read_u16_le(&data[fcs_offset]) !=
            scbp_crc16_modbus(&data[SCBP_CAN_HEADER_SIZE], payload_length) ||
        data[fcs_offset + 2U] != SCBP_CAN_EOF0 ||
        data[fcs_offset + 3U] != SCBP_CAN_EOF1) {
        return -4;
    }

    frame->can_id = can_id;
    frame->flags = data[4];
    frame->sequence = data[7];
    frame->length = payload_length;
    frame->payload = &data[SCBP_CAN_HEADER_SIZE];
    return 0;
}
