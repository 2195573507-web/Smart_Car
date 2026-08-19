#include "parser.h"

#include <string.h>

static void parser_reset(sc_frame_parser_t *parser)
{
    parser->length = 0U;
    parser->expected_length = 0U;
}

static void parser_discard_and_seek_aa(sc_frame_parser_t *parser)
{
    uint16_t index = parser->length;

    while (index > 1U) {
        --index;
        if (parser->bytes[index] == SC_FRAME_HEADER_0) {
            parser->bytes[0] = SC_FRAME_HEADER_0;
            parser->length = 1U;
            parser->expected_length = 0U;
            return;
        }
    }
    parser_reset(parser);
}

static uint8_t parser_sequence_status(sc_frame_parser_t *parser, uint8_t src,
                                      uint8_t sequence)
{
    const uint8_t mask = (uint8_t)(1U << (src & 7U));
    const uint8_t byte_index = (uint8_t)(src >> 3U);
    uint8_t previous;
    uint8_t delta;

    if ((parser->sequence_seen[byte_index] & mask) == 0U) {
        parser->sequence_seen[byte_index] |= mask;
        parser->sequence_last[src] = sequence;
        return SCBP_SEQUENCE_FIRST;
    }
    previous = parser->sequence_last[src];
    delta = (uint8_t)(sequence - previous);
    if (delta == 1U) {
        parser->sequence_last[src] = sequence;
        return SCBP_SEQUENCE_IN_ORDER;
    }
    if (delta == 0U) {
        return SCBP_SEQUENCE_DUPLICATE;
    }
    if (delta < UINT8_C(128)) {
        parser->sequence_last[src] = sequence;
        return SCBP_SEQUENCE_GAP;
    }
    return SCBP_SEQUENCE_OUT_OF_ORDER;
}

void sc_frame_parser_init(sc_frame_parser_t *parser,
                          sc_frame_parser_callback_t callback,
                          sc_frame_parser_error_callback_t error_callback,
                          void *context)
{
    if (parser == NULL) {
        return;
    }
    memset(parser, 0, sizeof(*parser));
    parser->callback = callback;
    parser->error_callback = error_callback;
    parser->context = context;
}

size_t sc_frame_parser_feed(sc_frame_parser_t *parser, const uint8_t *data,
                            size_t length)
{
    size_t frames = 0U;

    if (parser == NULL || (data == NULL && length != 0U)) {
        return 0U;
    }
    for (size_t index = 0U; index < length; ++index) {
        const uint8_t byte = data[index];

        if (parser->length == 0U) {
            if (byte == SC_FRAME_HEADER_0) {
                parser->bytes[parser->length++] = byte;
            }
            continue;
        }
        if (parser->length == 1U && byte != SC_FRAME_HEADER_1) {
            if (parser->error_callback != NULL) {
                parser->error_callback(SC_FRAME_ERROR_AA55_FAIL, parser->bytes,
                                       parser->length, parser->context);
            }
            parser->length = byte == SC_FRAME_HEADER_0 ? 1U : 0U;
            if (parser->length != 0U) {
                parser->bytes[0] = byte;
            }
            continue;
        }
        if (parser->length >= sizeof(parser->bytes)) {
            parser_discard_and_seek_aa(parser);
            continue;
        }
        parser->bytes[parser->length++] = byte;
        if (parser->length == 12U) {
            const uint16_t payload_length = (uint16_t)parser->bytes[10] |
                                            ((uint16_t)parser->bytes[11] << 8U);
            if (payload_length > SC_FRAME_MAX_PAYLOAD) {
                if (parser->error_callback != NULL) {
                    parser->error_callback(SC_FRAME_ERROR_LEN_FAIL, parser->bytes,
                                           parser->length, parser->context);
                }
                parser_discard_and_seek_aa(parser);
                continue;
            }
            parser->expected_length = (uint16_t)(SC_FRAME_OVERHEAD + payload_length);
        }
        if (parser->expected_length != 0U &&
            parser->length == parser->expected_length) {
            sc_frame_view_t view;
            const int status = scbp_frame_decode(parser->bytes, parser->length,
                                                 &view);

            ++parser->frame_index;
            if (status == 0) {
                view.sequence_status = parser_sequence_status(parser, view.src,
                                                               view.seq);
                if (parser->callback != NULL) {
                    parser->callback(&view, parser->context);
                }
                ++frames;
                parser_reset(parser);
            } else {
                if (parser->error_callback != NULL) {
                    parser->error_callback(status, parser->bytes, parser->length,
                                           parser->context);
                }
                parser_discard_and_seek_aa(parser);
            }
        }
    }
    return frames;
}
