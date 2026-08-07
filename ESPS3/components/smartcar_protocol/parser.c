#include "parser.h"

#include <string.h>

static void parser_reset(sc_frame_parser_t *parser)
{
    parser->length = 0U;
    parser->expected_length = 0U;
}

void sc_frame_parser_init(sc_frame_parser_t *parser,
                          sc_frame_parser_callback_t callback,
                          sc_frame_parser_error_callback_t error_callback,
                          void *context)
{
    if (parser == NULL) return;
    memset(parser, 0, sizeof(*parser));
    parser->callback = callback;
    parser->error_callback = error_callback;
    parser->context = context;
}

size_t sc_frame_parser_feed(sc_frame_parser_t *parser, const uint8_t *data,
                            size_t length)
{
    if (parser == NULL || (data == NULL && length != 0U)) return 0U;
    size_t frames = 0U;
    for (size_t i = 0U; i < length; ++i) {
        const uint8_t byte = data[i];
        if (parser->length == 0U) {
            if (byte == SC_FRAME_HEADER_0) parser->bytes[parser->length++] = byte;
            continue;
        }
        if (parser->length == 1U && byte != SC_FRAME_HEADER_1) {
            parser->length = byte == SC_FRAME_HEADER_0 ? 1U : 0U;
            if (parser->length != 0U) parser->bytes[0] = byte;
            continue;
        }
        parser->bytes[parser->length++] = byte;
        if (parser->length == 6U) {
            const uint16_t payload_length = (uint16_t)parser->bytes[4] |
                                            ((uint16_t)parser->bytes[5] << 8U);
            if (payload_length > SC_FRAME_MAX_PAYLOAD) {
                if (parser->error_callback != NULL) {
                    parser->error_callback(-4, parser->bytes, parser->length,
                                           parser->context);
                }
                parser_reset(parser);
                continue;
            }
            parser->expected_length = (uint16_t)(SC_FRAME_OVERHEAD + payload_length);
        }
        if (parser->expected_length != 0U && parser->length == parser->expected_length) {
            sc_frame_view_t view;
            const int status = sc_frame_decode(parser->bytes, parser->length, &view);
            if (status == 0) {
                if (parser->callback != NULL) parser->callback(&view, parser->context);
                ++frames;
            } else if (parser->error_callback != NULL) {
                parser->error_callback(status, parser->bytes, parser->length,
                                       parser->context);
            }
            parser_reset(parser);
        }
    }
    return frames;
}
