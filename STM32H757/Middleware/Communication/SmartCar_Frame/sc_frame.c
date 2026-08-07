#include "sc_frame.h"

#include <string.h>

uint16_t sc_frame_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    if (data == NULL && length != 0U) {
        return 0U;
    }
    for (size_t index = 0U; index < length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) != 0U ? (uint16_t)((crc >> 1U) ^ 0xA001U)
                                   : (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

int sc_frame_encode(uint8_t type, const uint8_t *payload, uint16_t length,
                    uint8_t *out, size_t capacity, uint16_t *out_length)
{
    if (out == NULL || out_length == NULL || (payload == NULL && length != 0U)) {
        return -1;
    }
    if (length > SC_FRAME_MAX_PAYLOAD || capacity < SC_FRAME_OVERHEAD + length) {
        return -2;
    }
    out[0] = SC_FRAME_HEADER_0;
    out[1] = SC_FRAME_HEADER_1;
    out[2] = SC_FRAME_VERSION;
    out[3] = type;
    out[4] = (uint8_t)(length & 0xFFU);
    out[5] = (uint8_t)(length >> 8U);
    if (length != 0U) {
        memcpy(&out[6], payload, length);
    }
    const uint16_t crc = sc_frame_crc16(&out[2], 4U + length);
    out[6U + length] = (uint8_t)(crc & 0xFFU);
    out[7U + length] = (uint8_t)(crc >> 8U);
    *out_length = (uint16_t)(SC_FRAME_OVERHEAD + length);
    return 0;
}

int sc_frame_decode(const uint8_t *frame, size_t length, sc_frame_view_t *view)
{
    if (frame == NULL || view == NULL || length < SC_FRAME_OVERHEAD) return -1;
    if (frame[0] != SC_FRAME_HEADER_0 || frame[1] != SC_FRAME_HEADER_1) {
        return SC_FRAME_ERROR_AA55_FAIL;
    }
    if (frame[2] != SC_FRAME_VERSION) return SC_FRAME_ERROR_VERSION_FAIL;
    const uint16_t payload_length = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8U);
    if (payload_length > SC_FRAME_MAX_PAYLOAD ||
        length != SC_FRAME_OVERHEAD + payload_length) {
        return SC_FRAME_ERROR_LEN_FAIL;
    }
    const uint16_t received = (uint16_t)frame[6U + payload_length] |
                              ((uint16_t)frame[7U + payload_length] << 8U);
    if (received != sc_frame_crc16(&frame[2], 4U + payload_length)) {
        return SC_FRAME_ERROR_CRC_FAIL;
    }
    view->version = frame[2]; view->type = frame[3]; view->length = payload_length;
    view->payload = &frame[6];
    return 0;
}

static void parser_reset(sc_frame_parser_t *parser)
{
    parser->length = 0U;
    parser->expected_length = 0U;
}

/* Preserve the latest possible header start after rejecting a malformed LEN. */
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

void sc_frame_parser_init(sc_frame_parser_t *parser, sc_frame_callback_t callback,
                          sc_frame_error_callback_t error_callback, void *context)
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
            if (parser->error_callback != NULL) {
                parser->error_callback(SC_FRAME_ERROR_AA55_FAIL,
                                       parser->bytes, parser->length,
                                       parser->context);
            }
            parser->length = byte == SC_FRAME_HEADER_0 ? 1U : 0U;
            if (parser->length != 0U) parser->bytes[0] = byte;
            continue;
        }
        if (parser->length >= sizeof(parser->bytes)) {
            parser_reset(parser); continue;
        }
        parser->bytes[parser->length++] = byte;
        if (parser->length == 6U) {
            const uint16_t payload_length = (uint16_t)parser->bytes[4] |
                                            ((uint16_t)parser->bytes[5] << 8U);
            if (payload_length > SC_FRAME_MAX_PAYLOAD) {
                if (parser->error_callback != NULL) {
                    parser->error_callback(SC_FRAME_ERROR_LEN_FAIL,
                                           parser->bytes, parser->length,
                                           parser->context);
                }
                parser_discard_and_seek_aa(parser);
                continue;
            }
            parser->expected_length = (uint16_t)(SC_FRAME_OVERHEAD + payload_length);
        }
        if (parser->expected_length != 0U && parser->length == parser->expected_length) {
            sc_frame_view_t view;
            ++parser->frame_index;
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
