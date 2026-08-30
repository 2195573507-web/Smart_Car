#include "app_parser.h"

#include <string.h>

#include "srp_crc.h"

static void app_parser_reset(sc_app_parser_t *parser)
{
    parser->length = 0U;
    parser->expected_length = 0U;
}

static void app_parser_report_error(sc_app_parser_t *parser, int error)
{
    if (parser->error_callback != NULL) {
        parser->error_callback(error, parser->bytes, parser->length,
                               parser->context);
    }
}

void sc_app_parser_init(sc_app_parser_t *parser,
                        sc_app_parser_callback_t callback,
                        sc_app_parser_error_callback_t error_callback,
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

size_t sc_app_parser_feed(sc_app_parser_t *parser, const uint8_t *data,
                          size_t length)
{
    if (parser == NULL || (data == NULL && length != 0U)) {
        return 0U;
    }

    size_t frames = 0U;
    for (size_t index = 0U; index < length; ++index) {
        const uint8_t byte = data[index];

        if (parser->length == 0U) {
            if (byte == SC_APP_FRAME_HEAD) {
                parser->bytes[parser->length++] = byte;
            }
            continue;
        }
        if (parser->length == 1U) {
            /* Accept only the two explicitly supported App-BLE versions. */
            if (byte == SC_APP_FRAME_VERSION ||
                byte == SC_APP_FRAME_VERSION_V2) {
                parser->bytes[parser->length++] = byte;
            } else if (byte == SC_APP_FRAME_HEAD) {
                parser->bytes[0] = byte;
            } else {
                app_parser_reset(parser);
            }
            continue;
        }
        parser->bytes[parser->length++] = byte;

        if (parser->length == 5U) {
            const uint16_t payload_length = (uint16_t)parser->bytes[3] |
                                            ((uint16_t)parser->bytes[4] << 8U);
            if (payload_length > SC_APP_FRAME_MAX_PAYLOAD) {
                app_parser_report_error(parser, -4);
                app_parser_reset(parser);
                if (byte == SC_APP_FRAME_HEAD) {
                    parser->bytes[parser->length++] = byte;
                }
                continue;
            }
            parser->expected_length =
                (uint16_t)(SC_APP_FRAME_OVERHEAD + payload_length);
        }

        if (parser->expected_length != 0U &&
            parser->length == parser->expected_length) {
            const uint16_t payload_length =
                (uint16_t)parser->bytes[3] |
                ((uint16_t)parser->bytes[4] << 8U);
            const uint16_t received_crc =
                (uint16_t)parser->bytes[5U + payload_length] |
                ((uint16_t)parser->bytes[6U + payload_length] << 8U);
            const uint16_t calculated_crc =
                srp_crc16_modbus(&parser->bytes[1], 4U + payload_length);

            if (parser->bytes[1] != SC_APP_FRAME_VERSION &&
                parser->bytes[1] != SC_APP_FRAME_VERSION_V2) {
                app_parser_report_error(parser, -3);
            } else if (parser->bytes[7U + payload_length] !=
                       SC_APP_FRAME_TAIL) {
                app_parser_report_error(parser, -2);
            } else if (received_crc != calculated_crc) {
                app_parser_report_error(parser, -5);
            } else {
                sc_app_frame_view_t view = {
                    .version = parser->bytes[1],
                    .type = parser->bytes[2],
                    .length = payload_length,
                    .payload = &parser->bytes[5],
                };
                if (parser->callback != NULL) {
                    parser->callback(&view, parser->context);
                }
                ++frames;
            }
            app_parser_reset(parser);
        }
    }
    return frames;
}

int sc_app_frame_encode(uint8_t type, const uint8_t *payload, uint16_t length,
                        uint8_t *out, size_t capacity, uint16_t *out_length)
{
    return sc_app_frame_encode_version(SC_APP_FRAME_VERSION, type, payload,
                                       length, out, capacity, out_length);
}

int sc_app_frame_encode_version(uint8_t version, uint8_t type,
                                const uint8_t *payload, uint16_t length,
                                uint8_t *out, size_t capacity,
                                uint16_t *out_length)
{
    if (out == NULL || out_length == NULL ||
        (payload == NULL && length != 0U)) {
        return -1;
    }
    if ((version != SC_APP_FRAME_VERSION &&
         version != SC_APP_FRAME_VERSION_V2) ||
        length > SC_APP_FRAME_MAX_PAYLOAD ||
        capacity < SC_APP_FRAME_OVERHEAD + length) {
        return -2;
    }

    out[0] = SC_APP_FRAME_HEAD;
    out[1] = version;
    out[2] = type;
    out[3] = (uint8_t)(length & 0xFFU);
    out[4] = (uint8_t)(length >> 8U);
    if (length != 0U) {
        memcpy(&out[5], payload, length);
    }
    const uint16_t crc = srp_crc16_modbus(&out[1], 4U + length);
    out[5U + length] = (uint8_t)(crc & 0xFFU);
    out[6U + length] = (uint8_t)(crc >> 8U);
    out[7U + length] = SC_APP_FRAME_TAIL;
    *out_length = (uint16_t)(SC_APP_FRAME_OVERHEAD + length);
    return 0;
}
