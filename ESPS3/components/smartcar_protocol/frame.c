#include "frame.h"

#include <string.h>

uint16_t sc_frame_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    if (data == NULL && length != 0U) return 0U;
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
    if (out == NULL || out_length == NULL || (payload == NULL && length != 0U)) return -1;
    if (length > SC_FRAME_MAX_PAYLOAD || capacity < SC_FRAME_OVERHEAD + length) return -2;
    out[0] = SC_FRAME_HEADER_0; out[1] = SC_FRAME_HEADER_1;
    out[2] = SC_FRAME_VERSION; out[3] = type;
    out[4] = (uint8_t)(length & 0xFFU); out[5] = (uint8_t)(length >> 8U);
    if (length != 0U) memcpy(&out[6], payload, length);
    const uint16_t crc = sc_frame_crc16(&out[2], 4U + length);
    out[6U + length] = (uint8_t)(crc & 0xFFU);
    out[7U + length] = (uint8_t)(crc >> 8U);
    *out_length = (uint16_t)(SC_FRAME_OVERHEAD + length);
    return 0;
}

int sc_frame_decode(const uint8_t *frame, size_t length, sc_frame_view_t *view)
{
    if (frame == NULL || view == NULL || length < SC_FRAME_OVERHEAD) return -1;
    if (frame[0] != SC_FRAME_HEADER_0 || frame[1] != SC_FRAME_HEADER_1) return -2;
    if (frame[2] != SC_FRAME_VERSION) return -3;
    const uint16_t payload_length = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8U);
    if (payload_length > SC_FRAME_MAX_PAYLOAD || length != SC_FRAME_OVERHEAD + payload_length) return -4;
    const uint16_t received = (uint16_t)frame[6U + payload_length] |
                              ((uint16_t)frame[7U + payload_length] << 8U);
    if (received != sc_frame_crc16(&frame[2], 4U + payload_length)) return -5;
    view->version = frame[2]; view->type = frame[3]; view->length = payload_length;
    view->payload = &frame[6];
    return 0;
}
