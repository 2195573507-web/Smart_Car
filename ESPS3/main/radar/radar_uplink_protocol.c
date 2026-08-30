#include "radar_uplink_protocol.h"

#include <string.h>

static uint16_t crc16_modbus(const uint8_t *data, size_t length)
{
    uint16_t crc = UINT16_C(0xFFFF);
    for (size_t index = 0U; index < length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 1U) != 0U
                      ? (uint16_t)((crc >> 1U) ^ UINT16_C(0xA001))
                      : (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t read_le32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

static void write_le16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void write_le32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

static bool has_magic(const uint8_t *packet)
{
    return packet[0] == RADAR_UPLINK_MAGIC_0 &&
           packet[1] == RADAR_UPLINK_MAGIC_1 &&
           packet[2] == RADAR_UPLINK_MAGIC_2 &&
           packet[3] == RADAR_UPLINK_MAGIC_3;
}

radar_uplink_status_t radar_uplink_encode_envelope(
    const uint8_t *payload,
    size_t payload_length,
    uint8_t message_type,
    uint16_t flags,
    uint32_t device_id,
    uint32_t stream_id,
    uint32_t sequence,
    uint32_t timestamp_ms,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length)
{
    size_t packet_length;
    uint16_t crc;

    if (output_length == NULL) {
        return RADAR_UPLINK_INVALID_ARG;
    }
    *output_length = 0U;
    if (output == NULL || (payload == NULL && payload_length != 0U)) {
        return RADAR_UPLINK_INVALID_ARG;
    }
    if (message_type == 0U) {
        return RADAR_UPLINK_MESSAGE_UNSUPPORTED;
    }
    if (payload_length > RADAR_UPLINK_MAX_PAYLOAD_SIZE ||
        payload_length > UINT16_MAX) {
        return RADAR_UPLINK_LENGTH_INVALID;
    }
    if (payload_length > SIZE_MAX - RADAR_UPLINK_MIN_PACKET_SIZE) {
        return RADAR_UPLINK_LENGTH_INVALID;
    }
    packet_length = RADAR_UPLINK_MIN_PACKET_SIZE + payload_length;
    if (output_capacity < packet_length) {
        return RADAR_UPLINK_BUFFER_TOO_SMALL;
    }

    output[0] = RADAR_UPLINK_MAGIC_0;
    output[1] = RADAR_UPLINK_MAGIC_1;
    output[2] = RADAR_UPLINK_MAGIC_2;
    output[3] = RADAR_UPLINK_MAGIC_3;
    output[4] = RADAR_UPLINK_PROTOCOL_VERSION;
    output[5] = message_type;
    write_le16(&output[6], flags);
    write_le32(&output[8], device_id);
    write_le32(&output[12], stream_id);
    write_le32(&output[16], sequence);
    write_le32(&output[20], timestamp_ms);
    write_le16(&output[24], (uint16_t)payload_length);
    if (payload_length != 0U) {
        memcpy(&output[RADAR_UPLINK_HEADER_SIZE], payload, payload_length);
    }
    crc = crc16_modbus(&output[4], RADAR_UPLINK_HEADER_SIZE - 4U + payload_length);
    write_le16(&output[RADAR_UPLINK_HEADER_SIZE + payload_length], crc);
    *output_length = packet_length;
    return RADAR_UPLINK_OK;
}

radar_uplink_status_t radar_uplink_decode_envelope(
    const uint8_t *packet,
    size_t packet_length,
    radar_uplink_packet_t *decoded)
{
    size_t payload_length;
    size_t expected_length;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if (packet == NULL || decoded == NULL) {
        return RADAR_UPLINK_INVALID_ARG;
    }
    if (packet_length < RADAR_UPLINK_MIN_PACKET_SIZE ||
        !has_magic(packet)) {
        return RADAR_UPLINK_LENGTH_INVALID;
    }
    if (packet[4] != RADAR_UPLINK_PROTOCOL_VERSION) {
        return RADAR_UPLINK_VERSION_UNSUPPORTED;
    }
    if (packet[5] == 0U) {
        return RADAR_UPLINK_MESSAGE_UNSUPPORTED;
    }
    payload_length = read_le16(&packet[24]);
    if (payload_length > SIZE_MAX - RADAR_UPLINK_MIN_PACKET_SIZE) {
        return RADAR_UPLINK_LENGTH_INVALID;
    }
    expected_length = RADAR_UPLINK_MIN_PACKET_SIZE + payload_length;
    if (payload_length > RADAR_UPLINK_MAX_PAYLOAD_SIZE ||
        packet_length != expected_length) {
        return RADAR_UPLINK_LENGTH_INVALID;
    }

    received_crc = read_le16(&packet[RADAR_UPLINK_HEADER_SIZE + payload_length]);
    calculated_crc = crc16_modbus(&packet[4],
                                  RADAR_UPLINK_HEADER_SIZE - 4U + payload_length);
    if (received_crc != calculated_crc) {
        return RADAR_UPLINK_CRC_MISMATCH;
    }

    decoded->version = packet[4];
    decoded->message_type = packet[5];
    decoded->flags = read_le16(&packet[6]);
    decoded->device_id = read_le32(&packet[8]);
    decoded->stream_id = read_le32(&packet[12]);
    decoded->sequence = read_le32(&packet[16]);
    decoded->timestamp_ms = read_le32(&packet[20]);
    decoded->payload = &packet[RADAR_UPLINK_HEADER_SIZE];
    decoded->payload_length = payload_length;
    return RADAR_UPLINK_OK;
}

radar_uplink_status_t radar_uplink_encode_frame(
    const uint8_t *frame,
    size_t frame_length,
    uint32_t device_id,
    uint32_t stream_id,
    uint32_t sequence,
    uint32_t timestamp_ms,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length)
{
    if (output_length == NULL || output == NULL ||
        (frame == NULL && frame_length != 0U)) {
        return RADAR_UPLINK_INVALID_ARG;
    }
    if (frame_length == 0U || frame_length > RADAR_PARSER_MAX_FRAME_SIZE) {
        return RADAR_UPLINK_FRAME_INVALID;
    }
    if (!radar_parser_validate_frame(frame, frame_length, NULL)) {
        return RADAR_UPLINK_FRAME_INVALID;
    }

    const uint16_t flags =
        (frame[2] & 0x01U) != 0U ? RADAR_UPLINK_FLAG_ZERO_PACKET : 0U;
    return radar_uplink_encode_envelope(frame,
                                        frame_length,
                                        RADAR_UPLINK_MESSAGE_RAW_FRAME,
                                        flags,
                                        device_id,
                                        stream_id,
                                        sequence,
                                        timestamp_ms,
                                        output,
                                        output_capacity,
                                        output_length);
}

radar_uplink_status_t radar_uplink_decode_packet(
    const uint8_t *packet,
    size_t packet_length,
    radar_uplink_packet_t *decoded)
{
    /* Preserve the legacy decoder's header rejection order and status codes. */
    if (packet == NULL || decoded == NULL) {
        return RADAR_UPLINK_INVALID_ARG;
    }
    if (packet_length < RADAR_UPLINK_MIN_PACKET_SIZE ||
        !has_magic(packet)) {
        return RADAR_UPLINK_LENGTH_INVALID;
    }
    if (packet[4] != RADAR_UPLINK_PROTOCOL_VERSION) {
        return RADAR_UPLINK_VERSION_UNSUPPORTED;
    }
    if (packet[5] != RADAR_UPLINK_MESSAGE_RAW_FRAME) {
        return RADAR_UPLINK_MESSAGE_UNSUPPORTED;
    }
    const uint16_t header_flags = read_le16(&packet[6]);
    if ((header_flags & (uint16_t)~RADAR_UPLINK_FLAG_ZERO_PACKET) != 0U) {
        return RADAR_UPLINK_LENGTH_INVALID;
    }

    radar_uplink_status_t status =
        radar_uplink_decode_envelope(packet, packet_length, decoded);
    if (status != RADAR_UPLINK_OK) {
        return status;
    }
    if (decoded->message_type != RADAR_UPLINK_MESSAGE_RAW_FRAME) {
        return RADAR_UPLINK_MESSAGE_UNSUPPORTED;
    }
    if (decoded->payload_length == 0U ||
        decoded->payload_length > RADAR_PARSER_MAX_FRAME_SIZE) {
        return RADAR_UPLINK_LENGTH_INVALID;
    }
    if (!radar_parser_validate_frame(decoded->payload,
                                     decoded->payload_length,
                                     NULL)) {
        return RADAR_UPLINK_FRAME_INVALID;
    }
    const bool zero_packet = (decoded->payload[2] & 0x01U) != 0U;
    if (((decoded->flags & RADAR_UPLINK_FLAG_ZERO_PACKET) != 0U) !=
        zero_packet) {
        return RADAR_UPLINK_FRAME_INVALID;
    }
    return RADAR_UPLINK_OK;
}
