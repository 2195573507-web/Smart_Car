#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "radar_parser.h"
#include "radar_uplink_protocol.h"

static void write_le16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static size_t make_frame(uint8_t *frame)
{
    const size_t length = RADAR_X3PRO_HEADER_BYTES + 2U * RADAR_X3PRO_SAMPLE_BYTES;
    memset(frame, 0, length);
    frame[0] = RADAR_X3PRO_HEADER_BYTE_0;
    frame[1] = RADAR_X3PRO_HEADER_BYTE_1;
    frame[2] = 0U;
    frame[3] = 2U;
    write_le16(&frame[4], 0xAE53U);
    write_le16(&frame[6], 0xB553U);
    write_le16(&frame[10], 0x0400U);
    write_le16(&frame[12], 0x0800U);

    uint16_t checksum = 0x55AAU;
    checksum ^= read_le16(&frame[4]);
    checksum ^= read_le16(&frame[10]);
    checksum ^= read_le16(&frame[12]);
    checksum ^= (uint16_t)(((uint16_t)frame[3] << 8U) | frame[2]);
    checksum ^= read_le16(&frame[6]);
    write_le16(&frame[8], checksum);
    return length;
}

static void test_round_trip(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    uint8_t packet[RADAR_UPLINK_MAX_PACKET_SIZE];
    size_t frame_length = make_frame(frame);
    size_t packet_length = 0U;
    radar_uplink_packet_t decoded = {0};

    assert(radar_uplink_encode_frame(frame,
                                     frame_length,
                                     0x10203040U,
                                     1U,
                                     42U,
                                     123456U,
                                     packet,
                                     sizeof(packet),
                                     &packet_length) == RADAR_UPLINK_OK);
    assert(packet_length == RADAR_UPLINK_HEADER_SIZE + frame_length +
                            RADAR_UPLINK_CRC_SIZE);
    assert(radar_uplink_decode_packet(packet, packet_length, &decoded) ==
           RADAR_UPLINK_OK);
    assert(decoded.flags == 0U);
    assert(decoded.device_id == 0x10203040U);
    assert(decoded.stream_id == 1U);
    assert(decoded.sequence == 42U);
    assert(decoded.timestamp_ms == 123456U);
    assert(decoded.payload_length == frame_length);
    assert(memcmp(decoded.payload, frame, frame_length) == 0);
}

static void test_rejects_corruption_and_truncation(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    uint8_t packet[RADAR_UPLINK_MAX_PACKET_SIZE];
    size_t frame_length = make_frame(frame);
    size_t packet_length = 0U;
    radar_uplink_packet_t decoded = {0};

    assert(radar_uplink_encode_frame(frame,
                                     frame_length,
                                     1U,
                                     1U,
                                     1U,
                                     2U,
                                     packet,
                                     sizeof(packet),
                                     &packet_length) == RADAR_UPLINK_OK);
    packet[packet_length - 1U] ^= 0x01U;
    assert(radar_uplink_decode_packet(packet, packet_length, &decoded) ==
           RADAR_UPLINK_CRC_MISMATCH);

    packet[packet_length - 1U] ^= 0x01U;
    assert(radar_uplink_decode_packet(packet, packet_length - 1U, &decoded) ==
           RADAR_UPLINK_LENGTH_INVALID);
    assert(radar_uplink_encode_frame(frame,
                                     frame_length,
                                     1U,
                                     1U,
                                     1U,
                                     2U,
                                     packet,
                                     sizeof(packet),
                                     &packet_length) == RADAR_UPLINK_OK);
    packet[6] = 0x02U;
    assert(radar_uplink_decode_packet(packet, packet_length, &decoded) ==
           RADAR_UPLINK_LENGTH_INVALID);
    packet[6] = 0U;
    assert(radar_uplink_decode_packet(packet, packet_length, &decoded) ==
           RADAR_UPLINK_OK);
    assert(radar_uplink_encode_frame(frame,
                                     frame_length,
                                     1U,
                                     1U,
                                     1U,
                                     2U,
                                     packet,
                                     RADAR_UPLINK_HEADER_SIZE,
                                     &packet_length) ==
           RADAR_UPLINK_BUFFER_TOO_SMALL);
}

static void test_rejects_invalid_lidar_frame(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    uint8_t packet[RADAR_UPLINK_MAX_PACKET_SIZE];
    size_t frame_length = make_frame(frame);
    size_t packet_length = 0U;

    frame[8] ^= 0x10U;
    assert(radar_uplink_encode_frame(frame,
                                     frame_length,
                                     1U,
                                     1U,
                                     1U,
                                     2U,
                                     packet,
                                     sizeof(packet),
                                     &packet_length) == RADAR_UPLINK_FRAME_INVALID);
}

static void test_zero_packet_flag_and_header_validation(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    uint8_t packet[RADAR_UPLINK_MAX_PACKET_SIZE];
    size_t frame_length = make_frame(frame);
    size_t packet_length = 0U;
    radar_uplink_packet_t decoded = {0};

    frame[2] = 0x01U;
    frame[8] ^= 0x01U; /* CT is part of the official LSN/CT XOR word. */
    assert(radar_uplink_encode_frame(frame,
                                     frame_length,
                                     1U,
                                     2U,
                                     3U,
                                     4U,
                                     packet,
                                     sizeof(packet),
                                     &packet_length) == RADAR_UPLINK_OK);
    assert(read_le16(&packet[6]) == RADAR_UPLINK_FLAG_ZERO_PACKET);
    assert(radar_uplink_decode_packet(packet, packet_length, &decoded) ==
           RADAR_UPLINK_OK);
    assert(decoded.flags == RADAR_UPLINK_FLAG_ZERO_PACKET);

    packet[4] = (uint8_t)(RADAR_UPLINK_PROTOCOL_VERSION + 1U);
    assert(radar_uplink_decode_packet(packet, packet_length, &decoded) ==
           RADAR_UPLINK_VERSION_UNSUPPORTED);
    packet[4] = RADAR_UPLINK_PROTOCOL_VERSION;
    packet[5] = (uint8_t)(RADAR_UPLINK_MESSAGE_RAW_FRAME + 1U);
    assert(radar_uplink_decode_packet(packet, packet_length, &decoded) ==
           RADAR_UPLINK_MESSAGE_UNSUPPORTED);
}

static void test_maximum_frame_round_trip(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    uint8_t packet[RADAR_UPLINK_MAX_PACKET_SIZE];
    size_t packet_length = 0U;
    radar_uplink_packet_t decoded = {0};
    memset(frame, 0, sizeof(frame));

    /* Build a maximum-size intensity frame and calculate its official XOR. */
    frame[0] = RADAR_X3PRO_HEADER_BYTE_0;
    frame[1] = RADAR_X3PRO_HEADER_BYTE_1;
    frame[2] = 0x00U;
    frame[3] = RADAR_X3PRO_MAX_SAMPLES;
    write_le16(&frame[4], 0xAE53U);
    write_le16(&frame[6], 0xAE53U);
    for (size_t index = 0U; index < RADAR_X3PRO_MAX_SAMPLES; ++index) {
        const size_t offset = RADAR_X3PRO_HEADER_BYTES +
                              index * RADAR_X3PRO_MAX_SAMPLE_BYTES;
        frame[offset] = (uint8_t)index;
        write_le16(&frame[offset + 1U], (uint16_t)(0x0400U + index));
    }
    uint16_t checksum = 0x55AAU;
    checksum ^= read_le16(&frame[4]);
    for (size_t index = 0U; index < RADAR_X3PRO_MAX_SAMPLES; ++index) {
        const size_t offset = RADAR_X3PRO_HEADER_BYTES +
                              index * RADAR_X3PRO_MAX_SAMPLE_BYTES;
        checksum ^= frame[offset];
        checksum ^= read_le16(&frame[offset + 1U]);
    }
    checksum ^= (uint16_t)(((uint16_t)frame[3] << 8U) | frame[2]);
    checksum ^= read_le16(&frame[6]);
    write_le16(&frame[8], checksum);

    assert(radar_uplink_encode_frame(frame,
                                     sizeof(frame),
                                     UINT32_C(0xAABBCCDD),
                                     UINT32_C(0x11223344),
                                     UINT32_C(0x55667788),
                                     UINT32_C(0x99AABBCC),
                                     packet,
                                     sizeof(packet),
                                     &packet_length) == RADAR_UPLINK_OK);
    assert(packet_length == RADAR_UPLINK_MAX_PACKET_SIZE);
    assert(radar_uplink_decode_packet(packet, packet_length, &decoded) ==
           RADAR_UPLINK_OK);
    assert(decoded.payload_length == RADAR_PARSER_MAX_FRAME_SIZE);
    assert(decoded.device_id == UINT32_C(0xAABBCCDD));
    assert(decoded.stream_id == UINT32_C(0x11223344));
    assert(decoded.sequence == UINT32_C(0x55667788));
    assert(decoded.timestamp_ms == UINT32_C(0x99AABBCC));
}

int main(void)
{
    test_round_trip();
    test_rejects_corruption_and_truncation();
    test_rejects_invalid_lidar_frame();
    test_zero_packet_flag_and_header_validation();
    test_maximum_frame_round_trip();
    return 0;
}
