#ifndef S3_RADAR_UPLINK_PROTOCOL_H
#define S3_RADAR_UPLINK_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "radar_parser.h"

/* Experimental S3-to-host framing. Keep this separate from SCBP and BLE logs. */
#define RADAR_UPLINK_MAGIC_0 0x53U /* 'S' */
#define RADAR_UPLINK_MAGIC_1 0x33U /* '3' */
#define RADAR_UPLINK_MAGIC_2 0x52U /* 'R' */
#define RADAR_UPLINK_MAGIC_3 0x44U /* 'D' */
#define RADAR_UPLINK_PROTOCOL_VERSION 1U
#define RADAR_UPLINK_MESSAGE_RAW_FRAME 1U
#define RADAR_UPLINK_FLAG_ZERO_PACKET 0x0001U
#define RADAR_UPLINK_HEADER_SIZE 26U
#define RADAR_UPLINK_CRC_SIZE 2U
#define RADAR_UPLINK_MAX_PACKET_SIZE \
    (RADAR_UPLINK_HEADER_SIZE + RADAR_PARSER_MAX_FRAME_SIZE + \
     RADAR_UPLINK_CRC_SIZE)

typedef enum {
    RADAR_UPLINK_OK = 0,
    RADAR_UPLINK_INVALID_ARG,
    RADAR_UPLINK_FRAME_INVALID,
    RADAR_UPLINK_BUFFER_TOO_SMALL,
    RADAR_UPLINK_LENGTH_INVALID,
    RADAR_UPLINK_VERSION_UNSUPPORTED,
    RADAR_UPLINK_MESSAGE_UNSUPPORTED,
    RADAR_UPLINK_CRC_MISMATCH
} radar_uplink_status_t;

typedef struct {
    uint16_t flags;
    uint32_t device_id;
    uint32_t stream_id;
    uint32_t sequence;
    uint32_t timestamp_ms;
    const uint8_t *payload;
    size_t payload_length;
} radar_uplink_packet_t;

radar_uplink_status_t radar_uplink_encode_frame(
    const uint8_t *frame,
    size_t frame_length,
    uint32_t device_id,
    uint32_t stream_id,
    uint32_t sequence,
    uint32_t timestamp_ms,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length);

radar_uplink_status_t radar_uplink_decode_packet(
    const uint8_t *packet,
    size_t packet_length,
    radar_uplink_packet_t *decoded);

#endif
