#ifndef S3_RADAR_UPLINK_PROTOCOL_H
#define S3_RADAR_UPLINK_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "radar_parser.h"
#include "srp_registry.h"

/* Experimental S3-to-host framing; payload policy remains message-type specific. */
#define RADAR_UPLINK_MAGIC_0 0x53U /* 'S' */
#define RADAR_UPLINK_MAGIC_1 0x33U /* '3' */
#define RADAR_UPLINK_MAGIC_2 0x52U /* 'R' */
#define RADAR_UPLINK_MAGIC_3 0x44U /* 'D' */
#define RADAR_UPLINK_PROTOCOL_VERSION 1U
/* Kept as the original public name for source compatibility. */
#define RADAR_UPLINK_MESSAGE_RAW_FRAME 1U
#define RADAR_UPLINK_MESSAGE_TYPE_RAW_FRAME RADAR_UPLINK_MESSAGE_RAW_FRAME
/* Candidate value only: SRPv4 telemetry type is experimental until joint
 * S3/Windows review freezes the ID. */
#define RADAR_UPLINK_MESSAGE_SRP_TELEMETRY_EXPERIMENTAL 2U
#define RADAR_UPLINK_FLAG_ZERO_PACKET 0x0001U
#define RADAR_UPLINK_HEADER_SIZE 26U
#define RADAR_UPLINK_CRC_SIZE 2U
#define RADAR_UPLINK_MIN_PACKET_SIZE \
    (RADAR_UPLINK_HEADER_SIZE + RADAR_UPLINK_CRC_SIZE)
/*
 * Keep the envelope large enough for both the current YDLIDAR parser and a
 * complete SRPv4 frame.  The shared protocol definition is the authority
 * for the latter limit; do not duplicate that value here.
 */
#define RADAR_UPLINK_MAX_PAYLOAD_SIZE \
    ((RADAR_PARSER_MAX_FRAME_SIZE > SRP_MAX_FRAME_SIZE) \
         ? RADAR_PARSER_MAX_FRAME_SIZE \
         : SRP_MAX_FRAME_SIZE)
#define RADAR_UPLINK_MAX_PACKET_SIZE \
    (RADAR_UPLINK_HEADER_SIZE + RADAR_UPLINK_MAX_PAYLOAD_SIZE + \
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
    /* Appended to preserve the existing positional member order. */
    uint8_t version;
    uint8_t message_type;
} radar_uplink_packet_t;

/*
 * Encode one generic S3RD envelope.  The payload is copied into output and
 * may use any nonzero message type and any 16-bit flags value.  A zero-length
 * payload is valid when payload is NULL; message type zero is reserved.
 */
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
    size_t *output_length);

/*
 * Decode and validate only the generic S3RD envelope.  Unknown nonzero
 * message types and arbitrary flags are deliberately accepted; callers own
 * type-specific payload validation.  decoded->payload points into packet.
 */
radar_uplink_status_t radar_uplink_decode_envelope(
    const uint8_t *packet,
    size_t packet_length,
    radar_uplink_packet_t *decoded);

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
