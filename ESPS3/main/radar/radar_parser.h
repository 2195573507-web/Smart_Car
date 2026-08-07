#ifndef S3_RADAR_PARSER_H
#define S3_RADAR_PARSER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * X3PRO scan packets are little-endian and start with the byte stream AA 55
 * (the protocol value is 0x55AA).  The packet header contains CT, LSN, FSA,
 * LSA, and CS.  LSN is the number of samples that follow the ten-byte header.
 */
#define RADAR_X3PRO_HEADER_BYTE_0 0xAAU
#define RADAR_X3PRO_HEADER_BYTE_1 0x55U
#define RADAR_X3PRO_LENGTH_OFFSET 3U
#define RADAR_X3PRO_HEADER_BYTES 10U
#define RADAR_X3PRO_SAMPLE_BYTES 2U
#define RADAR_X3PRO_MAX_SAMPLE_BYTES 3U
#define RADAR_X3PRO_MAX_SAMPLES 255U

#define RADAR_PARSER_RING_BUFFER_SIZE 4096U
#define RADAR_PARSER_MAX_FRAME_SIZE \
    (RADAR_X3PRO_HEADER_BYTES + \
     (RADAR_X3PRO_MAX_SAMPLES * RADAR_X3PRO_MAX_SAMPLE_BYTES))

typedef void (*radar_frame_callback_t)(const uint8_t *data,
                                       size_t length,
                                       void *context);

/*
 * The checksum algorithm differs between protocol variants.  The first-stage
 * receiver leaves validation disabled until the exact X3PRO variant is
 * confirmed from a captured device frame.  A caller can install its own
 * validator without changing ring-buffer or framing logic.
 */
typedef bool (*radar_parser_checksum_validator_t)(const uint8_t *frame,
                                                   size_t length,
                                                   void *context);

/* Reserved result shape for a future point decoder. */
typedef struct {
    bool valid;
    uint16_t angle_cdeg;
    uint16_t distance_mm;
    uint16_t quality;
} radar_measurement_t;

typedef struct {
    uint8_t buffer[RADAR_PARSER_RING_BUFFER_SIZE];
    size_t head;
    size_t tail;
    size_t size;
    size_t sample_bytes;
    radar_parser_checksum_validator_t checksum_validator;
    void *checksum_context;
    size_t overflow_count;
} radar_parser_t;

void radar_parser_init(radar_parser_t *parser);

void radar_parser_set_checksum_validator(
    radar_parser_t *parser,
    radar_parser_checksum_validator_t validator,
    void *context);

/* Use two-byte distance samples by default; three-byte intensity samples are
 * supported as a framing option for compatible protocol variants. */
bool radar_parser_set_sample_bytes(radar_parser_t *parser, size_t sample_bytes);

void radar_parser_feed(radar_parser_t *parser,
                       const uint8_t *data,
                       size_t length,
                       radar_frame_callback_t callback,
                       void *context);

/* Reserved until the X3PRO point-field variant is confirmed; never fabricates
 * angle, distance, or quality values. */
bool radar_parser_parse_measurement(const uint8_t *frame,
                                    size_t length,
                                    radar_measurement_t *measurement);

void radar_parser_format_hex(const uint8_t *data,
                             size_t length,
                             char *output,
                             size_t output_size);

#endif
