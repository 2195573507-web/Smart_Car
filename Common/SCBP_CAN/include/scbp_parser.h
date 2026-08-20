#ifndef SCBP_PARSER_H
#define SCBP_PARSER_H

#include <stddef.h>
#include <stdint.h>

#include "scbp_protocol_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SCBP_PARSER_WAIT_SOF0 = 0,
    SCBP_PARSER_WAIT_SOF1,
    SCBP_PARSER_READ_HEADER,
    SCBP_PARSER_VERIFY_HCS,
    SCBP_PARSER_READ_PAYLOAD_AND_TAIL,
    SCBP_PARSER_VERIFY_FCS_EOF
} scbp_parser_state_t;

typedef enum {
    SCBP_PARSER_ERROR_HCS = 1,
    SCBP_PARSER_ERROR_FCS,
    SCBP_PARSER_ERROR_EOF,
    SCBP_PARSER_ERROR_FLAGS,
    SCBP_PARSER_ERROR_NODE
} scbp_parser_error_t;

typedef struct {
    uint16_t can_id;
    uint8_t flags;
    uint8_t sequence;
    uint8_t length;
    const uint8_t *payload;
} scbp_can_frame_t;

typedef void (*scbp_parser_frame_callback_t)(const scbp_can_frame_t *frame,
                                             void *context);
typedef void (*scbp_parser_error_callback_t)(scbp_parser_error_t error,
                                             const uint8_t *data,
                                             size_t length,
                                             void *context);

typedef struct {
    scbp_parser_state_t state;
    uint8_t header_index;
    uint16_t body_index;
    uint8_t payload_length;
    uint32_t frame_count;
    _Alignas(4) uint8_t bytes[SCBP_CAN_MAX_FRAME_SIZE];
    scbp_parser_frame_callback_t frame_callback;
    scbp_parser_error_callback_t error_callback;
    void *context;
} scbp_parser_t;

void scbp_parser_init(scbp_parser_t *parser,
                      scbp_parser_frame_callback_t frame_callback,
                      scbp_parser_error_callback_t error_callback,
                      void *context);
size_t scbp_parser_feed(scbp_parser_t *parser, const uint8_t *data, size_t length);
int scbp_can_encode(const scbp_can_frame_t *frame, uint8_t *out, size_t capacity,
                    uint16_t *out_length);
int scbp_can_decode(const uint8_t *data, size_t length, scbp_can_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* SCBP_PARSER_H */
