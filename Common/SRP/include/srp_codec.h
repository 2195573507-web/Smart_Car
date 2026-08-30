#ifndef SRP_CODEC_H
#define SRP_CODEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "srp_def.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SRP_CODEC_OK = 0,
    SRP_CODEC_INVALID_ARGUMENT = 1,
    SRP_CODEC_INVALID_LENGTH,
    SRP_CODEC_INVALID_HEADER,
    SRP_CODEC_BAD_MAGIC,
    SRP_CODEC_BAD_CRC,
    SRP_CODEC_BAD_EOF,
    SRP_CODEC_OVERFLOW
} srp_codec_status_t;

typedef enum {
    SRP_PARSER_WAIT_MAGIC0 = 0,
    SRP_PARSER_WAIT_MAGIC1,
    SRP_PARSER_READ_HEADER,
    SRP_PARSER_READ_BODY
} srp_parser_state_t;

typedef enum {
    SRP_PARSER_ERROR_MAGIC = 1,
    SRP_PARSER_ERROR_LENGTH,
    SRP_PARSER_ERROR_HEADER,
    SRP_PARSER_ERROR_CRC,
    SRP_PARSER_ERROR_EOF,
    SRP_PARSER_ERROR_OVERFLOW
} srp_parser_error_t;

#pragma pack(push, 4)
typedef struct {
    srp_parser_state_t state;
    uint16_t index;
    uint16_t expected_length;
    uint32_t frame_count;
    uint32_t crc_error_count;
    uint32_t eof_error_count;
    uint32_t length_error_count;
    /* Last byte/state associated with a header or length rejection. These
     * fields are diagnostic only and are not part of the SRP wire format. */
    srp_parser_state_t last_error_state;
    uint8_t last_drop_byte;
    _Alignas(4) uint8_t bytes[SRP_MAX_FRAME_SIZE];
    void (*frame_callback)(const srp_frame_t *frame, void *context);
    void (*error_callback)(srp_parser_error_t error, const uint8_t *data,
                           size_t length, void *context);
    void *context;
} srp_parser_t;

typedef struct {
    const uint8_t *data;
    size_t length;
    size_t offset;
} srp_tlv_iter_t;
#pragma pack(pop)

int srp_encode(const srp_frame_t *frame, uint8_t *out, size_t capacity,
               uint16_t *out_length);
/* Canonical frame-encoding entry point used by all transports. */
int srp_encode_frame(const srp_frame_t *frame, uint8_t *out, size_t capacity,
                     uint16_t *out_length);
int srp_decode(const uint8_t *data, size_t length, srp_frame_t *frame);

void srp_parser_init(srp_parser_t *parser,
                     void (*frame_callback)(const srp_frame_t *frame,
                                             void *context),
                     void (*error_callback)(srp_parser_error_t error,
                                            const uint8_t *data, size_t length,
                                            void *context),
                     void *context);
/* Drop a partial frame while retaining callbacks and parser diagnostics. */
void srp_parser_reset(srp_parser_t *parser);
size_t srp_parser_feed(srp_parser_t *parser, const uint8_t *data, size_t length);

void srp_tlv_iter_init(srp_tlv_iter_t *iterator, const uint8_t *data,
                       size_t length);
bool srp_tlv_next(srp_tlv_iter_t *iterator, uint8_t *tag, uint8_t *value_length,
                  const uint8_t **value);

#ifdef __cplusplus
}
#endif

#endif /* SRP_CODEC_H */
