#ifndef SMARTCAR_LOG_H
#define SMARTCAR_LOG_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* This envelope is intentionally separate from SmartCar AA/55 control frames. */
#define SMARTCAR_LOG_HEAD_0 UINT8_C(0xAA)
#define SMARTCAR_LOG_HEAD_1 UINT8_C(0x55)
#define SMARTCAR_LOG_VERSION UINT8_C(0x01)
#define SMARTCAR_LOG_MAX_PAYLOAD UINT8_C(96)
#define SMARTCAR_LOG_HEADER_SIZE UINT8_C(10)
#define SMARTCAR_LOG_CRC_SIZE UINT8_C(2)
#define SMARTCAR_LOG_FRAME_OVERHEAD \
    (SMARTCAR_LOG_HEADER_SIZE + SMARTCAR_LOG_CRC_SIZE)
#define SMARTCAR_LOG_MAX_FRAME_SIZE \
    (SMARTCAR_LOG_FRAME_OVERHEAD + SMARTCAR_LOG_MAX_PAYLOAD)

typedef enum {
    SMARTCAR_LOG_SOURCE_STM32 = 0U,
    SMARTCAR_LOG_SOURCE_S3 = 1U
} smartcar_log_source_t;

typedef enum {
    SMARTCAR_LOG_LEVEL_DEBUG = 0U,
    SMARTCAR_LOG_LEVEL_INFO = 1U,
    SMARTCAR_LOG_LEVEL_WARN = 2U,
    SMARTCAR_LOG_LEVEL_ERROR = 3U
} smartcar_log_level_t;

typedef enum {
    SMARTCAR_LOG_OK = 0,
    SMARTCAR_LOG_INVALID_ARG,
    SMARTCAR_LOG_BUFFER_TOO_SMALL,
    SMARTCAR_LOG_PAYLOAD_TOO_LARGE,
    SMARTCAR_LOG_INVALID_FRAME,
    SMARTCAR_LOG_CRC_MISMATCH
} smartcar_log_status_t;

typedef struct {
    smartcar_log_source_t source;
    smartcar_log_level_t level;
    uint32_t timestamp_ms;
    const uint8_t *payload;
    uint8_t payload_length;
} smartcar_log_record_t;

uint16_t smartcar_log_crc16_modbus(const uint8_t *data, size_t length);

smartcar_log_status_t smartcar_log_encode(
    smartcar_log_source_t source,
    smartcar_log_level_t level,
    uint32_t timestamp_ms,
    const uint8_t *payload,
    uint8_t payload_length,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length);

smartcar_log_status_t smartcar_log_decode(
    const uint8_t *frame,
    size_t frame_length,
    smartcar_log_record_t *record);

#ifdef __cplusplus
}
#endif

#endif /* SMARTCAR_LOG_H */
