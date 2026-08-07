#include "smartcar_log.h"

static uint16_t smartcar_log_crc16_update(uint16_t crc, uint8_t value)
{
    crc ^= value;
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
        crc = (crc & 1U) != 0U ? (uint16_t)((crc >> 1U) ^ UINT16_C(0xA001))
                               : (uint16_t)(crc >> 1U);
    }
    return crc;
}

static void put_u32_le(uint8_t *output, uint32_t value)
{
    output[0] = (uint8_t)(value & UINT32_C(0xFF));
    output[1] = (uint8_t)((value >> 8U) & UINT32_C(0xFF));
    output[2] = (uint8_t)((value >> 16U) & UINT32_C(0xFF));
    output[3] = (uint8_t)((value >> 24U) & UINT32_C(0xFF));
}

static uint32_t get_u32_le(const uint8_t *input)
{
    return (uint32_t)input[0] |
           ((uint32_t)input[1] << 8U) |
           ((uint32_t)input[2] << 16U) |
           ((uint32_t)input[3] << 24U);
}

uint16_t smartcar_log_crc16_modbus(const uint8_t *data, size_t length)
{
    uint16_t crc = UINT16_C(0xFFFF);

    if (data == NULL && length != 0U) {
        return 0U;
    }
    for (size_t index = 0U; index < length; ++index) {
        crc = smartcar_log_crc16_update(crc, data[index]);
    }
    return crc;
}

smartcar_log_status_t smartcar_log_encode(
    smartcar_log_source_t source,
    smartcar_log_level_t level,
    uint32_t timestamp_ms,
    const uint8_t *payload,
    uint8_t payload_length,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length)
{
    const size_t frame_length = SMARTCAR_LOG_FRAME_OVERHEAD + payload_length;
    const size_t crc_offset = SMARTCAR_LOG_HEADER_SIZE + payload_length;
    uint16_t crc;

    if (output == NULL || output_length == NULL ||
        (payload == NULL && payload_length != 0U) ||
        source > SMARTCAR_LOG_SOURCE_S3 || level > SMARTCAR_LOG_LEVEL_ERROR) {
        return SMARTCAR_LOG_INVALID_ARG;
    }
    if (payload_length > SMARTCAR_LOG_MAX_PAYLOAD) {
        return SMARTCAR_LOG_PAYLOAD_TOO_LARGE;
    }
    if (output_capacity < frame_length) {
        return SMARTCAR_LOG_BUFFER_TOO_SMALL;
    }

    output[0] = SMARTCAR_LOG_HEAD_0;
    output[1] = SMARTCAR_LOG_HEAD_1;
    output[2] = SMARTCAR_LOG_VERSION;
    output[3] = (uint8_t)source;
    output[4] = (uint8_t)level;
    put_u32_le(&output[5], timestamp_ms);
    output[9] = payload_length;
    for (uint8_t index = 0U; index < payload_length; ++index) {
        output[SMARTCAR_LOG_HEADER_SIZE + index] = payload[index];
    }

    crc = smartcar_log_crc16_modbus(&output[2], crc_offset - 2U);
    output[crc_offset] = (uint8_t)(crc & UINT16_C(0xFF));
    output[crc_offset + 1U] = (uint8_t)(crc >> 8U);
    *output_length = frame_length;
    return SMARTCAR_LOG_OK;
}

smartcar_log_status_t smartcar_log_decode(
    const uint8_t *frame,
    size_t frame_length,
    smartcar_log_record_t *record)
{
    uint8_t payload_length;
    size_t expected_length;
    size_t crc_offset;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if (frame == NULL || record == NULL) {
        return SMARTCAR_LOG_INVALID_ARG;
    }
    if (frame_length < SMARTCAR_LOG_FRAME_OVERHEAD ||
        frame[0] != SMARTCAR_LOG_HEAD_0 ||
        frame[1] != SMARTCAR_LOG_HEAD_1 ||
        frame[2] != SMARTCAR_LOG_VERSION ||
        frame[3] > SMARTCAR_LOG_SOURCE_S3 ||
        frame[4] > SMARTCAR_LOG_LEVEL_ERROR) {
        return SMARTCAR_LOG_INVALID_FRAME;
    }

    payload_length = frame[9];
    if (payload_length > SMARTCAR_LOG_MAX_PAYLOAD) {
        return SMARTCAR_LOG_INVALID_FRAME;
    }
    expected_length = SMARTCAR_LOG_FRAME_OVERHEAD + payload_length;
    if (frame_length != expected_length) {
        return SMARTCAR_LOG_INVALID_FRAME;
    }

    crc_offset = SMARTCAR_LOG_HEADER_SIZE + payload_length;
    received_crc = (uint16_t)frame[crc_offset] |
                   ((uint16_t)frame[crc_offset + 1U] << 8U);
    calculated_crc = smartcar_log_crc16_modbus(&frame[2], crc_offset - 2U);
    if (received_crc != calculated_crc) {
        return SMARTCAR_LOG_CRC_MISMATCH;
    }

    record->source = (smartcar_log_source_t)frame[3];
    record->level = (smartcar_log_level_t)frame[4];
    record->timestamp_ms = get_u32_le(&frame[5]);
    record->payload = &frame[SMARTCAR_LOG_HEADER_SIZE];
    record->payload_length = payload_length;
    return SMARTCAR_LOG_OK;
}
