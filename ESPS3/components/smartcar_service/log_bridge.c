#include "log_bridge.h"

#include <stdint.h>

#include "esp_log.h"
#include "s3_ble.h"
#include "smartcar_log.h"

#define LOG_BRIDGE_PAYLOAD_HEADER_SIZE 8U

static const char *TAG = "UART_LOG_BRIDGE";

void log_bridge_handle(const sc_frame_view_t *frame)
{
    uint8_t legacy_frame[SMARTCAR_LOG_MAX_FRAME_SIZE];
    size_t legacy_length = 0U;
    const uint8_t *payload;
    uint16_t text_length;

    if (frame == NULL || frame->type != SC_TYPE_LOG ||
        frame->length < LOG_BRIDGE_PAYLOAD_HEADER_SIZE) {
        return;
    }
    payload = frame->payload;
    text_length = (uint16_t)payload[6] | ((uint16_t)payload[7] << 8U);
    if ((payload[0] > SMARTCAR_LOG_SOURCE_S3) ||
        (payload[1] < SMARTCAR_LOG_LEVEL_INFO) ||
        (payload[1] > SMARTCAR_LOG_LEVEL_ERROR) ||
        text_length > SMARTCAR_LOG_MAX_PAYLOAD ||
        frame->length != LOG_BRIDGE_PAYLOAD_HEADER_SIZE + text_length) {
        ESP_LOGW(TAG, "LOG_DROP invalid payload length=%u", (unsigned)frame->length);
        return;
    }

    if (smartcar_log_encode((smartcar_log_source_t)payload[0],
                            (smartcar_log_level_t)payload[1],
                            (uint32_t)payload[2] |
                                ((uint32_t)payload[3] << 8U) |
                                ((uint32_t)payload[4] << 16U) |
                                ((uint32_t)payload[5] << 24U),
                            &payload[LOG_BRIDGE_PAYLOAD_HEADER_SIZE],
                            (uint8_t)text_length, legacy_frame,
                            sizeof(legacy_frame), &legacy_length) != SMARTCAR_LOG_OK) {
        ESP_LOGW(TAG, "LOG_DROP encode failed");
        return;
    }

    if (payload[0] == SMARTCAR_LOG_SOURCE_STM32) {
        ESP_LOGI(TAG, "STM_LOG_RX");
    }
    (void)s3_ble_log_notify_send(legacy_frame, (uint16_t)legacy_length);
}
