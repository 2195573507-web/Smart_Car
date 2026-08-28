#include "log_bridge.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "s3_ble.h"
#include "smartcar_log.h"

#define LOG_BRIDGE_PAYLOAD_HEADER_SIZE 8U
#define LOG_BRIDGE_MIN_INTERVAL_MS 50U
#define LOG_BRIDGE_MIN_INTERVAL_TICKS \
    ((TickType_t)((((uint32_t)LOG_BRIDGE_MIN_INTERVAL_MS * configTICK_RATE_HZ) + \
                  999U) / 1000U))

static const char *TAG = "UART_LOG_BRIDGE";
/* SRP LOG currently enters only from the serialized service parser. */
static uint8_t s_legacy_frame[SMARTCAR_LOG_MAX_FRAME_SIZE];
static TickType_t s_last_stm_log_tick;
static uint32_t s_suppressed_count;
static bool s_stm_log_rate_initialized;

static void log_bridge_emit_stm_log_marker(void)
{
    const TickType_t now = xTaskGetTickCount();

    /* STM32 log floods can starve the S3 service/BLE path; cap this marker at 20 Hz. */
    if (s_stm_log_rate_initialized &&
        (TickType_t)(now - s_last_stm_log_tick) < LOG_BRIDGE_MIN_INTERVAL_TICKS) {
        ++s_suppressed_count;
        return;
    }

    s_stm_log_rate_initialized = true;
    s_last_stm_log_tick = now;
    if (s_suppressed_count > 0U) {
        ESP_LOGW(TAG, "STM logs suppressed=%lu",
                 (unsigned long)s_suppressed_count);
        s_suppressed_count = 0U;
    }
    ESP_LOGI(TAG, "STM_LOG_RX");
}

void log_bridge_handle(const srp_frame_t *frame)
{
    size_t legacy_length = 0U;
    const uint8_t *payload;
    uint16_t text_length;

    if (frame == NULL || frame->type != SRP_MSG_ID_LOG ||
        frame->length < LOG_BRIDGE_PAYLOAD_HEADER_SIZE) {
        return;
    }
    if (frame->payload == NULL) {
        ESP_LOGW(TAG, "LOG_DROP missing payload frame=%u", (unsigned)frame->length);
        return;
    }
    payload = frame->payload;
    text_length = (uint16_t)payload[6] | ((uint16_t)payload[7] << 8U);
    if ((payload[0] > SMARTCAR_LOG_SOURCE_S3) ||
        (payload[1] > SMARTCAR_LOG_LEVEL_ERROR) ||
        text_length > SMARTCAR_LOG_MAX_PAYLOAD ||
        frame->length != LOG_BRIDGE_PAYLOAD_HEADER_SIZE + text_length) {
        ESP_LOGW(TAG,
                 "LOG_DROP invalid envelope source=%u level=%u text=%u frame=%u",
                 (unsigned)payload[0], (unsigned)payload[1],
                 (unsigned)text_length, (unsigned)frame->length);
        return;
    }

    if (smartcar_log_encode((smartcar_log_source_t)payload[0],
                            (smartcar_log_level_t)payload[1],
                            (uint32_t)payload[2] |
                                ((uint32_t)payload[3] << 8U) |
                                ((uint32_t)payload[4] << 16U) |
                                ((uint32_t)payload[5] << 24U),
                            &payload[LOG_BRIDGE_PAYLOAD_HEADER_SIZE],
                            (uint8_t)text_length, s_legacy_frame,
                            sizeof(s_legacy_frame), &legacy_length) != SMARTCAR_LOG_OK) {
        ESP_LOGW(TAG, "LOG_DROP encode failed");
        return;
    }

    if (payload[0] == SMARTCAR_LOG_SOURCE_STM32) {
        log_bridge_emit_stm_log_marker();
    }
    (void)s3_ble_log_notify_send(s_legacy_frame, (uint16_t)legacy_length);
}
