#include "smartcar_service.h"
#include "log_bridge.h"
#include "radar_calibration_manager.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "frame.h"
#include "parser.h"
#include "stm_uart.h"
#include "esp_log.h"

#define SMARTCAR_SERVICE_TASK_STACK 3072U
#define SMARTCAR_SERVICE_TASK_PRIORITY 8U
#define SMARTCAR_SERVICE_PING_LIMIT 1000U
#define SMARTCAR_SERVICE_TASK_DELAY_TICKS 1U

static sc_frame_parser_t s_parser;
static const char *TAG = "UART_VALIDATION";
static uint32_t s_ping_rx;
static uint32_t s_pong_tx;
static uint32_t s_crc_errors;
static bool s_stats_printed;

static void command_bridge_on_frame(const sc_frame_view_t *frame, void *context)
{
    (void)context;
    ESP_LOGI(TAG, "STM UART RX FRAME type=0x%02X", (unsigned)frame->type);
    if (frame->type == SC_TYPE_LOG) {
        log_bridge_handle(frame);
        return;
    }
    if (frame->type == SC_TYPE_STM_BOOT_READY ||
        frame->type == SC_TYPE_RADAR_PWM_ACK ||
        frame->type == SC_TYPE_CAL_EVENT) {
        if (frame->type == SC_TYPE_RADAR_PWM_ACK) {
            ESP_LOGI(TAG, "RADAR_PWM_ACK RX");
        }
        radar_calibration_manager_on_frame(frame->type, frame->payload,
                                            frame->length);
        return;
    }
    ESP_LOGI(TAG, "RX_FRAME: version=%u type=0x%02X length=%u crc_result=OK",
             (unsigned)frame->version, (unsigned)frame->type,
             (unsigned)frame->length);
    if (frame->type != SC_TYPE_PING) return;
    ++s_ping_rx;
    uint8_t response[SC_FRAME_MAX_SIZE];
    uint16_t response_length = 0U;
    if (sc_frame_encode(SC_TYPE_PONG, NULL, 0U, response, sizeof(response),
                        &response_length) == 0) {
        ESP_LOGI(TAG, "PONG_TX");
        (void)stm_uart_send(response, response_length);
        ++s_pong_tx;
    }
}

static void command_bridge_on_error(int error, const uint8_t *data,
                                    size_t length, void *context)
{
    (void)context;
    uint8_t version = 0U;
    uint8_t type = 0U;
    uint16_t frame_length = 0U;
    if (data != NULL && length >= 6U) {
        version = data[2]; type = data[3];
        frame_length = (uint16_t)data[4] | ((uint16_t)data[5] << 8U);
    }
    if (error == -5) ++s_crc_errors;
    ESP_LOGI(TAG, "RX_FRAME: version=%u type=0x%02X length=%u crc_result=ERROR",
             (unsigned)version, (unsigned)type, (unsigned)frame_length);
}

static void smartcar_service_task(void *context)
{
    (void)context;
    uint8_t buffer[256];
    s_ping_rx = 0U; s_pong_tx = 0U; s_crc_errors = 0U; s_stats_printed = false;
    sc_frame_parser_init(&s_parser, command_bridge_on_frame,
                         command_bridge_on_error, NULL);
    for (;;) {
        const int received = stm_uart_receive_nonblock(buffer, sizeof(buffer));
        if (received > 0) {
            (void)sc_frame_parser_feed(&s_parser, buffer, (size_t)received);
        }
        radar_calibration_manager_step();
        if (s_ping_rx >= SMARTCAR_SERVICE_PING_LIMIT && !s_stats_printed) {
            stm_uart_stats_t uart_stats = {0};
            stm_uart_get_stats(&uart_stats);
            ESP_LOGI(TAG, "UART_STATS ping_rx=%lu pong_tx=%lu lost=%lu "
                     "crc_errors=%lu loss_rate_x100=%lu rx_bytes=%lu "
                     "tx_bytes=%lu overflow=%lu drop=%lu short_write=%lu "
                     "hal_error=%lu",
                     (unsigned long)s_ping_rx, (unsigned long)s_pong_tx,
                     (unsigned long)(s_ping_rx - s_pong_tx),
                     (unsigned long)s_crc_errors,
                     (unsigned long)((s_ping_rx - s_pong_tx) * 10000U / s_ping_rx),
                     (unsigned long)uart_stats.rx_bytes,
                     (unsigned long)uart_stats.tx_bytes,
                     (unsigned long)uart_stats.overflow,
                     (unsigned long)uart_stats.drop,
                     (unsigned long)uart_stats.short_write,
                     (unsigned long)uart_stats.hal_error);
            s_stats_printed = true;
        }
        vTaskDelay(SMARTCAR_SERVICE_TASK_DELAY_TICKS);
    }
}

esp_err_t smartcar_service_init(void)
{
    radar_calibration_manager_init();
    return xTaskCreate(smartcar_service_task, "smartcar_service",
                       SMARTCAR_SERVICE_TASK_STACK, NULL,
                       SMARTCAR_SERVICE_TASK_PRIORITY, NULL) == pdPASS
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}
