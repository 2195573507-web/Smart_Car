#include "smartcar_service.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "app_parser.h"
#include "log_bridge.h"
#include "radar_calibration_manager.h"
#include "radar_control.h"
#include "s3_ble.h"
#include "scbp_link.h"
#include "scbp_parser.h"
#include "stm_uart.h"

#define SMARTCAR_SERVICE_TASK_NAME "smartcar_svc"
#define SMARTCAR_SERVICE_TASK_STACK 4096U
#define SMARTCAR_SERVICE_TASK_PRIORITY 8U
#define SMARTCAR_SERVICE_TASK_DELAY_TICKS 1U
#define SMARTCAR_SERVICE_BLE_RX_QUEUE_DEPTH 8U
#define SMARTCAR_SERVICE_BLE_RX_BUDGET 4U
#define SMARTCAR_SERVICE_STACK_REPORT_PERIOD_MS UINT32_C(5000)
#define SMARTCAR_SERVICE_BUS_OFF_RECOVERY_MS UINT32_C(100)
#define APP_RADAR_STATUS_PERIOD_MS UINT32_C(1000)

#ifndef SMARTCAR_BMI323_DEBUG_ONLY
#define SMARTCAR_BMI323_DEBUG_ONLY 0
#endif

typedef struct {
    uint16_t length;
    uint8_t bytes[SC_APP_FRAME_MAX_SIZE];
} smartcar_ble_rx_item_t;

static const char *TAG = "SCBP_CAN";
static scbp_parser_t s_parser;
static scbp_link_t s_link;
static sc_app_parser_t s_app_parser;
static QueueHandle_t s_ble_rx_queue;
static StaticQueue_t s_ble_rx_queue_storage;
static TaskHandle_t s_task;
static uint8_t s_link_ready;
static bool s_bus_off_recovery_pending;
static uint64_t s_bus_off_recovery_at_us;
static volatile uint32_t s_ble_rx_dropped;
static volatile uint32_t s_ble_rx_protocol_errors;
static volatile UBaseType_t s_stack_min_free_bytes;
static volatile bool s_stack_hwm_valid;
static uint32_t s_parser_errors;
static uint32_t s_dual_len_reject;
static uint32_t s_dual_schema_reject;
static uint32_t s_dual_crc_reject;
static uint32_t s_dual_notify_drop;
static uint32_t s_dual_ble_not_ready;

static uint8_t s_ble_rx_queue_buffer[
    SMARTCAR_SERVICE_BLE_RX_QUEUE_DEPTH * sizeof(smartcar_ble_rx_item_t)]
    __attribute__((aligned(4)));
static uint8_t s_uart_rx_buffer[256U];
static smartcar_ble_rx_item_t s_ble_rx_item;
static uint8_t s_app_tx_frame[SC_APP_FRAME_MAX_SIZE];

static uint64_t now_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

static bool notify_app_frame(uint8_t type, const uint8_t *payload, uint16_t length)
{
    uint16_t frame_length = 0U;

    if (sc_app_frame_encode(type, payload, length, s_app_tx_frame,
                            sizeof(s_app_tx_frame), &frame_length) != 0) {
        return false;
    }
    return s3_ble_notify_send(s_app_tx_frame, frame_length) == ESP_OK;
}

static int command_bridge_transport_send(const uint8_t *data, uint16_t length,
                                         void *context)
{
    (void)context;
    return stm_uart_send(data, length) == (int)length ? 0 : -1;
}

static void command_bridge_bus_off(void *context)
{
    (void)context;
    stm_uart_recover();
    s_bus_off_recovery_pending = true;
    s_bus_off_recovery_at_us = now_us() +
        ((uint64_t)SMARTCAR_SERVICE_BUS_OFF_RECOVERY_MS * UINT64_C(1000));
    ESP_LOGE(TAG, "BUS_OFF: UART2 receive queue flushed");
}

static void command_bridge_send_response(const scbp_can_frame_t *request,
                                         uint8_t is_error, uint8_t status_code)
{
    const uint8_t source = request == NULL ? 0U : SCBP_CAN_ID_SOURCE(request->can_id);

    if (request == NULL || source == SCBP_NODE_BROADCAST) {
        return;
    }
    (void)scbp_link_send_fast_response(&s_link, SCBP_CAN_PRIORITY_REALTIME,
                                       source, is_error, request->can_id,
                                       request->sequence, status_code,
                                       (uint32_t)(now_us() / UINT64_C(1000)));
}

static bool relay_dual_attitude(const scbp_can_frame_t *frame)
{
    if (frame == NULL || frame->payload == NULL ||
        frame->length != SCBP_PAYLOAD_DUAL_AHRS_SIZE) {
        ++s_dual_len_reject;
        return false;
    }
    if (frame->payload[0] != SCBP_DUAL_AHRS_SCHEMA || frame->payload[2] != 0U ||
        frame->payload[3] != 0U) {
        ++s_dual_schema_reject;
        return false;
    }
    if (!s3_ble_is_ready()) {
        ++s_dual_ble_not_ready;
    }
    if (!notify_app_frame(SC_APP_TYPE_ATTITUDE, frame->payload, frame->length)) {
        ++s_dual_notify_drop;
        return false;
    }
    return true;
}

static void relay_telemetry(const scbp_can_frame_t *frame, uint16_t message_id)
{
#if SMARTCAR_BMI323_DEBUG_ONLY
    (void)frame;
    (void)message_id;
#else
    uint8_t app_type;

    if (frame == NULL || frame->payload == NULL) {
        return;
    }
    switch (message_id) {
    case SCBP_MSG_ID_ATTITUDE:
        (void)relay_dual_attitude(frame);
        return;
    case SCBP_MSG_ID_IMU_CAL_STATUS:
        if (frame->length != SCBP_PAYLOAD_IMU_CAL_STATUS_SIZE) {
            return;
        }
        app_type = UINT8_C(0x12);
        break;
    case SCBP_MSG_ID_IMU_TELEMETRY:
        if (frame->length != SCBP_PAYLOAD_IMU_TELEMETRY_SIZE ||
            (frame->payload[0] != SCBP_IMU_SENSOR_LSM303 &&
             frame->payload[0] != SCBP_IMU_SENSOR_BMI323)) {
            return;
        }
        app_type = UINT8_C(0x27);
        break;
    case SCBP_MSG_ID_RADAR_STATUS:
        if (frame->length != SCBP_PAYLOAD_RADAR_STATUS_SIZE) {
            return;
        }
        app_type = UINT8_C(0x15);
        break;
    default:
        return;
    }
    (void)notify_app_frame(app_type, frame->payload, frame->length);
#endif
}

static void command_bridge_ready_response(scbp_link_tx_result_t result,
                                          uint8_t status_code, void *context)
{
    (void)context;
    radar_calibration_manager_on_ready_response(result, status_code);
}

static int command_bridge_send_radar_pwm_ready(uint8_t speed_percent, void *context)
{
    const uint8_t payload[SCBP_PAYLOAD_RADAR_PWM_READY_SIZE] = {speed_percent};

    (void)context;
    return scbp_link_send(&s_link, SCBP_CAN_PRIORITY_REALTIME,
                          SCBP_NODE_STM32H757, SCBP_MSG_ID_RADAR_PWM_READY,
                          SCBP_CAN_FLAG_ACK_REQUIRED, payload, sizeof(payload),
                          (uint32_t)(now_us() / UINT64_C(1000)),
                          command_bridge_ready_response, NULL);
}

static void command_bridge_on_frame(const scbp_can_frame_t *frame, void *context)
{
    const uint16_t message_id = frame == NULL ? 0U : SCBP_CAN_ID_MESSAGE(frame->can_id);
    const uint8_t source = frame == NULL ? 0U : SCBP_CAN_ID_SOURCE(frame->can_id);
    const uint8_t destination = frame == NULL ? 0U : SCBP_CAN_ID_DESTINATION(frame->can_id);
    const bool ack_required = frame != NULL &&
        (frame->flags & SCBP_CAN_FLAG_ACK_REQUIRED) != 0U;
    bool admitted = false;

    (void)context;
    if (frame == NULL || source != SCBP_NODE_STM32H757 ||
        (destination != SCBP_NODE_ESP32_S3 && destination != SCBP_NODE_BROADCAST)) {
        return;
    }

    switch (message_id) {
    case SCBP_MSG_ID_ATTITUDE:
    case SCBP_MSG_ID_IMU_CAL_STATUS:
    case SCBP_MSG_ID_IMU_TELEMETRY:
    case SCBP_MSG_ID_RADAR_STATUS:
        relay_telemetry(frame, message_id);
        return;
    case SCBP_MSG_ID_LOG:
        log_bridge_handle(frame);
        return;
    case SCBP_MSG_ID_BOOT_READY:
#if SMARTCAR_BMI323_DEBUG_ONLY
        admitted = false;
#else
        admitted = radar_calibration_manager_on_boot_ready(frame->payload, frame->length);
#endif
        break;
    case SCBP_MSG_ID_CAL_EVENT:
#if SMARTCAR_BMI323_DEBUG_ONLY
        admitted = false;
#else
        admitted = radar_calibration_manager_on_cal_event(frame->payload, frame->length);
#endif
        break;
    default:
        if (ack_required) {
            command_bridge_send_response(frame, 1U, SCBP_FAST_RESP_INVALID_PARAM);
        }
        return;
    }

    if (ack_required) {
        command_bridge_send_response(frame, admitted ? 0U : 1U,
                                     admitted ? SCBP_FAST_RESP_OK : SCBP_FAST_RESP_BUSY);
    }
}

static void command_bridge_parsed_frame(const scbp_can_frame_t *frame, void *context)
{
    (void)context;
    scbp_link_receive(&s_link, frame);
}

static void command_bridge_on_parser_error(scbp_parser_error_t error,
                                           const uint8_t *data, size_t length,
                                           void *context)
{
    uint16_t can_id = 0U;

    (void)context;
    ++s_parser_errors;
    if (error == SCBP_PARSER_ERROR_FCS && data != NULL && length >= SCBP_CAN_HEADER_SIZE) {
        can_id = (uint16_t)data[2] | ((uint16_t)data[3] << 8U);
        if (SCBP_CAN_ID_MESSAGE(can_id) == SCBP_MSG_ID_ATTITUDE) {
            ++s_dual_crc_reject;
        }
    }
    scbp_link_report_parser_error(&s_link, error);
    ESP_LOGW(TAG, "RX parser error=%u bytes=%u rec=%u tec=%u",
             (unsigned)error, (unsigned)length,
             (unsigned)scbp_link_get_rec(&s_link),
             (unsigned)scbp_link_get_tec(&s_link));
}

static void service_ble_rx_enqueue(const uint8_t *data, size_t length, void *context)
{
    smartcar_ble_rx_item_t item;

    (void)context;
    if (s_ble_rx_queue == NULL || data == NULL || length == 0U ||
        length > sizeof(item.bytes)) {
        ++s_ble_rx_dropped;
        return;
    }
    item.length = (uint16_t)length;
    (void)memcpy(item.bytes, data, length);
    if (xQueueSend(s_ble_rx_queue, &item, 0U) != pdPASS) {
        ++s_ble_rx_dropped;
    }
}

static void send_app_ack(uint8_t acknowledged_type, uint8_t result)
{
    const uint8_t payload[2] = {acknowledged_type, result};
    (void)notify_app_frame(SC_APP_TYPE_ACK, payload, sizeof(payload));
}

static void app_command_on_frame(const sc_app_frame_view_t *frame, void *context)
{
    uint8_t result = SC_APP_ACK_REJECTED;

    (void)context;
    if (frame == NULL) {
        return;
    }
#if !SMARTCAR_BMI323_DEBUG_ONLY
    if (frame->type == SC_APP_TYPE_RADAR_SET_SPEED && frame->length == 1U &&
        frame->payload != NULL && frame->payload[0] <= RADAR_MAX_SPEED &&
        radar_control_set_speed(frame->payload[0])) {
        result = SC_APP_ACK_OK;
    }
#endif
    send_app_ack(frame->type, result);
}

static void app_command_on_error(int error, const uint8_t *data,
                                 size_t length, void *context)
{
    (void)data;
    (void)length;
    (void)context;
    ++s_ble_rx_protocol_errors;
    ESP_LOGW(TAG, "App BLE envelope error=%d", error);
}

static void notify_radar_status(void)
{
    uint8_t payload[SCBP_PAYLOAD_RADAR_STATUS_SIZE];

#if !SMARTCAR_BMI323_DEBUG_ONLY
    if (!radar_control_is_running()) {
        return;
    }
    payload[0] = 1U;
    payload[1] = radar_control_get_speed();
    (void)scbp_link_send(&s_link, SCBP_CAN_PRIORITY_NORMAL,
                          SCBP_NODE_STM32H757, SCBP_MSG_ID_RADAR_STATUS,
                          SCBP_CAN_FLAG_STREAM_DATA, payload, sizeof(payload),
                          (uint32_t)(now_us() / UINT64_C(1000)), NULL, NULL);
    (void)notify_app_frame(UINT8_C(0x15), payload, sizeof(payload));
#else
    (void)payload;
#endif
}

static void smartcar_service_task(void *context)
{
    TickType_t last_radar_status = xTaskGetTickCount();
    TickType_t last_stack_report = last_radar_status;

    (void)context;
    for (;;) {
        for (uint8_t budget = 0U; budget < SMARTCAR_SERVICE_BLE_RX_BUDGET;
             ++budget) {
            if (s_ble_rx_queue == NULL ||
                xQueueReceive(s_ble_rx_queue, &s_ble_rx_item, 0U) != pdPASS) {
                break;
            }
            (void)sc_app_parser_feed(&s_app_parser, s_ble_rx_item.bytes,
                                     s_ble_rx_item.length);
        }

        {
            const int received = stm_uart_receive_nonblock(s_uart_rx_buffer,
                                                            sizeof(s_uart_rx_buffer));
            if (received > 0) {
                (void)scbp_parser_feed(&s_parser, s_uart_rx_buffer, (size_t)received);
            }
        }
        scbp_link_tick(&s_link, (uint32_t)(now_us() / UINT64_C(1000)));
        if (s_bus_off_recovery_pending && now_us() >= s_bus_off_recovery_at_us) {
            scbp_link_recover(&s_link);
            s_bus_off_recovery_pending = false;
            ESP_LOGI(TAG, "SCBP-CAN link recovered");
        }

#if !SMARTCAR_BMI323_DEBUG_ONLY
        radar_calibration_manager_step();
        if ((xTaskGetTickCount() - last_radar_status) >=
            pdMS_TO_TICKS(APP_RADAR_STATUS_PERIOD_MS)) {
            last_radar_status = xTaskGetTickCount();
            notify_radar_status();
        }
#endif
        if ((xTaskGetTickCount() - last_stack_report) >=
            pdMS_TO_TICKS(SMARTCAR_SERVICE_STACK_REPORT_PERIOD_MS)) {
            last_stack_report = xTaskGetTickCount();
            s_stack_min_free_bytes = uxTaskGetStackHighWaterMark(NULL);
            s_stack_hwm_valid = true;
            ESP_LOGI(TAG, "STACK_HWM task=%s min_free_words=%u errors=%lu rec=%u tec=%u",
                     SMARTCAR_SERVICE_TASK_NAME, (unsigned)s_stack_min_free_bytes,
                     (unsigned long)s_parser_errors,
                     (unsigned)scbp_link_get_rec(&s_link),
                     (unsigned)scbp_link_get_tec(&s_link));
        }
        vTaskDelay(SMARTCAR_SERVICE_TASK_DELAY_TICKS);
    }
}

esp_err_t smartcar_service_init(void)
{
    const scbp_link_config_t link_config = {
        .local_node = SCBP_NODE_ESP32_S3,
        .ack_timeout_ms = SCBP_LINK_ACK_TIMEOUT_MS,
        .max_retries = SCBP_LINK_MAX_RETRIES,
        .transport_send = command_bridge_transport_send,
        .on_frame = command_bridge_on_frame,
        .on_bus_off = command_bridge_bus_off,
        .context = NULL,
    };

    s_ble_rx_dropped = 0U;
    s_ble_rx_protocol_errors = 0U;
    s_parser_errors = 0U;
    s_dual_len_reject = 0U;
    s_dual_schema_reject = 0U;
    s_dual_crc_reject = 0U;
    s_dual_notify_drop = 0U;
    s_dual_ble_not_ready = 0U;
    s_stack_hwm_valid = false;
    s_bus_off_recovery_pending = false;
    s_ble_rx_queue = xQueueCreateStatic(SMARTCAR_SERVICE_BLE_RX_QUEUE_DEPTH,
                                        sizeof(smartcar_ble_rx_item_t),
                                        s_ble_rx_queue_buffer,
                                        &s_ble_rx_queue_storage);
    if (s_ble_rx_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    scbp_link_init(&s_link, &link_config);
    scbp_parser_init(&s_parser, command_bridge_parsed_frame,
                     command_bridge_on_parser_error, NULL);
    sc_app_parser_init(&s_app_parser, app_command_on_frame, app_command_on_error, NULL);
    s_link_ready = 1U;
#if !SMARTCAR_BMI323_DEBUG_ONLY
    radar_calibration_manager_init();
    radar_calibration_manager_set_transport(command_bridge_send_radar_pwm_ready, NULL);
#endif
    if (xTaskCreate(smartcar_service_task, SMARTCAR_SERVICE_TASK_NAME,
                    SMARTCAR_SERVICE_TASK_STACK, NULL,
                    SMARTCAR_SERVICE_TASK_PRIORITY, &s_task) != pdPASS) {
        s_ble_rx_queue = NULL;
        s_link_ready = 0U;
        return ESP_ERR_NO_MEM;
    }
    return s3_ble_register_rx_callback(service_ble_rx_enqueue, NULL);
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)s_link_ready;
    ESP_EARLY_LOGE(TAG, "STACK_OVERFLOW task=%s min_free_words=%u",
                   task == s_task && task_name != NULL ? task_name : "<unknown>",
                   (unsigned)s_stack_min_free_bytes);
    abort();
}
