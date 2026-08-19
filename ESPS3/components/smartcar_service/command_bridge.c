#include "smartcar_service.h"
#include "log_bridge.h"
#include "radar_calibration_manager.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "app_parser.h"
#include "frame.h"
#include "parser.h"
#include "radar_control.h"
#include "s3_ble.h"
#include "stm_uart.h"
#include "esp_log.h"

#define SMARTCAR_SERVICE_TASK_NAME "smartcar_svc"
#define SMARTCAR_SERVICE_TASK_STACK 4096U
#define SMARTCAR_SERVICE_TASK_PRIORITY 8U
#define SMARTCAR_SERVICE_PING_LIMIT 1000U
#define SMARTCAR_SERVICE_TASK_DELAY_TICKS 1U
#define SMARTCAR_SERVICE_BLE_RX_QUEUE_DEPTH 8U
#define SMARTCAR_SERVICE_BLE_RX_BUDGET 4U
#define SMARTCAR_SERVICE_STACK_REPORT_PERIOD_MS UINT32_C(5000)

#ifndef SMARTCAR_BMI323_DEBUG_ONLY
#define SMARTCAR_BMI323_DEBUG_ONLY 0
#endif

static sc_frame_parser_t s_parser;
static sc_app_parser_t s_app_parser;
static QueueHandle_t s_ble_rx_queue;
static StaticQueue_t s_ble_rx_queue_storage;
static TaskHandle_t s_task;
static const char *TAG = "UART_VALIDATION";
static uint32_t s_ping_rx;
static uint32_t s_pong_tx;
static uint32_t s_crc_errors;
static bool s_stats_printed;
static volatile uint32_t s_ble_rx_dropped;
static volatile uint32_t s_ble_rx_protocol_errors;
static volatile UBaseType_t s_stack_min_free_bytes;
static volatile bool s_stack_hwm_valid;
static uint32_t s_dual_len_reject;
static uint32_t s_dual_schema_reject;
static uint32_t s_dual_crc_reject;
static uint32_t s_dual_seq_gap;
static uint32_t s_dual_duplicate;
static uint32_t s_dual_notify_drop;
static uint32_t s_dual_ble_not_ready;

typedef struct {
    uint16_t length;
    uint8_t bytes[SC_APP_FRAME_MAX_SIZE];
} smartcar_ble_rx_item_t;

static uint8_t s_ble_rx_queue_buffer[
    SMARTCAR_SERVICE_BLE_RX_QUEUE_DEPTH * sizeof(smartcar_ble_rx_item_t)]
    __attribute__((aligned(4)));

/* These buffers are used only by smartcar_service and its synchronous parser
 * callbacks, so moving them out of the task frame does not add shared access. */
static uint8_t s_uart_rx_buffer[256U];
static smartcar_ble_rx_item_t s_ble_rx_item;
static uint8_t s_app_tx_frame[SC_APP_FRAME_MAX_SIZE];
static uint8_t s_pong_frame[SC_FRAME_MAX_SIZE];
static uint8_t s_stm_radar_status_frame[SC_FRAME_MAX_SIZE];
static uint8_t s_error_frame[SC_FRAME_MAX_SIZE];

#define APP_RADAR_STATUS_PERIOD_MS UINT32_C(1000)

/* The STM-S3 transport frame has an AA55 prefix and no tail byte. The App
 * BLE frame has a single AA prefix and a trailing 55. Keep this conversion at
 * the gateway boundary so both existing transports remain unchanged. */
static bool notify_app_frame(uint8_t type, const uint8_t *payload,
                             uint16_t length)
{
    uint16_t frame_length = 0U;

    if (sc_app_frame_encode(type, payload, length, s_app_tx_frame,
                            sizeof(s_app_tx_frame),
                            &frame_length) != 0) {
        return false;
    }
    if (s3_ble_notify_send(s_app_tx_frame, frame_length) != ESP_OK) {
        return false;
    }
    return true;
}

static bool relay_dual_attitude(const sc_frame_view_t *frame)
{
    if (frame == NULL || frame->payload == NULL ||
        frame->length != SC_DUAL_ATTITUDE_PAYLOAD_LENGTH) {
        ++s_dual_len_reject;
        return false;
    }
    if (frame->payload[0] != SC_DUAL_ATTITUDE_SCHEMA ||
        frame->payload[2] != 0U || frame->payload[3] != 0U) {
        ++s_dual_schema_reject;
        return false;
    }
    if (!s3_ble_is_ready()) {
        ++s_dual_ble_not_ready;
    }
    if (!notify_app_frame(SC_APP_TYPE_ATTITUDE, frame->payload,
                          frame->length)) {
        ++s_dual_notify_drop;
        return false;
    }
    return true;
}

static void send_protocol_error(const scbp_frame_t *request, uint8_t error_code)
{
    uint8_t payload[SCBP_ERROR_PAYLOAD_LENGTH];
    uint16_t frame_length = 0U;
    const scbp_frame_t response = {
        .version = SC_FRAME_VERSION,
        .priority = SCBP_PRIORITY_DEBUG,
        .src = SCBP_LOCAL_NODE_ID,
        .dst = request != NULL ? request->src : SCBP_DEFAULT_DESTINATION,
        .msg_id = SCBP_MSG_ID_ERROR,
        .seq = scbp_next_tx_sequence(),
        .flags = SCBP_FLAG_ERROR_FRAME,
        .length = SCBP_ERROR_PAYLOAD_LENGTH,
        .payload = payload,
        .crc = 0U,
        .sequence_status = SCBP_SEQUENCE_FIRST,
    };

    if (request == NULL || request->src == SCBP_NODE_BROADCAST ||
        request->msg_id == SCBP_MSG_ID_ERROR) {
        return;
    }
    payload[0] = SCBP_LOCAL_NODE_ID;
    payload[1] = error_code;
    payload[2] = (uint8_t)(request->msg_id & UINT16_C(0x00FF));
    payload[3] = (uint8_t)(request->msg_id >> 8U);
    payload[4] = request->seq;
    if (scbp_frame_encode(&response, s_error_frame, sizeof(s_error_frame),
                          &frame_length) == 0) {
        (void)stm_uart_send(s_error_frame, frame_length);
    }
}

/* CAL_EVENT is a request/response transaction. ACK it at the transport
 * boundary before calibration admission so STM32 stops retrying even when
 * the local calibration state has recovered or is between transitions. */
static bool send_cal_event_transport_ack(const sc_frame_view_t *request)
{
    uint8_t payload[SCBP_ACK_PAYLOAD_LENGTH];
    uint8_t frame_bytes[SC_FRAME_MAX_SIZE];
    uint16_t frame_length = 0U;
    const scbp_frame_t response = {
        .version = SC_FRAME_VERSION,
        .priority = SCBP_PRIORITY_REALTIME,
        .src = SCBP_LOCAL_NODE_ID,
        .dst = request != NULL ? request->src : SCBP_DEFAULT_DESTINATION,
        .msg_id = SCBP_MSG_ID_ACK,
        .seq = scbp_next_tx_sequence(),
        .flags = SCBP_FLAG_ACK_FRAME,
        .length = SCBP_ACK_PAYLOAD_LENGTH,
        .payload = payload,
        .crc = 0U,
        .sequence_status = SCBP_SEQUENCE_FIRST,
    };
    int sent;

    if (request == NULL || request->src == SCBP_NODE_BROADCAST) {
        return false;
    }
    payload[0] = (uint8_t)(request->msg_id & UINT16_C(0x00FF));
    payload[1] = (uint8_t)(request->msg_id >> 8U);
    payload[2] = request->seq;
    payload[3] = SCBP_ACK_RESULT_OK;
    payload[4] = SCBP_ERROR_OK;
    if (scbp_frame_encode(&response, frame_bytes, sizeof(frame_bytes),
                          &frame_length) != 0) {
        ESP_LOGE(TAG, "CAL_EVENT_ACK encode failed seq=%u",
                 (unsigned)request->seq);
        return false;
    }
    sent = stm_uart_send(frame_bytes, frame_length);
    ESP_LOGI(TAG, "CAL_EVENT_ACK_TX msg=0x%04X rx_seq=%u result=%u sent=%d",
             (unsigned)request->msg_id, (unsigned)request->seq,
             (unsigned)SCBP_ACK_RESULT_OK, sent);
    return sent == (int)frame_length;
}

static void service_ble_rx_enqueue(const uint8_t *data, size_t length,
                                   void *context)
{
    (void)context;
    smartcar_ble_rx_item_t item;

    /* This callback runs in the Bluedroid GATT event context. It only copies
     * the ephemeral write buffer and performs a zero-timeout queue handoff. */
    if (s_ble_rx_queue == NULL || data == NULL || length == 0U ||
        length > sizeof(item.bytes)) {
        ++s_ble_rx_dropped;
        return;
    }
    item.length = (uint16_t)length;
    memcpy(item.bytes, data, length);
    if (xQueueSend(s_ble_rx_queue, &item, 0U) != pdPASS) {
        ++s_ble_rx_dropped;
    }
}

static void send_app_ack(uint8_t acknowledged_type, uint8_t result)
{
    const uint8_t payload[2] = {acknowledged_type, result};
    (void)notify_app_frame(SC_APP_TYPE_ACK, payload, sizeof(payload));
}

static void app_command_on_frame(const sc_app_frame_view_t *frame,
                                 void *context)
{
    (void)context;
    if (frame == NULL) {
        return;
    }

#if SMARTCAR_BMI323_DEBUG_ONLY
    ESP_LOGW(TAG, "BMI323 debug mode rejected App command type=0x%02X",
             (unsigned)frame->type);
    return;
#else
    uint8_t result = SC_APP_ACK_REJECTED;
    if (frame->type == SC_APP_TYPE_RADAR_SET_SPEED &&
        frame->length == 1U && frame->payload != NULL &&
        frame->payload[0] <= RADAR_MAX_SPEED &&
        radar_control_set_speed(frame->payload[0])) {
        result = SC_APP_ACK_OK;
        ESP_LOGI(TAG, "BLE RADAR_SET_SPEED speed=%u accepted",
                 (unsigned)frame->payload[0]);
    } else {
        ESP_LOGW(TAG, "BLE command rejected type=0x%02X length=%u state=%u",
                 (unsigned)frame->type, (unsigned)frame->length,
                 (unsigned)radar_control_get_state());
    }
    send_app_ack(frame->type, result);
#endif
}

static void app_command_on_error(int error, const uint8_t *data,
                                 size_t length, void *context)
{
    (void)data;
    (void)context;
    ++s_ble_rx_protocol_errors;
    if ((s_ble_rx_protocol_errors & 0x1FU) == 0U) {
        ESP_LOGW(TAG, "BLE APP frame rejected error=%d length=%u count=%lu",
                 error, (unsigned)length,
                 (unsigned long)s_ble_rx_protocol_errors);
    }
}

static void relay_telemetry(const sc_frame_view_t *frame)
{
#if SMARTCAR_BMI323_DEBUG_ONLY
    (void)frame;
    return;
#else
    uint8_t app_type;

    if (frame == NULL) {
        return;
    }
    switch (frame->msg_id) {
    case SCBP_MSG_ID_IMU_STATUS:
        if (frame->length != 38U && frame->length != 43U) return;
        app_type = 0x10U;
        break;
    case SCBP_MSG_ID_IMU_BIAS:
        if (frame->length != 12U) return;
        app_type = 0x13U;
        ESP_LOGI(TAG, "[CAL_BIAS_RELAY] type=0x13 len=12");
        break;
    case SCBP_MSG_ID_ATTITUDE:
        if (frame->length == SC_DUAL_ATTITUDE_PAYLOAD_LENGTH) {
            (void)relay_dual_attitude(frame);
            return;
        }
        if (frame->length != SC_ATTITUDE_PAYLOAD_LENGTH) {
            ++s_dual_len_reject;
            return;
        }
        app_type = SC_APP_TYPE_ATTITUDE;
        break;
    case SCBP_MSG_ID_IMU_CAL_STATUS:
        if (frame->length != 11U) return;
        app_type = 0x12U;
        break;
    case SCBP_MSG_ID_RADAR_STATUS:
        if (frame->length != 2U) return;
        app_type = 0x15U;
        break;
    case SCBP_MSG_ID_VIBRATION_STATUS:
        if (frame->length != 17U) return;
        app_type = 0x18U;
        break;
    case SCBP_MSG_ID_IMU_CAL_RESULT:
        if (frame->length != 14U && frame->length != 26U) return;
        app_type = 0x25U;
        break;
    case SCBP_MSG_ID_IMU_VIBRATION_PROFILE:
        if (frame->length != 26U && frame->length != 42U) return;
        app_type = 0x26U;
        break;
    case SCBP_MSG_ID_IMU_TELEMETRY:
        if (frame->length != 30U) return;
        app_type = 0x27U;
        /* Preserve payload byte 1 (including the BMI online bit) verbatim;
         * this bridge only rebuilds the App BLE envelope. */
        break;
    case SCBP_MSG_ID_DUAL_IMU_STATUS:
        if (frame->length != SC_DUAL_IMU_STATUS_PAYLOAD_LENGTH) return;
        app_type = SC_TYPE_DUAL_IMU_STATUS;
        break;
    default:
        return;
    }
    if (!notify_app_frame(app_type, frame->payload, frame->length) &&
        frame->msg_id == SCBP_MSG_ID_ATTITUDE &&
        frame->length == SC_DUAL_ATTITUDE_PAYLOAD_LENGTH) {
        ++s_dual_notify_drop;
    }
#endif
}

static void notify_radar_status(void)
{
    uint8_t payload[2];
    uint8_t calibration[2];

    /* No status frame is emitted before the S3 radar control reaches RUNNING;
     * the App consequently stays in WAITING instead of mislabeling startup
     * as a sensor error. */
    if (radar_control_is_running()) {
        payload[0] = 1U;
        payload[1] = radar_control_get_speed();
        uint16_t frame_length = 0U;
        if (sc_frame_encode(SC_TYPE_RADAR_STATUS, payload,
                            (uint16_t)sizeof(payload),
                            s_stm_radar_status_frame,
                            sizeof(s_stm_radar_status_frame),
                            &frame_length) == 0) {
            (void)stm_uart_send(s_stm_radar_status_frame, frame_length);
        }
        (void)notify_app_frame(0x15U, payload, (uint16_t)sizeof(payload));
    }
    calibration[0] = radar_control_get_calibration_pwm();
    calibration[1] = radar_control_is_calibration_active() ? 1U : 0U;
    (void)notify_app_frame(0x18U, calibration,
                           (uint16_t)sizeof(calibration));
}

static void command_bridge_on_frame(const sc_frame_view_t *frame, void *context)
{
    (void)context;
    if (frame == NULL || (frame->dst != SCBP_LOCAL_NODE_ID &&
                          frame->dst != SCBP_NODE_BROADCAST)) {
        return;
    }
    if (frame->sequence_status == SCBP_SEQUENCE_GAP ||
        frame->sequence_status == SCBP_SEQUENCE_DUPLICATE ||
        frame->sequence_status == SCBP_SEQUENCE_OUT_OF_ORDER) {
        ESP_LOGW(TAG, "SCBP_SEQ src=0x%02X seq=%u state=%u",
                 (unsigned)frame->src, (unsigned)frame->seq,
                 (unsigned)frame->sequence_status);
        if (frame->msg_id == SCBP_MSG_ID_ATTITUDE &&
            frame->length == SC_DUAL_ATTITUDE_PAYLOAD_LENGTH) {
            if (frame->sequence_status == SCBP_SEQUENCE_GAP) {
                ++s_dual_seq_gap;
            } else if (frame->sequence_status == SCBP_SEQUENCE_DUPLICATE) {
                ++s_dual_duplicate;
            }
        }
    }
    relay_telemetry(frame);
    if (frame->msg_id == SCBP_MSG_ID_IMU_STATUS ||
        frame->msg_id == SCBP_MSG_ID_IMU_BIAS ||
        frame->msg_id == SCBP_MSG_ID_ATTITUDE ||
        frame->msg_id == SCBP_MSG_ID_IMU_CAL_STATUS ||
        frame->msg_id == SCBP_MSG_ID_RADAR_STATUS ||
        frame->msg_id == SCBP_MSG_ID_IMU_CAL_RESULT ||
        frame->msg_id == SCBP_MSG_ID_IMU_VIBRATION_PROFILE ||
        frame->msg_id == SCBP_MSG_ID_IMU_TELEMETRY ||
        frame->msg_id == SCBP_MSG_ID_DUAL_IMU_STATUS ||
        frame->msg_id == SCBP_MSG_ID_VIBRATION_STATUS) {
        return;
    }
    ESP_LOGI(TAG, "SCBP_RX msg=0x%04X src=0x%02X dst=0x%02X seq=%u len=%u",
             (unsigned)frame->msg_id, (unsigned)frame->src,
             (unsigned)frame->dst, (unsigned)frame->seq,
             (unsigned)frame->length);
    if (frame->msg_id == SCBP_MSG_ID_LOG) {
        log_bridge_handle(frame);
        return;
    }
    if (frame->msg_id == SCBP_MSG_ID_CAL_EVENT) {
        if (frame->length != 1U) {
            send_protocol_error(frame, SCBP_ERROR_INVALID_LENGTH);
            return;
        }
        /* This is intentionally before any radar state check. The manager
         * remains responsible for the id-specific hardware transition. */
        (void)send_cal_event_transport_ack(frame);
        radar_calibration_manager_on_frame(SC_TYPE_CAL_EVENT, frame->payload,
                                           frame->length);
        return;
    }
#if SMARTCAR_BMI323_DEBUG_ONLY
    return;
#endif
    if (frame->msg_id == SCBP_MSG_ID_BOOT_READY) {
        ESP_LOGI(TAG, "BOOT_READY_ROUTE src=%u dst=%u seq=%u len=%u",
                 (unsigned)frame->src, (unsigned)frame->dst,
                 (unsigned)frame->seq, (unsigned)frame->length);
        if (frame->length != 2U) {
            send_protocol_error(frame, SCBP_ERROR_INVALID_LENGTH);
            return;
        }
        ESP_LOGI(TAG, "BOOT_READY_PAYLOAD state=%u result=%u",
                 (unsigned)frame->payload[0], (unsigned)frame->payload[1]);
        radar_calibration_manager_on_frame(SC_TYPE_STM_BOOT_READY,
                                           frame->payload, frame->length);
        return;
    }
    if (frame->msg_id == SCBP_MSG_ID_ACK) {
        uint8_t legacy_type;
        uint8_t legacy_payload0;
        uint8_t result;
        uint8_t legacy_ack_payload[2];

        if (frame->length != SCBP_ACK_PAYLOAD_LENGTH) {
            send_protocol_error(frame, SCBP_ERROR_INVALID_LENGTH);
            return;
        }
        if (scbp_pending_tx_match_ack(frame, &legacy_type, &legacy_payload0,
                                      &result) != 0) {
            if (legacy_type == SC_TYPE_RADAR_PWM_READY) {
                legacy_ack_payload[0] = legacy_payload0;
                legacy_ack_payload[1] =
                    result == SCBP_ACK_RESULT_OK ? CAL_ACK_OK : CAL_ACK_ERROR;
                radar_calibration_manager_on_frame(
                    SC_TYPE_RADAR_PWM_ACK, legacy_ack_payload,
                    (uint16_t)sizeof(legacy_ack_payload));
            } else if (legacy_type == SC_TYPE_CAL_EVENT &&
                       legacy_payload0 == SC_CAL_EVENT_COMPLETE) {
                ESP_LOGI(TAG, "CAL_EVENT_COMPLETE_ACK result=%u",
                         (unsigned)result);
            } else {
                ESP_LOGW(TAG, "SCBP_ACK matched unexpected type=0x%02X",
                         (unsigned)legacy_type);
            }
        } else {
            ESP_LOGW(TAG, "SCBP_ACK rejected; pending transaction unchanged");
        }
        return;
    }
    if (frame->msg_id == SCBP_MSG_ID_ERROR) {
        return;
    }
    if (frame->msg_id == SCBP_MSG_ID_PING) {
        uint16_t response_length = 0U;

        ++s_ping_rx;
        if (sc_frame_encode(SC_TYPE_PONG, NULL, 0U, s_pong_frame,
                            sizeof(s_pong_frame),
                            &response_length) == 0) {
            ESP_LOGI(TAG, "PONG_TX");
            (void)stm_uart_send(s_pong_frame, response_length);
            ++s_pong_tx;
        }
        return;
    }
    send_protocol_error(frame, SCBP_ERROR_UNKNOWN_MSG);
}

static void command_bridge_on_error(int error, const uint8_t *data,
                                    size_t length, void *context)
{
    (void)context;
    uint8_t version = 0U;
    uint16_t msg_id = 0U;
    uint16_t frame_length = 0U;
    if (data != NULL && length >= 12U) {
        version = data[2];
        msg_id = (uint16_t)data[6] | ((uint16_t)data[7] << 8U);
        frame_length = (uint16_t)data[10] | ((uint16_t)data[11] << 8U);
    }
    if (error == SC_FRAME_ERROR_CRC_FAIL) {
        ++s_crc_errors;
        if (msg_id == SCBP_MSG_ID_ATTITUDE &&
            frame_length == SC_DUAL_ATTITUDE_PAYLOAD_LENGTH) {
            ++s_dual_crc_reject;
        }
    }
    ESP_LOGI(TAG, "SCBP_RX_ERROR error=%d version=%u msg=0x%04X length=%u",
             error, (unsigned)version, (unsigned)msg_id,
             (unsigned)frame_length);
}

static void smartcar_service_task(void *context)
{
    (void)context;
    TickType_t last_radar_status = xTaskGetTickCount();
    TickType_t last_stack_report = last_radar_status;
    s_ping_rx = 0U; s_pong_tx = 0U; s_crc_errors = 0U; s_stats_printed = false;
    s_dual_len_reject = 0U;
    s_dual_schema_reject = 0U;
    s_dual_crc_reject = 0U;
    s_dual_seq_gap = 0U;
    s_dual_duplicate = 0U;
    s_dual_notify_drop = 0U;
    s_dual_ble_not_ready = 0U;
    sc_frame_parser_init(&s_parser, command_bridge_on_frame,
                         command_bridge_on_error, NULL);
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
        const int received = stm_uart_receive_nonblock(s_uart_rx_buffer,
                                                        sizeof(s_uart_rx_buffer));
        if (received > 0) {
            (void)sc_frame_parser_feed(&s_parser, s_uart_rx_buffer,
                                       (size_t)received);
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
            ESP_LOGI(TAG, "STACK_HWM task=%s min_free_bytes=%u stack_bytes=%u",
                     SMARTCAR_SERVICE_TASK_NAME,
                     (unsigned)s_stack_min_free_bytes,
                     (unsigned)SMARTCAR_SERVICE_TASK_STACK);
            ESP_LOGI(TAG, "DUAL_ATTITUDE_STATS len_reject=%lu schema_reject=%lu "
                     "crc_reject=%lu seq_gap=%lu duplicate=%lu notify_drop=%lu "
                     "ble_not_ready=%lu",
                     (unsigned long)s_dual_len_reject,
                     (unsigned long)s_dual_schema_reject,
                     (unsigned long)s_dual_crc_reject,
                     (unsigned long)s_dual_seq_gap,
                     (unsigned long)s_dual_duplicate,
                     (unsigned long)s_dual_notify_drop,
                     (unsigned long)s_dual_ble_not_ready);
        }
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
    s_ble_rx_dropped = 0U;
    s_ble_rx_protocol_errors = 0U;
    s_stack_hwm_valid = false;
    s_ble_rx_queue = xQueueCreateStatic(
        SMARTCAR_SERVICE_BLE_RX_QUEUE_DEPTH, sizeof(smartcar_ble_rx_item_t),
        s_ble_rx_queue_buffer, &s_ble_rx_queue_storage);
    if (s_ble_rx_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    sc_app_parser_init(&s_app_parser, app_command_on_frame,
                       app_command_on_error, NULL);
#if !SMARTCAR_BMI323_DEBUG_ONLY
    radar_calibration_manager_init();
#endif
    if (xTaskCreate(smartcar_service_task, SMARTCAR_SERVICE_TASK_NAME,
                    SMARTCAR_SERVICE_TASK_STACK, NULL,
                    SMARTCAR_SERVICE_TASK_PRIORITY, &s_task) != pdPASS) {
        s_ble_rx_queue = NULL;
        return ESP_ERR_NO_MEM;
    }
    return s3_ble_register_rx_callback(service_ble_rx_enqueue, NULL);
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    if (task == s_task && s_stack_hwm_valid) {
        ESP_EARLY_LOGE(TAG,
                       "STACK_OVERFLOW task=%s min_free_bytes=%u "
                       "trigger=FreeRTOS_canary_context_switch",
                       task_name != NULL ? task_name : "<unknown>",
                       (unsigned)s_stack_min_free_bytes);
    } else {
        ESP_EARLY_LOGE(TAG,
                       "STACK_OVERFLOW task=%s min_free_bytes=unavailable "
                       "trigger=FreeRTOS_canary_context_switch",
                       task_name != NULL ? task_name : "<unknown>");
    }
    abort();
}
