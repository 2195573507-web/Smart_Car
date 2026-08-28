#include "smartcar_service.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
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
#include "srp_crc.h"
#include "srp_link.h"
#include "srp_codec.h"
#include "srp_wire.h"
#include "stm_uart.h"

#define SMARTCAR_SERVICE_TASK_NAME "smartcar_svc"
/*
 * ESP-IDF measures xTaskCreate() stack depth in bytes. The service task is
 * the serialized consumer for BLE RX, SRP parsing, UART forwarding, and
 * reconnect/stop handling, so reserve 16 KB for nested protocol callbacks.
 *
 * menuconfig recommendation: FreeRTOS stack overflow check = Method 3
 * (ISR + MPU), when this check is available for the selected target/IDF.
 * Real-vehicle check: run the `tasks` CLI command during BLE reconnect and
 * radar-burst tests; keep `smartcar_svc` min_free_words > 4096.
 */
#define SMARTCAR_SERVICE_TASK_STACK 16384U
#define SMARTCAR_SERVICE_TASK_PRIORITY 8U
#define SMARTCAR_SERVICE_TASK_DELAY_TICKS 1U
#define SMARTCAR_SERVICE_BLE_RX_QUEUE_DEPTH 8U
#define SMARTCAR_SERVICE_BLE_RX_BUDGET 4U
#define SMARTCAR_SERVICE_STACK_REPORT_PERIOD_MS UINT32_C(5000)
#define SMARTCAR_SERVICE_BUS_OFF_RECOVERY_MS UINT32_C(100)
#define APP_RADAR_STATUS_PERIOD_MS UINT32_C(1000)
#define APP_V2_HEARTBEAT_PERIOD_MS UINT32_C(500)
#define APP_V2_SESSION_TTL_MS UINT32_C(3000)
#define SMARTCAR_SERVICE_BAUD_SWITCH_GUARD_MS UINT32_C(20)
#define SMARTCAR_SERVICE_SYNC_RETRY_PERIOD_MS UINT32_C(500)
#define SMARTCAR_SERVICE_SYNC_HEARTBEAT_PERIOD_MS UINT32_C(100)
#define SMARTCAR_SERVICE_SYNC_MAX_ATTEMPTS UINT8_C(10)
#define SMARTCAR_SERVICE_SYNC_TIMEOUT_RETRY_MS UINT32_C(500)
#define SMARTCAR_SERVICE_STM_FRAME_TIMEOUT_MS UINT32_C(1500)
#define SMARTCAR_SERVICE_UART_DIAG_PERIOD_MS UINT32_C(1000)
#define SMARTCAR_SERVICE_RX_RESET_LOG_PERIOD_MS UINT32_C(500)
#define SC_APP_SYNC_TIMEOUT_ERROR UINT16_C(0x0201)
#define SRP_DEBUG_RAW_FRAME_MAX_BYTES UINT16_C(64)

#ifndef SMARTCAR_BMI323_DEBUG_ONLY
#define SMARTCAR_BMI323_DEBUG_ONLY 0
#endif

typedef struct {
    uint16_t length;
    uint8_t bytes[SC_APP_FRAME_MAX_SIZE];
} smartcar_ble_rx_item_t;

typedef struct {
    bool valid;
    uint8_t app_type;
    uint8_t app_version;
    uint32_t app_session_id;
    uint32_t app_sequence;
    uint16_t message_id;
    uint8_t length;
    uint32_t generation;
    uint64_t valid_until_us;
    uint8_t payload[SRP_PAYLOAD_CHASSIS_SPEED_CMD_SIZE];
} smartcar_motion_command_t;

typedef struct {
    uint8_t version;
    uint32_t session_id;
    uint32_t sequence;
    uint64_t valid_until_us;
} smartcar_app_command_meta_t;

typedef struct {
    uint8_t app_type;
    uint8_t app_version;
    uint32_t session_id;
    uint32_t sequence;
} smartcar_app_tx_context_t;

typedef struct {
    bool active;
    uint32_t session_id;
    uint32_t last_sequence;
    uint64_t last_activity_us;
    bool have_sequence;
    bool have_last_ack;
    uint32_t last_ack_sequence;
    uint8_t last_ack_type;
    uint8_t last_ack_result;
    uint8_t last_ack_stage;
} smartcar_app_v2_session_t;

typedef enum {
    SMARTCAR_SYNC_UART_READY = 0U,
    SMARTCAR_SYNC_SYNCING = 1U,
    SMARTCAR_SYNC_SYNCED = 2U,
    SMARTCAR_SYNC_TIMEOUT = 3U
} smartcar_sync_state_t;

static const char *TAG = "SRP";
static srp_parser_t s_parser;
static srp_link_t s_link;
static sc_app_parser_t s_app_parser;
static QueueHandle_t s_ble_rx_queue;
static StaticQueue_t s_ble_rx_queue_storage;
static TaskHandle_t s_task;
static uint8_t s_link_ready;
static bool s_bus_off_recovery_pending;
static bool s_bus_off_latched;
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
static volatile bool s_ble_disconnect_stop_pending;
static bool s_baud_change_pending;
static uint32_t s_baud_change_value;
static uint64_t s_baud_change_due_us;

static uint8_t s_ble_rx_queue_buffer[
    SMARTCAR_SERVICE_BLE_RX_QUEUE_DEPTH * sizeof(smartcar_ble_rx_item_t)]
    __attribute__((aligned(4)));
static uint8_t s_uart_rx_buffer[256U];
static smartcar_ble_rx_item_t s_ble_rx_item;
static smartcar_motion_command_t s_motion_inflight;
static smartcar_motion_command_t s_motion_pending_target;
static smartcar_motion_command_t s_motion_pending_scale;
static bool s_motion_tx_in_flight;
static uint32_t s_motion_generation;
static uint8_t s_app_tx_frame[SC_APP_FRAME_MAX_SIZE];
static smartcar_app_v2_session_t s_app_v2_session;
static uint32_t s_app_v2_next_session_id = 0x51A70001U;
static smartcar_app_tx_context_t s_pid_tx_context;
static smartcar_app_tx_context_t s_baud_tx_context;
static smartcar_sync_state_t s_sync_state;
static uint8_t s_sync_attempts;
static uint8_t s_sync_tx_sequence;
static uint64_t s_next_sync_us;
static uint64_t s_last_sync_timeout_notify_us;
static uint64_t s_last_stm_frame_us;
static uint64_t s_last_uart_diag_us;
static uint64_t s_last_rx_reset_log_us;
static uint64_t s_last_header_fail_log_us;

static void cancel_motion_transactions(void);

static bool command_bridge_decode_baudrate(const srp_frame_t *frame,
                                           uint32_t *baud_rate)
{
    srp_tlv_iter_t iterator;
    bool found = false;
    uint8_t tag;
    uint8_t value_length;
    const uint8_t *value;

    if (frame == NULL || baud_rate == NULL || frame->payload == NULL ||
        (frame->flags & SRP_FLAG_TLV) == 0U || frame->length < 2U) {
        return false;
    }
    srp_tlv_iter_init(&iterator, frame->payload, frame->length);
    while (iterator.offset < iterator.length) {
        if (iterator.length - iterator.offset < 2U ||
            !srp_tlv_next(&iterator, &tag, &value_length, &value)) {
            return false;
        }
        if (tag == SRP_TLV_TAG_BAUDRATE) {
            if (found || value_length != 4U) {
                return false;
            }
            *baud_rate = srp_wire_read_u32_le(value);
            found = true;
        }
    }
    return found && (*baud_rate == SRP_BAUDRATE_DEFAULT ||
                     *baud_rate == SRP_BAUDRATE_DEBUG);
}

static uint64_t now_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

static void command_bridge_restart_sync(const char *reason)
{
    const uint64_t restart_at = now_us();

    stm_uart_recover();
    srp_parser_reset(&s_parser);
    srp_link_recover(&s_link);
    stm_uart_set_sync_state(false);
    s_sync_state = SMARTCAR_SYNC_UART_READY;
    s_sync_attempts = 0U;
    s_sync_tx_sequence = 0U;
    s_next_sync_us = restart_at;
    s_last_sync_timeout_notify_us = 0U;
    s_last_stm_frame_us = 0U;
    s_bus_off_recovery_pending = false;
    s_bus_off_latched = false;
    s_motion_pending_target.valid = false;
    s_motion_pending_scale.valid = false;
    cancel_motion_transactions();
    ESP_LOGW(TAG, "SRP sync recovery reason=%s; probing STM32 again",
             reason == NULL ? "unknown" : reason);
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

static void command_bridge_notify_sync_status(bool synced, bool timeout)
{
    uint8_t payload[4] = {0U, 0U, 0U, 0U};

    if (timeout) {
        payload[3] = (uint8_t)(SC_APP_SYNC_TIMEOUT_ERROR & 0xFFU);
        payload[2] = (uint8_t)(SC_APP_SYNC_TIMEOUT_ERROR >> 8U);
    }
    if (!notify_app_frame(SC_APP_TYPE_STATUS, payload, sizeof(payload)) && timeout) {
        ESP_LOGW(TAG, "SYNC_TIMEOUT app status notify dropped");
    }
    ESP_LOGI(TAG, "SRP sync state=%s%s", synced ? "ESTABLISHED" : "WAITING",
             timeout ? " timeout" : "");
}

static void command_bridge_enter_sync_timeout(void)
{
    const uint64_t timeout_at = now_us();

    if (s_sync_state == SMARTCAR_SYNC_TIMEOUT) {
        return;
    }
    /* Drop stale UART bytes and link transactions before reopening the retry
     * window. This is the recovery boundary after ten unanswered probes. */
    stm_uart_recover();
    srp_parser_reset(&s_parser);
    srp_link_recover(&s_link);
    s_sync_state = SMARTCAR_SYNC_TIMEOUT;
    stm_uart_set_sync_state(false);
    s_motion_pending_target.valid = false;
    s_motion_pending_scale.valid = false;
    cancel_motion_transactions();
    s_last_sync_timeout_notify_us = timeout_at;
    s_next_sync_us = timeout_at +
                     ((uint64_t)SMARTCAR_SERVICE_SYNC_TIMEOUT_RETRY_MS * UINT64_C(1000));
    command_bridge_notify_sync_status(false, true);
    ESP_LOGW(TAG, "SYNC_TIMEOUT attempts=%u", (unsigned)s_sync_attempts);
}

static void command_bridge_sync_step(uint64_t now)
{
    static const uint8_t payload[SRP_PAYLOAD_CMD_SYNC_REQ_SIZE] = {
        SRP_PROTOCOL_VERSION_MAJOR, SRP_PROTOCOL_VERSION_MINOR, 0U, 0U
    };

    if (s_sync_state == SMARTCAR_SYNC_SYNCED) {
        if (now < s_next_sync_us) {
            return;
        }
        s_sync_tx_sequence = s_link.next_sequence;
        if (srp_link_send(&s_link, SRP_PRIORITY_COMMAND,
                          SRP_NODE_STM32H757, SRP_MSG_ID_CMD_SYNC_REQ,
                          SRP_FLAG_STREAM_DATA, payload, sizeof(payload),
                          (uint32_t)(now / UINT64_C(1000)), NULL, NULL) != 0) {
            ESP_LOGW(TAG, "SRP heartbeat transport send failed");
        }
        s_next_sync_us = now +
                         ((uint64_t)SMARTCAR_SERVICE_SYNC_HEARTBEAT_PERIOD_MS *
                          UINT64_C(1000));
        return;
    }
    if (s_sync_state == SMARTCAR_SYNC_TIMEOUT) {
        if (now < s_next_sync_us) {
            if (now - s_last_sync_timeout_notify_us >= UINT64_C(1000000)) {
                s_last_sync_timeout_notify_us = now;
                command_bridge_notify_sync_status(false, true);
            }
            return;
        }
        s_sync_state = SMARTCAR_SYNC_UART_READY;
        s_sync_attempts = 0U;
        s_next_sync_us = now;
    }
    if (now < s_next_sync_us) {
        return;
    }
    if (s_sync_attempts >= SMARTCAR_SERVICE_SYNC_MAX_ATTEMPTS) {
        command_bridge_enter_sync_timeout();
        return;
    }
    s_sync_state = SMARTCAR_SYNC_SYNCING;
    s_sync_tx_sequence = s_link.next_sequence;
    if (srp_link_send(&s_link, SRP_PRIORITY_COMMAND, SRP_NODE_STM32H757,
                      SRP_MSG_ID_CMD_SYNC_REQ, SRP_FLAG_STREAM_DATA,
                      payload, sizeof(payload),
                      (uint32_t)(now / UINT64_C(1000)), NULL, NULL) != 0) {
        ESP_LOGW(TAG, "SYNC_REQ transport send failed attempt=%u",
                 (unsigned)(s_sync_attempts + 1U));
    }
    ++s_sync_attempts;
    s_next_sync_us = now + ((uint64_t)SMARTCAR_SERVICE_SYNC_RETRY_PERIOD_MS *
                            UINT64_C(1000));
}

static int command_bridge_transport_send(const uint8_t *data, uint16_t length,
                                         void *context)
{
    (void)context;
    return stm_uart_send(data, length) == (int)length ? 0 : -1;
}

static void command_bridge_log_uart_diag(uint64_t now)
{
    stm_uart_stats_t stats;

    if (now - s_last_uart_diag_us <
        ((uint64_t)SMARTCAR_SERVICE_UART_DIAG_PERIOD_MS * UINT64_C(1000))) {
        return;
    }
    s_last_uart_diag_us = now;
    stm_uart_get_stats(&stats);
    ESP_LOGI(TAG,
             "STM_UART_DIAG sync=%u rx=%lu rx_reads=%lu rx_buf=%u tx=%lu tx_q=%u "
             "tx_qdrop=%lu tx_err=%lu rx_err=%lu break=%lu overflow=%lu "
             "break_recoveries=%lu drop=%lu hal=%lu",
             (unsigned)s_sync_state, (unsigned long)stats.rx_bytes,
             (unsigned long)stats.rx_task_reads, (unsigned)stats.rx_buffered,
             (unsigned long)stats.tx_bytes, (unsigned)stats.tx_queue_pending,
             (unsigned long)stats.tx_queue_drop,
             (unsigned long)stats.tx_write_errors,
             (unsigned long)stats.rx_error_events,
             (unsigned long)stats.break_events,
             (unsigned long)stats.overflow,
             (unsigned long)stats.break_recoveries,
             (unsigned long)stats.drop,
             (unsigned long)stats.hal_error);
}

static void command_bridge_check_stm_liveness(uint64_t now)
{
    const uint64_t timeout_us =
        (uint64_t)SMARTCAR_SERVICE_STM_FRAME_TIMEOUT_MS * UINT64_C(1000);

    if (s_sync_state != SMARTCAR_SYNC_SYNCED || s_last_stm_frame_us == 0U ||
        now - s_last_stm_frame_us < timeout_us) {
        return;
    }
    ESP_LOGW(TAG, "STM SRP receive timeout age_ms=%llu; restarting sync",
             (unsigned long long)((now - s_last_stm_frame_us) / UINT64_C(1000)));
    command_bridge_restart_sync("STM_RX_TIMEOUT");
}

static void command_bridge_bus_off(void *context)
{
    (void)context;
    if (s_bus_off_latched) {
        return;
    }
    s_bus_off_latched = true;
    stm_uart_recover();
    stm_uart_set_sync_state(false);
    s_sync_state = SMARTCAR_SYNC_UART_READY;
    s_sync_attempts = 0U;
    s_next_sync_us = now_us() + UINT64_C(100000);
    s_bus_off_recovery_pending = true;
    s_bus_off_recovery_at_us = now_us() +
        ((uint64_t)SMARTCAR_SERVICE_BUS_OFF_RECOVERY_MS * UINT64_C(1000));
    ESP_LOGE(TAG, "BUS_OFF: UART2 receive queue flushed");
}

static void command_bridge_send_response(const srp_frame_t *request,
                                         uint8_t is_error, uint8_t status_code)
{
    if (request == NULL) {
        return;
    }
    (void)srp_link_send_fast_response(&s_link, SRP_PRIORITY_COMMAND,
                                       0U, is_error, request->type,
                                       request->sequence, status_code,
                                       (uint32_t)(now_us() / UINT64_C(1000)));
}

static bool relay_dual_attitude(const srp_frame_t *frame)
{
    if (frame == NULL || frame->payload == NULL ||
        frame->length != SRP_PAYLOAD_DUAL_AHRS_SIZE) {
        ++s_dual_len_reject;
        return false;
    }
    if (frame->payload[0] != SRP_DUAL_AHRS_SCHEMA || frame->payload[2] != 0U ||
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

static void relay_telemetry(const srp_frame_t *frame, uint16_t message_id)
{
    if (frame == NULL || frame->payload == NULL) {
        return;
    }
#if SMARTCAR_BMI323_DEBUG_ONLY
    (void)message_id;
#else
    uint8_t app_type;

    switch (message_id) {
    case SRP_MSG_ID_RSP_BOOT_INFO: {
        const uint8_t attempts = s_sync_attempts;
        const bool was_synced = s_sync_state == SMARTCAR_SYNC_SYNCED;

        if (frame->length != SRP_PAYLOAD_RSP_BOOT_INFO_SIZE ||
            frame->payload == NULL ||
            frame->priority != SRP_PRIORITY_COMMAND ||
            frame->flags != SRP_FLAG_STREAM_DATA ||
            frame->payload[0] != SRP_PROTOCOL_VERSION_MAJOR ||
            frame->payload[1] != SRP_PROTOCOL_VERSION_MINOR ||
            frame->payload[2] != SRP_STM_STATE_HOST_SYNCED ||
            frame->payload[3] != SRP_SYNC_FLAG_VERSION_OK ||
            frame->payload[5] != 0U || frame->payload[6] != 0U ||
            frame->payload[7] != 0U ||
            frame->payload[4] != s_sync_tx_sequence) {
            ESP_LOGW(TAG, "invalid RSP_BOOT_INFO ignored");
            return;
        }
        s_sync_state = SMARTCAR_SYNC_SYNCED;
        s_bus_off_latched = false;
        stm_uart_set_sync_state(true);
        s_last_stm_frame_us = now_us();
        s_next_sync_us = s_last_stm_frame_us +
                         ((uint64_t)SMARTCAR_SERVICE_SYNC_HEARTBEAT_PERIOD_MS *
                          UINT64_C(1000));
        if (!was_synced) {
            command_bridge_notify_sync_status(true, false);
            ESP_LOGI(TAG, "SRP SYNCED after %u attempt(s)",
                     (unsigned)(attempts == 0U ? 1U : attempts));
        }
        return;
    }
    case SRP_MSG_ID_ATTITUDE:
        (void)relay_dual_attitude(frame);
        return;
    case SRP_MSG_ID_IMU_CAL_STATUS:
        if (frame->length != SRP_PAYLOAD_IMU_CAL_STATUS_SIZE) {
            return;
        }
        app_type = UINT8_C(0x12);
        break;
    case SRP_MSG_ID_IMU_TELEMETRY:
        if (frame->length != SRP_PAYLOAD_IMU_TELEMETRY_SIZE ||
            (frame->payload[0] != SRP_IMU_SENSOR_LSM303 &&
             frame->payload[0] != SRP_IMU_SENSOR_BMI323)) {
            return;
        }
        app_type = UINT8_C(0x27);
        break;
    case SRP_MSG_ID_RADAR_STATUS:
        if (frame->length != SRP_PAYLOAD_RADAR_STATUS_SIZE) {
            return;
        }
        app_type = SC_APP_TYPE_RADAR_STATUS;
        break;
    case SRP_MSG_ID_WHEEL_SPEED_STATUS:
        if (frame->length != SRP_PAYLOAD_WHEEL_SPEED_STATUS_SIZE ||
            !srp_wire_read_f32_array_le(frame->payload, frame->length,
                                         (float[4]){0}, 4U)) {
            return;
        }
        app_type = SC_APP_TYPE_WHEEL_SPEED_STATUS;
        break;
    case SRP_MSG_ID_CHASSIS_STATE: {
        float values[4] = {0};

        if (frame->length != SRP_PAYLOAD_CHASSIS_STATE_SIZE ||
            frame->payload[0] != SRP_CHASSIS_STATE_SCHEMA ||
            (frame->payload[1] & UINT8_C(0xF0)) != 0U ||
            frame->payload[2] != 0U || frame->payload[3] != 0U ||
            !srp_wire_read_f32_array_le(&frame->payload[8], 16U,
                                         values, 4U) ||
            !isfinite(values[0]) || !isfinite(values[1]) ||
            !isfinite(values[2]) || !isfinite(values[3])) {
            return;
        }
        app_type = SC_APP_TYPE_CHASSIS_STATE;
        break;
    }
    case SRP_MSG_ID_WHEEL_CONTROL_STATUS: {
        float values[9] = {0};

        if (frame->length != SRP_PAYLOAD_WHEEL_CONTROL_STATUS_SIZE ||
            frame->payload[0] != SRP_WHEEL_CONTROL_STATUS_SCHEMA ||
            (frame->payload[1] != SRP_CHASSIS_MODE_DIFF &&
             frame->payload[1] != SRP_CHASSIS_MODE_WHEEL_INDEPENDENT) ||
            frame->payload[2] != 0U || frame->payload[3] != 0U ||
            !srp_wire_read_f32_array_le(&frame->payload[8], 4U, values, 1U) ||
            !srp_wire_read_f32_array_le(&frame->payload[12], 16U,
                                         &values[1], 4U) ||
            !srp_wire_read_f32_array_le(&frame->payload[28], 16U,
                                         &values[5], 4U) ||
            !isfinite(values[0]) || values[0] < 0.0f || values[0] > 4.0f) {
            return;
        }
        app_type = SC_APP_TYPE_WHEEL_CONTROL_STATUS;
        break;
    }
    case SRP_MSG_ID_POWER_STATUS:
        if (frame->length != SRP_PAYLOAD_POWER_STATUS_SIZE ||
            !isfinite(srp_wire_read_f32_le(frame->payload))) {
            return;
        }
        app_type = SC_APP_TYPE_POWER_STATUS;
        break;
    default:
        return;
    }
    (void)notify_app_frame(app_type, frame->payload, frame->length);
#endif
}

static void command_bridge_ready_response(srp_link_tx_result_t result,
                                          uint8_t status_code, void *context)
{
    (void)context;
    radar_calibration_manager_on_ready_response(result, status_code);
}

static int command_bridge_send_radar_pwm_ready(uint8_t speed_percent, void *context)
{
    const uint8_t payload[SRP_PAYLOAD_RADAR_PWM_READY_SIZE] = {speed_percent};

    (void)context;
    return srp_link_send(&s_link, SRP_PRIORITY_COMMAND,
                          SRP_NODE_STM32H757, SRP_MSG_ID_RADAR_PWM_READY,
                          SRP_FLAG_ACK_REQUIRED, payload, sizeof(payload),
                          (uint32_t)(now_us() / UINT64_C(1000)),
                          command_bridge_ready_response, NULL);
}

static void command_bridge_on_frame(const srp_frame_t *frame, void *context)
{
    const uint16_t message_id = frame == NULL ? 0U : frame->type;
    const bool ack_required = frame != NULL &&
        (frame->flags & SRP_FLAG_ACK_REQUIRED) != 0U;
    bool admitted = false;

    (void)context;
    if (frame == NULL) {
        return;
    }

    switch (message_id) {
    case SRP_MSG_ID_RSP_BOOT_INFO:
    case SRP_MSG_ID_ATTITUDE:
    case SRP_MSG_ID_IMU_CAL_STATUS:
    case SRP_MSG_ID_IMU_TELEMETRY:
    case SRP_MSG_ID_RADAR_STATUS:
    case SRP_MSG_ID_WHEEL_SPEED_STATUS:
    case SRP_MSG_ID_CHASSIS_STATE:
    case SRP_MSG_ID_WHEEL_CONTROL_STATUS:
    case SRP_MSG_ID_POWER_STATUS:
        relay_telemetry(frame, message_id);
        return;
    case SRP_MSG_ID_LOG:
        log_bridge_handle(frame);
        return;
    case SRP_MSG_ID_SYS_CONFIG: {
        uint32_t baud_rate = 0U;
        const bool valid = command_bridge_decode_baudrate(frame, &baud_rate);

        if (ack_required) {
            command_bridge_send_response(frame, valid ? 0U : 1U,
                                         valid ? SRP_FAST_RESP_OK :
                                                 SRP_FAST_RESP_INVALID_PARAM);
        }
        if (valid) {
            s_baud_change_pending = true;
            s_baud_change_value = baud_rate;
            s_baud_change_due_us = now_us() +
                                   ((uint64_t)SMARTCAR_SERVICE_BAUD_SWITCH_GUARD_MS *
                                    UINT64_C(1000));
        }
        return;
    }
    case SRP_MSG_ID_BOOT_READY:
#if SMARTCAR_BMI323_DEBUG_ONLY
        admitted = false;
#else
        admitted = radar_calibration_manager_on_boot_ready(frame->payload, frame->length);
#endif
        break;
    case SRP_MSG_ID_CAL_EVENT:
#if SMARTCAR_BMI323_DEBUG_ONLY
        admitted = false;
#else
        admitted = radar_calibration_manager_on_cal_event(frame->payload, frame->length);
#endif
        break;
    default:
        if (ack_required) {
            command_bridge_send_response(frame, 1U, SRP_FAST_RESP_INVALID_PARAM);
        }
        return;
    }

    if (ack_required) {
        command_bridge_send_response(frame, admitted ? 0U : 1U,
                                     admitted ? SRP_FAST_RESP_OK : SRP_FAST_RESP_BUSY);
    }
}

static void command_bridge_parsed_frame(const srp_frame_t *frame, void *context)
{
    (void)context;
    if (frame != NULL) {
        s_last_stm_frame_us = now_us();
    }
    srp_link_receive(&s_link, frame);
}

static void command_bridge_log_crc_debug(const uint8_t *data, size_t length)
{
    uint16_t payload_length;
    uint16_t expected_crc;
    uint16_t received_crc;
    size_t expected_length;
    size_t raw_length = 0U;
    const size_t raw_count = length < SRP_DEBUG_RAW_FRAME_MAX_BYTES
                                 ? length
                                 : SRP_DEBUG_RAW_FRAME_MAX_BYTES;
    char raw_frame[(SRP_DEBUG_RAW_FRAME_MAX_BYTES * 3U) + 1U];

    if (data == NULL || length < (size_t)SRP_HEADER_SIZE + SRP_TRAILER_SIZE ||
        data[0] != SRP_MAGIC_BYTE0 || data[1] != SRP_MAGIC_BYTE1) {
        return;
    }
    payload_length = (uint16_t)data[2] | ((uint16_t)data[3] << 8U);
    expected_length = (size_t)SRP_HEADER_SIZE + payload_length + SRP_TRAILER_SIZE;
    if (payload_length > SRP_MAX_PAYLOAD || length != expected_length) {
        return;
    }
    expected_crc = srp_crc16_ccitt_false(&data[2], 6U + payload_length);
    received_crc = (uint16_t)data[8U + payload_length] |
                   ((uint16_t)data[9U + payload_length] << 8U);
    raw_frame[0] = '\0';
    for (size_t index = 0U; index < raw_count; ++index) {
        const int written = snprintf(&raw_frame[raw_length],
                                     sizeof(raw_frame) - raw_length,
                                     index == 0U ? "%02X" : " %02X",
                                     (unsigned)data[index]);
        if (written < 0 || (size_t)written >= sizeof(raw_frame) - raw_length) {
            break;
        }
        raw_length += (size_t)written;
    }
    ESP_LOGW(TAG,
             "[SRP_DEBUG] Expected CRC=0x%04X, Received CRC=0x%04X, "
             "Raw Frame: %s%s",
             (unsigned)expected_crc, (unsigned)received_crc, raw_frame,
             length > raw_count ? " ..." : "");
}

static void command_bridge_log_header_fail(srp_parser_error_t error)
{
    const uint64_t now = now_us();

    if (error != SRP_PARSER_ERROR_MAGIC && error != SRP_PARSER_ERROR_LENGTH &&
        error != SRP_PARSER_ERROR_HEADER) {
        return;
    }
    if (s_last_header_fail_log_us != 0U &&
        now - s_last_header_fail_log_us < UINT64_C(2000000)) {
        return;
    }
    s_last_header_fail_log_us = now;
    ESP_LOGW(TAG,
             "[SRP_HEADER_FAIL] drop_byte=0x%02X expected_state=%d error=%u",
             (unsigned)s_parser.last_drop_byte,
             (int)s_parser.last_error_state, (unsigned)error);
}

static void command_bridge_on_parser_error(srp_parser_error_t error,
                                           const uint8_t *data, size_t length,
                                           void *context)
{
    (void)context;
    ++s_parser_errors;
    command_bridge_log_header_fail(error);
    if (error == SRP_PARSER_ERROR_CRC && data != NULL && length >= SRP_HEADER_SIZE) {
        if ((uint8_t)data[6] == SRP_MSG_ID_ATTITUDE) {
            ++s_dual_crc_reject;
        }
        command_bridge_log_crc_debug(data, length);
    }
    /* A random preamble byte is expected while seeking AA 55. It is useful
     * for diagnostics but must not accumulate REC toward SRP BUS_OFF. */
    if (error != SRP_PARSER_ERROR_MAGIC) {
        srp_link_report_parser_error(&s_link, error);
    }
    if (error != SRP_PARSER_ERROR_MAGIC && error != SRP_PARSER_ERROR_LENGTH) {
        ESP_LOGW(TAG, "RX parser error=%u bytes=%u rec=%u tec=%u",
                 (unsigned)error, (unsigned)length,
                 (unsigned)srp_link_get_rec(&s_link),
                 (unsigned)srp_link_get_tec(&s_link));
    }
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

static void append_u16_le(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)(value >> 8U);
}

static void append_u32_le(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)((value >> 8U) & 0xFFU);
    out[2] = (uint8_t)((value >> 16U) & 0xFFU);
    out[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static bool notify_app_v2(uint8_t type, const uint8_t *payload, uint16_t length)
{
    uint16_t frame_length = 0U;

    if (sc_app_frame_encode_version(SC_APP_FRAME_VERSION_V2, type, payload,
                                    length, s_app_tx_frame,
                                    sizeof(s_app_tx_frame), &frame_length) != 0) {
        return false;
    }
    return s3_ble_notify_send(s_app_tx_frame, frame_length) == ESP_OK;
}

static void app_v2_send_hello_ack(void)
{
    uint8_t payload[13] = {0};

    append_u32_le(&payload[1], s_app_v2_session.session_id);
    append_u16_le(&payload[5], (uint16_t)APP_V2_HEARTBEAT_PERIOD_MS);
    append_u16_le(&payload[7], (uint16_t)APP_V2_SESSION_TTL_MS);
    append_u32_le(&payload[9], 0x00000007U);
    payload[0] = SC_APP_FRAME_VERSION_V2;
    (void)notify_app_v2(SC_APP_V2_TYPE_HELLO_ACK, payload, sizeof(payload));
}

static void app_v2_send_heartbeat_ack(uint32_t sequence)
{
    uint8_t payload[9] = {0};

    append_u32_le(&payload[0], s_app_v2_session.session_id);
    append_u32_le(&payload[4], sequence);
    payload[8] = SC_APP_V2_RESULT_OK;
    (void)notify_app_v2(SC_APP_V2_TYPE_HEARTBEAT_ACK, payload, sizeof(payload));
}

static void app_v2_send_command_ack(uint32_t sequence, uint8_t app_type,
                                    uint8_t result, uint8_t stage)
{
    uint8_t payload[11] = {0};

    append_u32_le(&payload[0], s_app_v2_session.session_id);
    append_u32_le(&payload[4], sequence);
    payload[8] = app_type;
    payload[9] = result;
    payload[10] = stage;
    (void)notify_app_v2(SC_APP_V2_TYPE_COMMAND_ACK, payload, sizeof(payload));
}

static void app_v2_remember_command_ack(uint32_t sequence, uint8_t app_type,
                                        uint8_t result, uint8_t stage)
{
    if (s_app_v2_session.active && sequence == s_app_v2_session.last_sequence) {
        s_app_v2_session.have_last_ack = true;
        s_app_v2_session.last_ack_sequence = sequence;
        s_app_v2_session.last_ack_type = app_type;
        s_app_v2_session.last_ack_result = result;
        s_app_v2_session.last_ack_stage = stage;
    }
    app_v2_send_command_ack(sequence, app_type, result, stage);
}

static bool start_motion_command(const smartcar_motion_command_t *command);

static void send_command_result(uint8_t app_version, uint8_t app_type,
                                uint32_t session_id, uint32_t sequence,
                                uint8_t result, uint8_t stage)
{
    if (app_version == SC_APP_FRAME_VERSION_V2) {
        if (s_app_v2_session.active && s_app_v2_session.session_id == session_id) {
            app_v2_remember_command_ack(sequence, app_type, result, stage);
        }
    } else {
        send_app_ack(app_type, result);
    }
}

static smartcar_motion_command_t *pending_motion_slot(uint16_t message_id)
{
    return message_id == SRP_MSG_ID_MASTER_SPEED_CMD
               ? &s_motion_pending_scale
               : &s_motion_pending_target;
}

static bool motion_command_is_zero_target(const smartcar_motion_command_t *command)
{
    float speeds[4] = {0.0f};

    if (command == NULL) {
        return false;
    }
    if (command->message_id == SRP_MSG_ID_CHASSIS_HEADING_CMD) {
        return command->length == SRP_PAYLOAD_CHASSIS_HEADING_CMD_SIZE &&
               srp_wire_read_f32_le(&command->payload[0]) == 0.0f &&
               srp_wire_read_f32_le(&command->payload[4]) == 0.0f &&
               srp_wire_read_u32_le(&command->payload[8]) ==
                   SRP_CHASSIS_HEADING_FLAGS_NONE;
    }
    return command->message_id == SRP_MSG_ID_WHEEL_SPEED_CMD &&
           command->length == SRP_PAYLOAD_WHEEL_SPEED_CMD_SIZE &&
           srp_wire_read_f32_array_le(command->payload, command->length,
                                       speeds, 4U) &&
           fabsf(speeds[0]) <= 0.001f && fabsf(speeds[1]) <= 0.001f &&
           fabsf(speeds[2]) <= 0.001f && fabsf(speeds[3]) <= 0.001f;
}

static void cancel_motion_transactions(void)
{
    static const uint16_t message_ids[] = {
        SRP_MSG_ID_WHEEL_SPEED_CMD,
        SRP_MSG_ID_WHEEL_SPEED_SINGLE_CMD,
        SRP_MSG_ID_MASTER_SPEED_CMD,
        SRP_MSG_ID_CHASSIS_SPEED_CMD,
        SRP_MSG_ID_CHASSIS_HEADING_CMD
    };

    for (size_t index = 0U; index < sizeof(message_ids) / sizeof(message_ids[0]);
         ++index) {
        srp_link_cancel_message(&s_link, message_ids[index]);
    }
    s_motion_inflight.valid = false;
    s_motion_tx_in_flight = false;
}

static bool send_motion_stop(const smartcar_motion_command_t *command)
{
    int result;

    if (command == NULL) {
        return false;
    }
    cancel_motion_transactions();
    if (command->message_id == SRP_MSG_ID_CHASSIS_HEADING_CMD) {
        /* Target Yaw stop remains the same ACK-required SRP command so CM7
         * can clear the heading controller explicitly. */
        result = start_motion_command(command);
    } else {
        /* Preserve the legacy immediate wheel-stop stream semantics. */
        result = srp_link_send(
            &s_link, SRP_PRIORITY_COMMAND, SRP_NODE_STM32H757,
            SRP_MSG_ID_WHEEL_SPEED_CMD, SRP_FLAG_STREAM_DATA,
            command->payload, command->length,
            (uint32_t)(now_us() / UINT64_C(1000)), NULL, NULL);
    }

    if (result == 0 && command->message_id != SRP_MSG_ID_CHASSIS_HEADING_CMD) {
        send_command_result(command->app_version, command->app_type,
                            command->app_session_id, command->app_sequence,
                            SC_APP_ACK_OK, SC_APP_V2_STAGE_STOP_QUEUED);
    }
    return result == 0;
}

static void app_transaction_tx_complete(srp_link_tx_result_t result,
                                        uint8_t status_code, void *context)
{
    const smartcar_app_tx_context_t *tx = context;
    const uint8_t accepted = result == SRP_LINK_TX_OK &&
                             status_code == SRP_FAST_RESP_OK;
    if (tx == NULL) {
        send_app_ack(SC_APP_TYPE_WHEEL_SPEED_CMD,
                     accepted ? SC_APP_ACK_OK : SC_APP_ACK_REJECTED);
    } else {
        send_command_result(tx->app_version, tx->app_type, tx->session_id,
                            tx->sequence,
                            accepted ? SC_APP_ACK_OK : SC_APP_ACK_REJECTED,
                            accepted ? SC_APP_V2_STAGE_STM32_ACCEPTED :
                                       SC_APP_V2_STAGE_GATEWAY_ADMITTED);
    }
}

static void baud_config_tx_complete(srp_link_tx_result_t result,
                                    uint8_t status_code, void *context)
{
    const smartcar_app_tx_context_t *tx = context;
    const bool accepted = result == SRP_LINK_TX_OK &&
                          status_code == SRP_FAST_RESP_OK;

    if (tx != NULL) {
        send_command_result(tx->app_version, tx->app_type, tx->session_id,
                            tx->sequence,
                            accepted ? SC_APP_ACK_OK : SC_APP_ACK_REJECTED,
                            accepted ? SC_APP_V2_STAGE_STM32_ACCEPTED :
                                       SC_APP_V2_STAGE_GATEWAY_ADMITTED);
    }
    if (accepted) {
        s_baud_change_pending = true;
        s_baud_change_due_us = now_us() +
                               ((uint64_t)SMARTCAR_SERVICE_BAUD_SWITCH_GUARD_MS *
                                UINT64_C(1000));
    }
}

static void motion_command_tx_complete(srp_link_tx_result_t result,
                                       uint8_t status_code, void *context)
{
    smartcar_motion_command_t completed = s_motion_inflight;
    smartcar_motion_command_t next = {0};
    const uint8_t accepted = result == SRP_LINK_TX_OK &&
                             status_code == SRP_FAST_RESP_OK;

    (void)context;
    s_motion_inflight.valid = false;
    s_motion_tx_in_flight = false;
    send_command_result(completed.app_version, completed.app_type,
                        completed.app_session_id, completed.app_sequence,
                        accepted ? SC_APP_ACK_OK : SC_APP_ACK_REJECTED,
                        accepted ? SC_APP_V2_STAGE_STM32_ACCEPTED :
                                   SC_APP_V2_STAGE_GATEWAY_ADMITTED);

    if (s_motion_pending_target.valid) {
        next = s_motion_pending_target;
        s_motion_pending_target.valid = false;
        (void)start_motion_command(&next);
    } else if (s_motion_pending_scale.valid) {
        next = s_motion_pending_scale;
        s_motion_pending_scale.valid = false;
        (void)start_motion_command(&next);
    }
}

static bool start_motion_command(const smartcar_motion_command_t *command)
{
    uint32_t generation;
    int result;

    if (command == NULL || !command->valid ||
        command->length > sizeof(s_motion_inflight.payload)) {
        return false;
    }
    if (command->valid_until_us != 0U && now_us() >= command->valid_until_us) {
        send_command_result(command->app_version, command->app_type,
                            command->app_session_id, command->app_sequence,
                            SC_APP_V2_RESULT_EXPIRED,
                            SC_APP_V2_STAGE_GATEWAY_ADMITTED);
        return false;
    }
    s_motion_inflight = *command;
    generation = ++s_motion_generation;
    s_motion_inflight.generation = generation;
    s_motion_tx_in_flight = true;
    result = srp_link_send(
        &s_link, SRP_PRIORITY_COMMAND, SRP_NODE_STM32H757,
        s_motion_inflight.message_id, SRP_FLAG_ACK_REQUIRED,
        s_motion_inflight.payload, s_motion_inflight.length,
        (uint32_t)(now_us() / UINT64_C(1000)), motion_command_tx_complete,
        &s_motion_inflight);
    if (result != 0 && s_motion_tx_in_flight &&
        s_motion_inflight.generation == generation) {
        s_motion_tx_in_flight = false;
        s_motion_inflight.valid = false;
        send_command_result(command->app_version, command->app_type,
                            command->app_session_id, command->app_sequence,
                            SC_APP_V2_RESULT_REJECTED,
                            SC_APP_V2_STAGE_GATEWAY_ADMITTED);
    }
    return result == 0;
}

static bool queue_motion_command(uint8_t app_type, uint16_t message_id,
                                 const uint8_t *payload, uint8_t length,
                                 const smartcar_app_command_meta_t *meta)
{
    smartcar_motion_command_t command = {0};

    if (payload == NULL || length > sizeof(command.payload)) {
        return false;
    }
    command.valid = true;
    command.app_type = app_type;
    command.app_version = meta == NULL ? SC_APP_FRAME_VERSION : meta->version;
    command.app_session_id = meta == NULL ? 0U : meta->session_id;
    command.app_sequence = meta == NULL ? 0U : meta->sequence;
    command.valid_until_us = meta == NULL ? 0U : meta->valid_until_us;
    command.message_id = message_id;
    command.length = length;
    (void)memcpy(command.payload, payload, length);
    if (motion_command_is_zero_target(&command)) {
        s_motion_pending_target.valid = false;
        s_motion_pending_scale.valid = false;
        return send_motion_stop(&command);
    }
    if (s_motion_tx_in_flight) {
        /* Keep the newest target and scale independently. A speed update
         * cannot erase an independent MasterScale update. */
        *pending_motion_slot(message_id) = command;
        return true;
    }
    return start_motion_command(&command);
}

static void command_bridge_on_ble_disconnect(void *context)
{
    (void)context;
    /* The GATT callback only signals the service task; SRP link state is
     * serialized there so the stop frame cannot race another transmission. */
    s_motion_pending_target.valid = false;
    s_motion_pending_scale.valid = false;
    s_ble_disconnect_stop_pending = true;
}

static void command_bridge_send_zero_wheel_speed(void)
{
    uint8_t heading_payload[SRP_PAYLOAD_CHASSIS_HEADING_CMD_SIZE] = {0U};
    uint8_t payload[SRP_PAYLOAD_WHEEL_SPEED_CMD_SIZE];
    const float speeds[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    int heading_result;
    int wheel_result;

    s_motion_pending_target.valid = false;
    s_motion_pending_scale.valid = false;
    cancel_motion_transactions();
    /* Local disconnect/session safety must clear the CM7 heading controller
     * using the same SRP command as the App stop path. */
    heading_result = srp_link_send(
        &s_link, SRP_PRIORITY_COMMAND, SRP_NODE_STM32H757,
        SRP_MSG_ID_CHASSIS_HEADING_CMD, SRP_FLAG_ACK_REQUIRED,
        heading_payload, sizeof(heading_payload),
        (uint32_t)(now_us() / UINT64_C(1000)), NULL, NULL);
    srp_wire_write_f32_array_le(payload, speeds, 4U);
    wheel_result = srp_link_send(&s_link, SRP_PRIORITY_COMMAND,
                                 SRP_NODE_STM32H757, SRP_MSG_ID_WHEEL_SPEED_CMD,
                                 SRP_FLAG_STREAM_DATA, payload, sizeof(payload),
                                 (uint32_t)(now_us() / UINT64_C(1000)), NULL, NULL);
    if (heading_result != 0 || wheel_result != 0) {
        ESP_LOGE(TAG, "BLE disconnect stop frame send failed heading=%d wheel=%d",
                 heading_result, wheel_result);
    } else {
        ESP_LOGI(TAG, "BLE disconnect stop frames queued heading=0x17 wheel=0x02");
    }
}

static void app_command_on_frame(const sc_app_frame_view_t *input_frame, void *context)
{
    uint8_t result = SC_APP_ACK_REJECTED;
    sc_app_frame_view_t v2_command = {0};
    smartcar_app_command_meta_t meta = {0};
    const sc_app_frame_view_t *frame = input_frame;

    (void)context;
    if (frame == NULL) {
        return;
    }
    meta.version = SC_APP_FRAME_VERSION;

    if (frame->version == SC_APP_FRAME_VERSION_V2) {
        if (frame->type == SC_APP_V2_TYPE_HELLO && frame->length == 6U &&
            frame->payload != NULL && frame->payload[0] == SC_APP_FRAME_VERSION_V2 &&
            frame->payload[1] == SC_APP_FRAME_VERSION_V2) {
            s_app_v2_session.active = true;
            s_app_v2_session.session_id = ++s_app_v2_next_session_id;
            if (s_app_v2_session.session_id == 0U) {
                s_app_v2_session.session_id = ++s_app_v2_next_session_id;
            }
            s_app_v2_session.have_sequence = false;
            s_app_v2_session.last_sequence = 0U;
            s_app_v2_session.have_last_ack = false;
            s_app_v2_session.last_activity_us = now_us();
            app_v2_send_hello_ack();
            return;
        }
        if (frame->type == SC_APP_V2_TYPE_HEARTBEAT && frame->length == 8U &&
            frame->payload != NULL) {
            const uint32_t session_id = read_u32_le(&frame->payload[0]);
            const uint32_t sequence = read_u32_le(&frame->payload[4]);
            if (s_app_v2_session.active && session_id == s_app_v2_session.session_id) {
                s_app_v2_session.last_activity_us = now_us();
                app_v2_send_heartbeat_ack(sequence);
            }
            return;
        }
        if (frame->type != SC_APP_V2_TYPE_COMMAND || frame->length < 11U ||
            frame->payload == NULL) {
            return;
        }
        meta.version = SC_APP_FRAME_VERSION_V2;
        meta.session_id = read_u32_le(&frame->payload[0]);
        meta.sequence = read_u32_le(&frame->payload[4]);
        const uint16_t valid_for_ms = read_u16_le(&frame->payload[8]);
        if (!s_app_v2_session.active ||
            meta.session_id != s_app_v2_session.session_id) {
            app_v2_send_command_ack(meta.sequence, frame->payload[10],
                                    SC_APP_V2_RESULT_SESSION_INVALID,
                                    SC_APP_V2_STAGE_GATEWAY_ADMITTED);
            return;
        }
        if (valid_for_ms < 20U || valid_for_ms > 1000U) {
            app_v2_send_command_ack(meta.sequence, frame->payload[10],
                                    SC_APP_V2_RESULT_EXPIRED,
                                    SC_APP_V2_STAGE_GATEWAY_ADMITTED);
            return;
        }
        if (s_app_v2_session.have_sequence &&
            meta.sequence <= s_app_v2_session.last_sequence) {
            if (s_app_v2_session.have_last_ack &&
                meta.sequence == s_app_v2_session.last_ack_sequence) {
                app_v2_send_command_ack(s_app_v2_session.last_ack_sequence,
                                        s_app_v2_session.last_ack_type,
                                        s_app_v2_session.last_ack_result,
                                        s_app_v2_session.last_ack_stage);
            } else {
                app_v2_send_command_ack(meta.sequence, frame->payload[10],
                                        SC_APP_V2_RESULT_STALE_SEQUENCE,
                                        SC_APP_V2_STAGE_GATEWAY_ADMITTED);
            }
            return;
        }
        s_app_v2_session.last_activity_us = now_us();
        s_app_v2_session.last_sequence = meta.sequence;
        s_app_v2_session.have_sequence = true;
        meta.valid_until_us = now_us() + ((uint64_t)valid_for_ms * UINT64_C(1000));
        v2_command.version = SC_APP_FRAME_VERSION;
        v2_command.type = frame->payload[10];
        v2_command.length = (uint16_t)(frame->length - 11U);
        v2_command.payload = &frame->payload[11];
        frame = &v2_command;
    }
#if !SMARTCAR_BMI323_DEBUG_ONLY
    if (frame->type == SC_APP_TYPE_WHEEL_SPEED_CMD &&
        frame->length == SRP_PAYLOAD_WHEEL_SPEED_CMD_SIZE &&
        frame->payload != NULL) {
        float speeds[4] = {0.0f};
        const bool valid = srp_wire_read_f32_array_le(
            frame->payload, frame->length, speeds, 4U);
        if (valid && queue_motion_command(
                         frame->type, SRP_MSG_ID_WHEEL_SPEED_CMD,
                         frame->payload, (uint8_t)frame->length, &meta)) {
            return;
        }
    } else if (frame->type == SC_APP_TYPE_CHASSIS_SPEED_CMD &&
               frame->length == SRP_PAYLOAD_CHASSIS_SPEED_CMD_SIZE &&
               frame->payload != NULL) {
        const float base_speed = srp_wire_read_f32_le(&frame->payload[0]);
        const float target_yaw_rate = srp_wire_read_f32_le(&frame->payload[4]);
        bool reserved_zero = true;

        for (size_t index = 8U;
             index < SRP_PAYLOAD_CHASSIS_SPEED_CMD_SIZE; ++index) {
            if (frame->payload[index] != 0U) {
                reserved_zero = false;
                break;
            }
        }
        if (isfinite(base_speed) && isfinite(target_yaw_rate) && reserved_zero &&
            queue_motion_command(frame->type, SRP_MSG_ID_CHASSIS_SPEED_CMD,
                                 frame->payload, (uint8_t)frame->length, &meta)) {
            return;
        }
    } else if (frame->type == SC_APP_TYPE_CHASSIS_HEADING_CMD &&
               frame->length == SRP_PAYLOAD_CHASSIS_HEADING_CMD_SIZE &&
               frame->payload != NULL) {
        const float target_v_mm_s = srp_wire_read_f32_le(&frame->payload[0]);
        const float target_yaw_deg = srp_wire_read_f32_le(&frame->payload[4]);
        const uint32_t flags = srp_wire_read_u32_le(&frame->payload[8]);

        if (isfinite(target_v_mm_s) && isfinite(target_yaw_deg) &&
            target_yaw_deg >= -180.0f && target_yaw_deg <= 180.0f &&
            flags == SRP_CHASSIS_HEADING_FLAGS_NONE &&
            queue_motion_command(frame->type, SRP_MSG_ID_CHASSIS_HEADING_CMD,
                                 frame->payload, (uint8_t)frame->length, &meta)) {
            return;
        }
    } else if (frame->type == SC_APP_TYPE_WHEEL_SPEED_SINGLE_CMD &&
               frame->length == SRP_PAYLOAD_WHEEL_SPEED_SINGLE_CMD_SIZE &&
               frame->payload != NULL && frame->payload[0] < 4U) {
        const float speed = srp_wire_read_f32_le(&frame->payload[1]);
        if (isfinite(speed) && fabsf(speed) <= 1000.0f &&
            queue_motion_command(frame->type,
                                 SRP_MSG_ID_WHEEL_SPEED_SINGLE_CMD,
                                 frame->payload, (uint8_t)frame->length, &meta)) {
            return;
        }
    } else if (frame->type == SC_APP_TYPE_MASTER_SPEED_CMD &&
               frame->length == SRP_PAYLOAD_MASTER_SPEED_CMD_SIZE &&
               frame->payload != NULL) {
        const float scale = srp_wire_read_f32_le(frame->payload);
        if (isfinite(scale) && scale >= 0.0f && scale <= 4.0f &&
            queue_motion_command(frame->type, SRP_MSG_ID_MASTER_SPEED_CMD,
                                 frame->payload, (uint8_t)frame->length, &meta)) {
            return;
        }
    } else if (frame->type == SC_APP_TYPE_PID_PARAMS_CMD &&
               frame->length == SRP_PAYLOAD_PID_PARAMS_SIZE &&
               frame->payload != NULL) {
        float params[4];
        const bool valid = srp_wire_read_f32_array_le(
            frame->payload, frame->length, params, 4U) &&
            params[0] >= SRP_PID_KP_MIN && params[0] <= SRP_PID_KP_MAX &&
            params[1] >= SRP_PID_KI_MIN && params[1] <= SRP_PID_KI_MAX &&
            params[2] >= SRP_PID_KD_MIN && params[2] <= SRP_PID_KD_MAX &&
            params[3] >= SRP_PID_ACCEL_MIN && params[3] <= SRP_PID_ACCEL_MAX;
        if (valid &&
            (s_pid_tx_context = (smartcar_app_tx_context_t){
                .app_type = frame->type,
                .app_version = meta.version,
                .session_id = meta.session_id,
                .sequence = meta.sequence
            },
            srp_link_send(&s_link, SRP_PRIORITY_COMMAND,
                           SRP_NODE_STM32H757, SRP_MSG_ID_PID_PARAMS_CMD,
                           SRP_FLAG_ACK_REQUIRED, frame->payload,
                           (uint8_t)frame->length,
                           (uint32_t)(now_us() / UINT64_C(1000)),
                           app_transaction_tx_complete,
                           &s_pid_tx_context) == 0)) {
            return;
        }
    } else if (frame->type == SC_APP_TYPE_SYS_CONFIG &&
               frame->length >= 2U && frame->payload != NULL) {
        srp_frame_t config_frame = {
            .priority = SRP_PRIORITY_COMMAND,
            .type = SRP_MSG_ID_SYS_CONFIG,
            .sequence = 0U,
            .flags = SRP_FLAG_TLV | SRP_FLAG_ACK_REQUIRED,
            .length = frame->length,
            .payload = frame->payload,
        };
        uint32_t baud_rate = 0U;
        if (command_bridge_decode_baudrate(&config_frame, &baud_rate) &&
            (s_baud_tx_context = (smartcar_app_tx_context_t){
                .app_type = frame->type,
                .app_version = meta.version,
                .session_id = meta.session_id,
                .sequence = meta.sequence
            },
            s_baud_change_value = baud_rate,
            srp_link_send(&s_link, SRP_PRIORITY_COMMAND, SRP_NODE_STM32H757,
                          SRP_MSG_ID_SYS_CONFIG, SRP_FLAG_TLV | SRP_FLAG_ACK_REQUIRED,
                          frame->payload, frame->length,
                          (uint32_t)(now_us() / UINT64_C(1000)),
                          baud_config_tx_complete, &s_baud_tx_context) == 0)) {
            return;
        }
    } else if (frame->type == SC_APP_TYPE_RADAR_SET_SPEED && frame->length == 1U &&
        frame->payload != NULL && frame->payload[0] <= RADAR_MAX_SPEED &&
        radar_control_set_speed(frame->payload[0])) {
        result = SC_APP_ACK_OK;
    }
#endif
    send_command_result(meta.version, frame->type, meta.session_id,
                        meta.sequence, result,
                        result == SC_APP_ACK_OK ? SC_APP_V2_STAGE_GATEWAY_ADMITTED :
                                                  SC_APP_V2_STAGE_GATEWAY_ADMITTED);
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
    uint8_t payload[SRP_PAYLOAD_RADAR_STATUS_SIZE];

#if !SMARTCAR_BMI323_DEBUG_ONLY
    payload[0] = radar_control_is_running() ? 1U : 0U;
    payload[1] = payload[0] != 0U ? radar_control_get_speed() : 0U;
    (void)srp_link_send(&s_link, SRP_PRIORITY_TELEMETRY,
                          SRP_NODE_STM32H757, SRP_MSG_ID_RADAR_STATUS,
                          SRP_FLAG_STREAM_DATA, payload, sizeof(payload),
                          (uint32_t)(now_us() / UINT64_C(1000)), NULL, NULL);
    (void)notify_app_frame(SC_APP_TYPE_RADAR_STATUS, payload, sizeof(payload));
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
        if (s_app_v2_session.active &&
            now_us() - s_app_v2_session.last_activity_us >=
                ((uint64_t)APP_V2_SESSION_TTL_MS * UINT64_C(1000))) {
            s_app_v2_session.active = false;
            s_motion_pending_target.valid = false;
            s_motion_pending_scale.valid = false;
            s_ble_disconnect_stop_pending = true;
            ESP_LOGW(TAG, "App BLE V2 session expired; stop requested");
        }
        command_bridge_sync_step(now_us());
        command_bridge_log_uart_diag(now_us());
        if (s_ble_disconnect_stop_pending) {
            s_ble_disconnect_stop_pending = false;
            command_bridge_send_zero_wheel_speed();
        }
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
            const bool rx_discontinuity = stm_uart_take_rx_discontinuity();
            const bool break_recovery = stm_uart_take_break_recovery();
            if (break_recovery && s_sync_state != SMARTCAR_SYNC_SYNCED) {
                command_bridge_restart_sync("BREAK_THRESHOLD");
            }
            if (rx_discontinuity) {
                srp_parser_reset(&s_parser);
                const uint64_t reset_now = now_us();
                if (reset_now - s_last_rx_reset_log_us >=
                    ((uint64_t)SMARTCAR_SERVICE_RX_RESET_LOG_PERIOD_MS * UINT64_C(1000))) {
                    s_last_rx_reset_log_us = reset_now;
                    ESP_LOGW(TAG, "UART2 RX discontinuity; SRP parser reset to header seek");
                }
            }
            const int received = stm_uart_receive_nonblock(s_uart_rx_buffer,
                                                            sizeof(s_uart_rx_buffer));
            if (received > 0) {
                (void)srp_parser_feed(&s_parser, s_uart_rx_buffer, (size_t)received);
            }
        }
        srp_link_tick(&s_link, (uint32_t)(now_us() / UINT64_C(1000)));
        command_bridge_check_stm_liveness(now_us());
        if (s_baud_change_pending && now_us() >= s_baud_change_due_us) {
            const uint32_t baud_rate = s_baud_change_value;
            s_baud_change_pending = false;
            if (stm_uart_set_baud_rate(baud_rate) == ESP_OK) {
                ESP_LOGI(TAG, "SRP UART2 baud changed to %lu",
                         (unsigned long)baud_rate);
            } else {
                ESP_LOGE(TAG, "SRP UART2 baud change failed");
            }
        }
        if (s_bus_off_recovery_pending && now_us() >= s_bus_off_recovery_at_us) {
            srp_link_recover(&s_link);
            s_bus_off_recovery_pending = false;
            s_bus_off_latched = false;
            ESP_LOGI(TAG, "SRP link recovered");
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
                     (unsigned)srp_link_get_rec(&s_link),
                     (unsigned)srp_link_get_tec(&s_link));
        }
        vTaskDelay(SMARTCAR_SERVICE_TASK_DELAY_TICKS);
    }
}

esp_err_t smartcar_service_init(void)
{
    const srp_link_config_t link_config = {
        .local_node = SRP_NODE_ESP32_S3,
        .ack_timeout_ms = SRP_LINK_ACK_TIMEOUT_MS,
        .max_retries = SRP_LINK_MAX_RETRIES,
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
    s_ble_disconnect_stop_pending = false;
    s_baud_change_pending = false;
    s_baud_change_value = STM_UART_BAUD_RATE;
    s_baud_change_due_us = 0U;
    s_sync_state = SMARTCAR_SYNC_UART_READY;
    s_sync_attempts = 0U;
    s_sync_tx_sequence = 0U;
    s_next_sync_us = 0U;
    s_last_sync_timeout_notify_us = 0U;
    s_last_stm_frame_us = 0U;
    s_last_uart_diag_us = 0U;
    s_last_rx_reset_log_us = 0U;
    s_last_header_fail_log_us = 0U;
    stm_uart_set_sync_state(false);
    memset(&s_app_v2_session, 0, sizeof(s_app_v2_session));
    s_motion_inflight.valid = false;
    s_motion_pending_target.valid = false;
    s_motion_pending_scale.valid = false;
    s_motion_tx_in_flight = false;
    s_motion_generation = 0U;
    s_stack_hwm_valid = false;
    s_bus_off_recovery_pending = false;
    s_bus_off_latched = false;
    s_ble_rx_queue = xQueueCreateStatic(SMARTCAR_SERVICE_BLE_RX_QUEUE_DEPTH,
                                        sizeof(smartcar_ble_rx_item_t),
                                        s_ble_rx_queue_buffer,
                                        &s_ble_rx_queue_storage);
    if (s_ble_rx_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }
    srp_link_init(&s_link, &link_config);
    srp_parser_init(&s_parser, command_bridge_parsed_frame,
                     command_bridge_on_parser_error, NULL);
    sc_app_parser_init(&s_app_parser, app_command_on_frame, app_command_on_error, NULL);
    s_link_ready = 1U;
#if !SMARTCAR_BMI323_DEBUG_ONLY
    radar_calibration_manager_init();
    radar_calibration_manager_set_transport(command_bridge_send_radar_pwm_ready, NULL);
#endif
    if (s3_ble_set_disconnect_callback(command_bridge_on_ble_disconnect, NULL) != ESP_OK) {
        s_ble_rx_queue = NULL;
        s_link_ready = 0U;
        return ESP_FAIL;
    }
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
