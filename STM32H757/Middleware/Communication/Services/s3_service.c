#include "s3_service.h"

#include <stdbool.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "stm32h7xx.h"

#include "imu_boot_manager.h"
#include "chassis_task.h"
#include "log_service.h"
#include "motor_board_task.h"
#include "srp_link.h"
#include "srp_codec.h"
#include "srp_wire.h"
#include "uart_link.h"

#define S3_SERVICE_STACK_WORDS UINT16_C(1024)
#define S3_SERVICE_PRIORITY (tskIDLE_PRIORITY + 2U)
#define S3_SERVICE_RX_TIMEOUT_MS UINT32_C(2000)
#define S3_SERVICE_BUS_OFF_RECOVERY_MS UINT32_C(100)
#define S3_SERVICE_LINK_LOCK_MS UINT32_C(20)
#define S3_SERVICE_BAUD_SWITCH_GUARD_MS UINT32_C(20)
#define S3_SERVICE_STACK_MONITOR_PERIOD_MS UINT32_C(5000)
#define S3_SERVICE_S3_FRAME_TIMEOUT_MS UINT32_C(200)
#define S3_SERVICE_PERIODIC_STATUS_PERIOD_MS UINT32_C(50)
#define S3_SERVICE_TELEMETRY_LOG_PERIOD_MS UINT32_C(1000)

typedef enum {
    S3_SERVICE_WAIT_FOR_HOST = 0U,
    S3_SERVICE_HOST_SYNCED = 1U
} s3_service_host_state_t;

static srp_parser_t s_parser;
static srp_link_t s_link;
static SemaphoreHandle_t s_link_mutex;
static TaskHandle_t s_s3_service_task_handle;
static UBaseType_t s_s3_service_min_free_words = S3_SERVICE_STACK_WORDS;
static uint8_t s_initialized;
static uint8_t s_bus_off_recovery_pending;
static uint8_t s_bus_off_latched;
static uint32_t s_bus_off_recovery_at_ms;
static uint32_t s_last_rx_time;
static uint8_t s_link_stale_logged;
static uint32_t s_parser_errors;
static uint8_t s_step_bytes[128U];
static char s_line[256U];
static uint32_t s_last_s3_frame_ms;
static uint8_t s_link_timeout_applied;
static volatile s3_service_host_state_t s_host_state;
/* Remains set after a successful sync response so telemetry can continue
 * while the control admission state is revoked by an S3 receive timeout. */
static volatile uint8_t s_telemetry_session_active;
static uint8_t s_last_sync_sequence;
static uint8_t s_baud_change_pending;
static uint32_t s_baud_change_value;
static uint32_t s_baud_change_due_ms;
static uint32_t s_sync_req_decoded_count;
static uint32_t s_sync_req_accepted_count;
static uint32_t s_sync_req_rejected_count;
static uint32_t s_boot_info_attempt_count;
static uint32_t s_boot_info_queued_count;
static uint32_t s_boot_info_enqueue_failures;
static uint32_t s_periodic_status_sent_count;
static uint32_t s_periodic_status_drop_count;
static uint32_t s_frame_timeout_count;
static uint32_t s_bus_off_count;
static uint32_t s_force_stop_success_count;
static uint32_t s_force_stop_failure_count;
static uint32_t s_last_timeout_elapsed_ms;
static uint16_t s_last_timeout_rec;
static uint16_t s_last_timeout_tec;
static s3_service_host_state_t s_last_timeout_sync_state;
static uint32_t s_task_loop_count;
static uint32_t s_periodic_status_attempt_count;
static int s_periodic_status_last_result;
static uint32_t s_task_last_loop_duration_ms;

static bool s3_service_is_motion_message(uint16_t message_id)
{
    return message_id == SRP_MSG_ID_MOTOR_CMD ||
           message_id == SRP_MSG_ID_WHEEL_SPEED_SINGLE_CMD ||
           message_id == SRP_MSG_ID_MASTER_SPEED_CMD ||
           message_id == SRP_MSG_ID_CHASSIS_SPEED_CMD ||
           message_id == SRP_MSG_ID_CHASSIS_HEADING_CMD;
}

static bool s3_service_is_telemetry_message(uint16_t message_id)
{
    switch (message_id) {
    case SRP_MSG_ID_IMU_TELEMETRY:
    case SRP_MSG_ID_ATTITUDE:
    case SRP_MSG_ID_IMU_CAL_STATUS:
    case SRP_MSG_ID_POWER_STATUS:
    case SRP_MSG_ID_WHEEL_SPEED_STATUS:
    case SRP_MSG_ID_CHASSIS_STATE:
    case SRP_MSG_ID_WHEEL_CONTROL_STATUS:
    case SRP_MSG_ID_RADAR_STATUS:
        return true;
    default:
        return false;
    }
}

static bool s3_service_telemetry_session_is_available(void)
{
    if (s_telemetry_session_active == 0U) {
        return false;
    }
    if (uart_link_is_ready() == 0U ||
        srp_link_get_state(&s_link) == SRP_LINK_BUS_OFF) {
        s_telemetry_session_active = 0U;
        return false;
    }
    return true;
}

static bool s3_service_decode_baudrate(const srp_frame_t *frame,
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

static uint8_t s3_service_link_lock(void)
{
    return s_link_mutex != NULL &&
           xSemaphoreTake(s_link_mutex, pdMS_TO_TICKS(S3_SERVICE_LINK_LOCK_MS)) == pdTRUE;
}

static void s3_service_link_unlock(void)
{
    if (s_link_mutex != NULL) {
        (void)xSemaphoreGive(s_link_mutex);
    }
}

static int s3_service_transport_send(const uint8_t *data, uint16_t length,
                                     void *context)
{
    (void)context;
    return uart_link_send(data, length) == HAL_OK ? 0 : -1;
}

static void s3_service_on_bus_off(void *context)
{
    bool force_stop_ok;

    (void)context;
    if (s_bus_off_latched != 0U) {
        return;
    }
    s_bus_off_latched = 1U;
    ++s_bus_off_count;
    s_link_timeout_applied = 1U;
    s_host_state = S3_SERVICE_WAIT_FOR_HOST;
    s_telemetry_session_active = 0U;
    chassis_task_force_stop();
    force_stop_ok = motor_board_force_stop();
    if (force_stop_ok) {
        ++s_force_stop_success_count;
    } else {
        ++s_force_stop_failure_count;
        LOG_ERROR("SRP BUS_OFF forced stop PWM queue drop\r\n");
    }
    s_bus_off_recovery_pending = 1U;
    s_bus_off_recovery_at_ms = HAL_GetTick() + S3_SERVICE_BUS_OFF_RECOVERY_MS;
    uart_link_recover();
    LOG_ERROR("SRP BUS_OFF; UART2 recovery requested\r\n");
}

static int s3_service_send_locked(uint8_t priority, uint16_t message_id,
                                  uint8_t flags, const uint8_t *payload,
                                  uint8_t length);

static int s3_service_send_boot_info(uint8_t request_sequence, uint8_t verbose)
{
    const uint8_t payload[SRP_PAYLOAD_RSP_BOOT_INFO_SIZE] = {
        SRP_PROTOCOL_VERSION_MAJOR,
        SRP_PROTOCOL_VERSION_MINOR,
        (uint8_t)SRP_STM_STATE_HOST_SYNCED,
        SRP_SYNC_FLAG_VERSION_OK,
        request_sequence,
        0U, 0U, 0U
    };

    int result;

    ++s_boot_info_attempt_count;
    /* This is the admission packet itself.  It must not depend on
     * s_host_state/s3_service_is_synced(), otherwise a closed session can
     * never be opened by the packet that opens it.  Keep the call below on
     * the SRP encoder and the physical UART transport, but deliberately skip
     * the ordinary business-message gate. */
    /* s3_service_send_boot_info() is called from the parser callback while
     * s3_service_step() already owns s_link_mutex.  Do not take the
     * non-recursive mutex again here: the outer lock serializes this direct
     * admission response with all other SRP producers. */
    result = srp_link_send(&s_link, SRP_PRIORITY_COMMAND,
                           SRP_NODE_ESP32_S3, SRP_MSG_ID_RSP_BOOT_INFO,
                           SRP_FLAG_STREAM_DATA, payload, sizeof(payload),
                           HAL_GetTick(), NULL, NULL);
    uart_link_stats_t stats;
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    uart_link_get_stats(&stats);
    if (result != 0) {
        ++s_boot_info_enqueue_failures;
        (void)snprintf(line, sizeof(line),
                       "[SRP] RSP_BOOT_INFO sent, status=%d\r\n", result);
        LOG_WARN(line);
        (void)snprintf(line, sizeof(line),
                       "BOOT_INFO TX_FAIL r=%d ready=%u state=0x%08lX "
                       "err=0x%08lX hal=%lu\r\n",
                       result, (unsigned)uart_link_is_ready(),
                       (unsigned long)stats.uart_tx_gstate,
                       (unsigned long)stats.uart_tx_error_code,
                       (unsigned long)stats.uart_hal_error);
        LOG_WARN(line);
    } else {
        ++s_boot_info_queued_count;
        if (verbose != 0U) {
            (void)snprintf(line, sizeof(line),
                           "[SRP] RSP_BOOT_INFO sent, status=%d\r\n", result);
            LOG_DEBUG(line);
        }
        if (verbose != 0U) {
            (void)snprintf(line, sizeof(line),
                           "SRP BOOT_INFO sent seq=%u tx=%lu dma_start=%lu\r\n",
                           (unsigned)request_sequence,
                           (unsigned long)stats.uart_tx_count,
                           (unsigned long)stats.uart_tx_dma_starts);
            LOG_INFO(line);
        }
    }
    return result;
}

static bool s3_service_handle_sync_req(const srp_frame_t *frame)
{
    const bool valid = frame != NULL &&
                       frame->length == SRP_PAYLOAD_CMD_SYNC_REQ_SIZE &&
                       frame->payload != NULL &&
                       frame->payload[0] == SRP_PROTOCOL_VERSION_MAJOR &&
                       frame->payload[1] == SRP_PROTOCOL_VERSION_MINOR &&
                       frame->payload[2] == 0U && frame->payload[3] == 0U;

    if (frame == NULL) {
        return false;
    }
    ++s_sync_req_decoded_count;
    if (!valid) {
        ++s_sync_req_rejected_count;
        (void)snprintf(s_line, sizeof(s_line),
                       "SRP SYNC_REQ rejected seq=%u len=%u payload=%02X %02X %02X %02X\r\n",
                       (unsigned)frame->sequence, (unsigned)frame->length,
                       frame->payload == NULL ? 0U : frame->payload[0],
                       frame->payload == NULL || frame->length < 2U ? 0U : frame->payload[1],
                       frame->payload == NULL || frame->length < 3U ? 0U : frame->payload[2],
                       frame->payload == NULL || frame->length < 4U ? 0U : frame->payload[3]);
        LOG_WARN(s_line);
        return false;
    }

    const uint8_t was_synced = s_host_state == S3_SERVICE_HOST_SYNCED ? 1U : 0U;
    const uint32_t ipsr = __get_IPSR();
    const BaseType_t scheduler_state = ipsr == 0U
                                            ? xTaskGetSchedulerState()
                                            : taskSCHEDULER_NOT_STARTED;

    s_last_sync_sequence = frame->sequence;
    if (was_synced == 0U) {
        (void)snprintf(s_line, sizeof(s_line),
                       "SRP_SYNC_CONTEXT ipsr=%lu sched=%ld tx=task-path\r\n",
                       (unsigned long)ipsr, (long)scheduler_state);
        LOG_INFO(s_line);
        (void)snprintf(s_line, sizeof(s_line),
                       "[SRP] CMD_SYNC_REQ received, seq=%lu\r\n",
                       (unsigned long)frame->sequence);
        LOG_DEBUG(s_line);
    }
    /* CMD_SYNC_REQ is the sole admission path while waiting. Send the
     * response first; publish HOST_SYNCED only after physical TX accepted it. */
    if (s3_service_send_boot_info(s_last_sync_sequence,
                                  was_synced == 0U ? 1U : 0U) != 0) {
        s_host_state = S3_SERVICE_WAIT_FOR_HOST;
        ++s_sync_req_rejected_count;
        LOG_WARN("SRP CMD_SYNC_REQ response enqueue failed; staying WAIT_FOR_HOST\r\n");
        return false;
    }
    s_host_state = S3_SERVICE_HOST_SYNCED;
    s_last_s3_frame_ms = HAL_GetTick();
    s_link_timeout_applied = 0U;
    s_bus_off_latched = 0U;
    s_telemetry_session_active = 1U;
    ++s_sync_req_accepted_count;
    if (was_synced == 0U) {
        (void)snprintf(s_line, sizeof(s_line),
                       "SRP CMD_SYNC_REQ decoded seq=%u len=%u; RSP_BOOT_INFO attempt=%lu\r\n",
                       (unsigned)frame->sequence, (unsigned)frame->length,
                       (unsigned long)s_boot_info_attempt_count);
        LOG_INFO(s_line);
    }
    return true;
}

static int s3_service_send_locked(uint8_t priority, uint16_t message_id,
                                  uint8_t flags, const uint8_t *payload,
                                  uint8_t length)
{
    const bool is_telemetry = s3_service_is_telemetry_message(message_id);

    /* CM7 is silent on USART2 until S3's valid CMD_SYNC_REQ is accepted. A
     * previously established session may continue telemetry after a control
     * timeout, but ordinary command/log traffic remains gated. */
    if (is_telemetry) {
        if (!s3_service_telemetry_session_is_available()) {
            return -1;
        }
    } else if (s_host_state != S3_SERVICE_HOST_SYNCED) {
        return -1;
    }
    return srp_link_send(&s_link, priority, SRP_NODE_ESP32_S3, message_id,
                          flags, payload, length, HAL_GetTick(), NULL, NULL);
}

static void s3_service_send_response_locked(const srp_frame_t *request,
                                            uint8_t is_error,
                                            uint8_t status_code)
{
    if (request == NULL) {
        return;
    }
    (void)srp_link_send_fast_response(
        &s_link, SRP_PRIORITY_COMMAND, 0U, is_error,
        request->type, request->sequence, status_code, HAL_GetTick());
}

static void s3_service_on_frame(const srp_frame_t *frame, void *context)
{
    const uint8_t ack_required = frame == NULL ? 0U :
        (uint8_t)((frame->flags & SRP_FLAG_ACK_REQUIRED) != 0U);

    (void)context;
    if (frame == NULL) {
        return;
    }
    const uint8_t message_id = frame->type;
    if (message_id == SRP_MSG_ID_CMD_SYNC_REQ) {
        (void)s3_service_handle_sync_req(frame);
        return;
    }
    if (srp_link_get_state(&s_link) == SRP_LINK_BUS_OFF) {
        return;
    }
    if (s_host_state != S3_SERVICE_HOST_SYNCED &&
        s3_service_is_motion_message(message_id)) {
        LOG_WARN("SRP motion dropped before CMD_SYNC_REQ\r\n");
        if (ack_required != 0U) {
            s3_service_send_response_locked(frame, 1U, SRP_FAST_RESP_BUSY);
        }
        return;
    }
    if (message_id == SRP_MSG_ID_RADAR_PWM_READY) {
        uint8_t admitted = 0U;
        if (frame->length == SRP_PAYLOAD_RADAR_PWM_READY_SIZE &&
            frame->payload != NULL) {
            admitted = imu_boot_manager_on_radar_pwm_ready(frame->payload[0]);
        }
        if (ack_required != 0U) {
            s3_service_send_response_locked(
                frame, admitted == 0U, admitted != 0U ? SRP_FAST_RESP_OK :
                                                   SRP_FAST_RESP_BUSY);
        }
        return;
    }

    if (message_id == SRP_MSG_ID_SYS_CONFIG) {
        uint32_t baud_rate = 0U;
        const bool valid = s3_service_decode_baudrate(frame, &baud_rate);

        if (ack_required != 0U) {
            s3_service_send_response_locked(
                frame, valid ? 0U : 1U,
                valid ? SRP_FAST_RESP_OK : SRP_FAST_RESP_INVALID_PARAM);
        }
        if (valid) {
            s_baud_change_pending = 1U;
            s_baud_change_value = baud_rate;
            s_baud_change_due_ms = HAL_GetTick() +
                                   S3_SERVICE_BAUD_SWITCH_GUARD_MS;
        }
        return;
    }

    if (message_id == SRP_MSG_ID_PID_PARAMS_CMD) {
        float params[4];
        const bool valid = frame->length == SRP_PAYLOAD_PID_PARAMS_SIZE &&
                           frame->payload != NULL &&
                           srp_wire_read_f32_array_le(frame->payload,
                                                       frame->length,
                                                       params, 4U) &&
                           motor_board_update_pid_params(params[0], params[1],
                                                         params[2], params[3]);
        if (valid) {
            (void)snprintf(s_line, sizeof(s_line),
                           "[PID_CONFIG] Updated: Kp=%.3f, Ki=%.3f, Kd=%.3f, Accel=%.1f\r\n",
                           (double)params[0], (double)params[1],
                           (double)params[2], (double)params[3]);
            LOG_INFO(s_line);
        }
        if (ack_required != 0U) {
            s3_service_send_response_locked(
                frame, valid ? 0U : 1U,
                valid ? SRP_FAST_RESP_OK : SRP_FAST_RESP_INVALID_PARAM);
        }
        return;
    }

    if (message_id == SRP_MSG_ID_WHEEL_SPEED_CMD) {
        float speeds[4];
        bool all_zero = true;
        bool valid = frame->length == SRP_PAYLOAD_WHEEL_SPEED_CMD_SIZE &&
                     frame->payload != NULL &&
                     srp_wire_read_f32_array_le(frame->payload,
                                                 frame->length, speeds, 4U);

        if (valid) {
            for (size_t index = 0U; index < 4U; ++index) {
                all_zero = all_zero && speeds[index] == 0.0f;
            }
            /* A zero wheel tuple is the cross-boundary stop primitive. It
             * must also clear an active heading target before MotorBoard is
             * allowed to observe the stop, so the 10 ms chassis task cannot
             * reapply a stale cruise command. */
            if (all_zero) {
                chassis_task_force_stop();
            }
            valid = motor_board_set_target_wheel_speeds(speeds);
        }
        if (ack_required != 0U) {
            s3_service_send_response_locked(
                frame, valid ? 0U : 1U,
                valid ? SRP_FAST_RESP_OK : SRP_FAST_RESP_INVALID_PARAM);
        }
        return;
    }

    if (message_id == SRP_MSG_ID_CHASSIS_SPEED_CMD) {
        float linear_mm_s = 0.0f;
        float angular_rad_s = 0.0f;
        bool valid = frame->length == SRP_PAYLOAD_CHASSIS_SPEED_CMD_SIZE &&
                     frame->payload != NULL;

        if (valid) {
            linear_mm_s = srp_wire_read_f32_le(&frame->payload[0]);
            angular_rad_s = srp_wire_read_f32_le(&frame->payload[4]);
            for (size_t index = 8U;
                 index < SRP_PAYLOAD_CHASSIS_SPEED_CMD_SIZE; ++index) {
                if (frame->payload[index] != 0U) {
                    valid = false;
                    break;
                }
            }
            valid = valid && isfinite(linear_mm_s) &&
                    isfinite(angular_rad_s);
        }
        if (valid) {
            (void)snprintf(s_line, sizeof(s_line),
                           "[CHASSIS_CMD] v=%.2f, w=%.4f",
                           (double)linear_mm_s, (double)angular_rad_s);
            LOG_INFO(s_line);
            valid = chassis_task_set_velocity(linear_mm_s, angular_rad_s);
        }
        if (ack_required != 0U) {
            s3_service_send_response_locked(
                frame, valid ? 0U : 1U,
                valid ? SRP_FAST_RESP_OK : SRP_FAST_RESP_INVALID_PARAM);
        }
        return;
    }

    if (message_id == SRP_MSG_ID_CHASSIS_HEADING_CMD) {
        float target_v_mm_s = 0.0f;
        float target_yaw_deg = 0.0f;
        uint32_t flags = UINT32_MAX;
        bool valid = frame->length == SRP_PAYLOAD_CHASSIS_HEADING_CMD_SIZE &&
                     frame->payload != NULL;

        if (valid) {
            target_v_mm_s = srp_wire_read_f32_le(&frame->payload[0]);
            target_yaw_deg = srp_wire_read_f32_le(&frame->payload[4]);
            flags = srp_wire_read_u32_le(&frame->payload[8]);
            valid = isfinite(target_v_mm_s) && isfinite(target_yaw_deg) &&
                    target_yaw_deg >= -180.0f && target_yaw_deg <= 180.0f &&
                    flags == SRP_CHASSIS_HEADING_FLAGS_NONE &&
                    chassis_task_heading_target_is_admissible(target_v_mm_s,
                                                              target_yaw_deg);
        }
        if (valid) {
            (void)snprintf(s_line, sizeof(s_line),
                           "[CHASSIS_HEADING_CMD] v=%.2f, target_yaw=%.2f",
                           (double)target_v_mm_s, (double)target_yaw_deg);
            LOG_INFO(s_line);
            chassis_task_set_heading_target(target_v_mm_s, target_yaw_deg);
        }
        if (ack_required != 0U) {
            s3_service_send_response_locked(
                frame, valid ? 0U : 1U,
                valid ? SRP_FAST_RESP_OK : SRP_FAST_RESP_INVALID_PARAM);
        }
        return;
    }

    if (ack_required != 0U) {
        s3_service_send_response_locked(frame, 1U, SRP_FAST_RESP_INVALID_PARAM);
    }
}

static void s3_service_on_parsed_frame(const srp_frame_t *frame, void *context)
{
    (void)context;
    if (frame != NULL) {
        if (srp_link_get_state(&s_link) != SRP_LINK_BUS_OFF &&
            s_host_state == S3_SERVICE_HOST_SYNCED) {
            s_last_s3_frame_ms = HAL_GetTick();
            s_link_timeout_applied = 0U;
        }
    }
    srp_link_receive(&s_link, frame);
}

static void s3_service_on_parser_error(srp_parser_error_t error,
                                       const uint8_t *data, size_t length,
                                       void *context)
{
    const char *reason = "UNKNOWN";
    (void)data;
    (void)context;

    ++s_parser_errors;
    /* Bytes discarded while seeking AA 55 are expected on a restarted UART
     * stream. Keep them observable without turning line noise into REC/BUS_OFF. */
    if (error != SRP_PARSER_ERROR_MAGIC) {
        srp_link_report_parser_error(&s_link, error);
    }
    if (error == SRP_PARSER_ERROR_MAGIC) {
        return;
    }
    switch (error) {
    case SRP_PARSER_ERROR_HEADER: reason = "HEADER"; break;
    case SRP_PARSER_ERROR_CRC: reason = "CRC"; break;
    case SRP_PARSER_ERROR_EOF: reason = "EOF"; break;
    default: break;
    }
    (void)snprintf(s_line, sizeof(s_line),
                   "SRP_RX_ERROR reason=%s bytes=%lu rec=%u tec=%u\r\n",
                   reason, (unsigned long)length,
                   (unsigned)srp_link_get_rec(&s_link),
                   (unsigned)srp_link_get_tec(&s_link));
    LOG_WARN(s_line);
}

static void s3_service_log_stack(void)
{
    uart_link_stats_t uart_stats;
    const UBaseType_t free_words = s_s3_service_task_handle == NULL
                                        ? 0U
                                        : uxTaskGetStackHighWaterMark(
                                              s_s3_service_task_handle);

    uart_link_get_stats(&uart_stats);
    if (free_words < s_s3_service_min_free_words) {
        s_s3_service_min_free_words = free_words;
    }
    (void)snprintf(s_line, sizeof(s_line),
                   "[S3_TASK_STACK] free_words=%lu min_free_words=%lu\r\n",
                   (unsigned long)free_words,
                   (unsigned long)s_s3_service_min_free_words);
    LOG_INFO(s_line);
    (void)snprintf(s_line, sizeof(s_line),
                   "SRP_S3 h=%u f=%lu pe=%lu sy=%lu/%lu/%lu "
                   "bi=%lu/%lu/%lu rec=%u tec=%u "
                   "periodic=%lu/%lu "
                   "timeout=%lu bus_off=%lu force_stop=%lu/%lu "
                   "last_timeout=%lu/%u/%u/%u "
                   "uart_rx=%lu ev=%lu rearm=%lu/%lu tx=%lu/%lu "
                   "err=%lu/%lu buf=%u\r\n",
                   (unsigned)s_host_state,
                   (unsigned long)s_parser.frame_count,
                   (unsigned long)s_parser_errors,
                   (unsigned long)s_sync_req_decoded_count,
                   (unsigned long)s_sync_req_accepted_count,
                   (unsigned long)s_sync_req_rejected_count,
                   (unsigned long)s_boot_info_attempt_count,
                   (unsigned long)s_boot_info_queued_count,
                   (unsigned long)s_boot_info_enqueue_failures,
                   (unsigned)srp_link_get_rec(&s_link),
                   (unsigned)srp_link_get_tec(&s_link),
                   (unsigned long)s_periodic_status_sent_count,
                   (unsigned long)s_periodic_status_drop_count,
                   (unsigned long)s_frame_timeout_count,
                   (unsigned long)s_bus_off_count,
                   (unsigned long)s_force_stop_success_count,
                   (unsigned long)s_force_stop_failure_count,
                   (unsigned long)s_last_timeout_elapsed_ms,
                   (unsigned)s_last_timeout_rec,
                   (unsigned)s_last_timeout_tec,
                   (unsigned)s_last_timeout_sync_state,
                   (unsigned long)uart_stats.uart_rx_bytes,
                   (unsigned long)uart_stats.uart_rx_events,
                   (unsigned long)uart_stats.uart_rx_rearms,
                   (unsigned long)uart_stats.uart_rx_rearm_failures,
                   (unsigned long)uart_stats.uart_tx_count,
                   (unsigned long)uart_stats.uart_tx_dma_starts,
                   (unsigned long)uart_stats.uart_usart_errors,
                   (unsigned long)uart_stats.uart_hal_error,
                   (unsigned)uart_stats.rx_buffered);
    LOG_INFO(s_line);
    (void)snprintf(s_line, sizeof(s_line),
                   "SRP_TELEMETRY diag session=%u loop=%lu periodic_try=%lu "
                   "last_result=%d last_loop_ms=%lu\r\n",
                   (unsigned)s_telemetry_session_active,
                   (unsigned long)s_task_loop_count,
                   (unsigned long)s_periodic_status_attempt_count,
                   s_periodic_status_last_result,
                   (unsigned long)s_task_last_loop_duration_ms);
    LOG_INFO(s_line);
    (void)snprintf(s_line, sizeof(s_line),
                   "[SRP_UART2_DIAG] rx_bytes=%lu, tx_bytes=%lu, state=%d\r\n",
                   (unsigned long)uart_stats.uart_rx_bytes,
                   (unsigned long)uart_stats.uart_tx_bytes,
                   (int)s_host_state);
    LOG_DEBUG(s_line);
}

void s3_service_init(void)
{
    const srp_link_config_t config = {
        .local_node = SRP_NODE_STM32H757,
        .ack_timeout_ms = SRP_LINK_ACK_TIMEOUT_MS,
        .max_retries = SRP_LINK_MAX_RETRIES,
        .transport_send = s3_service_transport_send,
        .on_frame = s3_service_on_frame,
        .on_bus_off = s3_service_on_bus_off,
        .context = NULL,
    };

    if (s_initialized != 0U) {
        return;
    }
    s_link_mutex = xSemaphoreCreateMutex();
    if (s_link_mutex == NULL) {
        LOG_ERROR("SRP link mutex allocation failed\r\n");
        return;
    }
    srp_link_init(&s_link, &config);
    srp_parser_init(&s_parser, s3_service_on_parsed_frame, s3_service_on_parser_error,
                     NULL);
    s_bus_off_recovery_pending = 0U;
    s_bus_off_latched = 0U;
    s_bus_off_recovery_at_ms = 0U;
    s_last_rx_time = 0U;
    s_last_s3_frame_ms = HAL_GetTick();
    s_link_timeout_applied = 0U;
    s_host_state = S3_SERVICE_WAIT_FOR_HOST;
    s_telemetry_session_active = 0U;
    s_last_sync_sequence = 0U;
    s_baud_change_pending = 0U;
    s_baud_change_value = UART_LINK_BAUD_RATE;
    s_baud_change_due_ms = 0U;
    s_link_stale_logged = 0U;
    s_parser_errors = 0U;
    s_sync_req_decoded_count = 0U;
    s_sync_req_accepted_count = 0U;
    s_sync_req_rejected_count = 0U;
    s_boot_info_attempt_count = 0U;
    s_boot_info_queued_count = 0U;
    s_boot_info_enqueue_failures = 0U;
    s_periodic_status_sent_count = 0U;
    s_periodic_status_drop_count = 0U;
    s_frame_timeout_count = 0U;
    s_bus_off_count = 0U;
    s_force_stop_success_count = 0U;
    s_force_stop_failure_count = 0U;
    s_last_timeout_elapsed_ms = 0U;
    s_last_timeout_rec = 0U;
    s_last_timeout_tec = 0U;
    s_last_timeout_sync_state = S3_SERVICE_WAIT_FOR_HOST;
    s_task_loop_count = 0U;
    s_periodic_status_attempt_count = 0U;
    s_periodic_status_last_result = -1;
    s_task_last_loop_duration_ms = 0U;
    s_s3_service_min_free_words = S3_SERVICE_STACK_WORDS;
    s_initialized = 1U;
    imu_boot_manager_set_transport(s3_service_send_boot_message);
}

int s3_service_send_message(uint8_t priority, uint16_t message_id, uint8_t flags,
                            const uint8_t *payload, uint8_t length)
{
    int result;

    if (s_initialized == 0U || s3_service_link_lock() == 0U) {
        return -1;
    }
    result = s3_service_send_locked(priority, message_id, flags, payload, length);
    s3_service_link_unlock();
    return result;
}

static void s3_service_send_periodic_status(void)
{
    uint8_t payload[SRP_PAYLOAD_IMU_CAL_STATUS_SIZE] = {0};
    imu_boot_status_t imu_status = {0};
    uint8_t stage = SRP_IMU_CAL_STAGE_WAIT_RADAR_READY;
    int result;

    ++s_periodic_status_attempt_count;
    if (!s3_service_telemetry_session_is_available()) {
        s_periodic_status_last_result = -1;
        ++s_periodic_status_drop_count;
        return;
    }
    /* This is a transport heartbeat, not the calibration authority. Read the
     * lifecycle snapshot without taking the calibration mutex in this task;
     * the detailed producer remains the IMU worker when it is available. */
    imu_boot_manager_get_status(&imu_status);
    if (imu_status.phase == IMU_PHASE_FAILED) {
        stage = SRP_IMU_CAL_STAGE_ERROR;
    } else if (imu_status.phase == IMU_PHASE_READY) {
        stage = SRP_IMU_CAL_STAGE_COMPLETE;
    }
    payload[0] = stage;
    /* Baseline IMU status keeps byte 1 reserved; calibration authority owns
     * the detailed status producer and emits the same 11-byte layout. */
    payload[1] = 0U;
    srp_wire_write_u32_le(&payload[2], imu_status.sample_count);
    srp_wire_write_u32_le(&payload[6], imu_status.sample_total);
    payload[10] = imu_status.error;
    result = s3_service_send_message(SRP_PRIORITY_TELEMETRY,
                                     SRP_MSG_ID_IMU_CAL_STATUS,
                                     SRP_FLAG_STREAM_DATA, payload,
                                     (uint8_t)sizeof(payload));
    s_periodic_status_last_result = result;
    if (result == 0) {
        ++s_periodic_status_sent_count;
    } else {
        ++s_periodic_status_drop_count;
    }
}

void s3_service_send_boot_message(uint16_t message_id, uint8_t flags,
                                  const uint8_t *payload, uint8_t length)
{
    (void)s3_service_send_message(SRP_PRIORITY_COMMAND, message_id, flags,
                                  payload, length);
}

void s3_service_send_imu_telemetry(const uint8_t *payload, uint8_t length)
{
    if (payload == NULL || length != SRP_PAYLOAD_IMU_TELEMETRY_SIZE) {
        return;
    }
    (void)s3_service_send_message(SRP_PRIORITY_TELEMETRY,
                                  SRP_MSG_ID_IMU_TELEMETRY,
                                  SRP_FLAG_STREAM_DATA, payload, length);
}

void s3_service_send_dual_attitude(const uint8_t *payload, uint8_t length)
{
    if (payload == NULL || length != SRP_PAYLOAD_DUAL_AHRS_SIZE ||
        payload[0] != SRP_DUAL_AHRS_SCHEMA || payload[2] != 0U ||
        payload[3] != 0U) {
        return;
    }
    (void)s3_service_send_message(SRP_PRIORITY_COMMAND,
                                  SRP_MSG_ID_ATTITUDE,
                                  SRP_FLAG_STREAM_DATA, payload, length);
}

void s3_service_send_chassis_state(const uint8_t *payload, uint8_t length)
{
    if (payload == NULL || length != SRP_PAYLOAD_CHASSIS_STATE_SIZE ||
        payload[0] != SRP_CHASSIS_STATE_SCHEMA || payload[2] != 0U ||
        payload[3] != 0U) {
        return;
    }
    (void)s3_service_send_message(SRP_PRIORITY_TELEMETRY,
                                  SRP_MSG_ID_CHASSIS_STATE,
                                  SRP_FLAG_STREAM_DATA, payload, length);
}

void s3_service_send_wheel_control_status(const uint8_t *payload, uint8_t length)
{
    if (payload == NULL || length != SRP_PAYLOAD_WHEEL_CONTROL_STATUS_SIZE ||
        payload[0] != SRP_WHEEL_CONTROL_STATUS_SCHEMA ||
        (payload[1] != SRP_CHASSIS_MODE_DIFF &&
         payload[1] != SRP_CHASSIS_MODE_WHEEL_INDEPENDENT) ||
        payload[2] != 0U || payload[3] != 0U) {
        return;
    }
    (void)s3_service_send_message(SRP_PRIORITY_TELEMETRY,
                                  SRP_MSG_ID_WHEEL_CONTROL_STATUS,
                                  SRP_FLAG_STREAM_DATA, payload, length);
}

int s3_service_send_log(const uint8_t *payload, uint8_t length)
{
    return s3_service_send_message(SRP_PRIORITY_LOG, SRP_MSG_ID_LOG,
                                   SRP_FLAG_STREAM_DATA, payload, length);
}

void s3_service_step(void)
{
    const size_t length = uart_link_read(s_step_bytes, sizeof(s_step_bytes));
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t latest_rx_time = uart_link_get_last_rx_time();

    if (s_initialized == 0U) {
        return;
    }
    if (s_telemetry_session_active != 0U &&
        (uart_link_is_ready() == 0U ||
         srp_link_get_state(&s_link) == SRP_LINK_BUS_OFF)) {
        s_telemetry_session_active = 0U;
    }
    if (s_link_timeout_applied == 0U &&
        s_host_state == S3_SERVICE_HOST_SYNCED &&
        (uint32_t)(now_ms - s_last_s3_frame_ms) >=
            S3_SERVICE_S3_FRAME_TIMEOUT_MS) {
        bool force_stop_ok;

        ++s_frame_timeout_count;
        s_last_timeout_elapsed_ms = now_ms - s_last_s3_frame_ms;
        s_last_timeout_rec = srp_link_get_rec(&s_link);
        s_last_timeout_tec = srp_link_get_tec(&s_link);
        s_last_timeout_sync_state = s_host_state;
        /* Revoke motion/control admission only. A previously established
         * telemetry session remains eligible for the periodic heartbeat. */
        s_host_state = S3_SERVICE_WAIT_FOR_HOST;
        s_link_timeout_applied = 1U;
        chassis_task_force_stop();
        force_stop_ok = motor_board_force_stop();
        if (force_stop_ok) {
            ++s_force_stop_success_count;
            LOG_WARN("SRP S3 link timeout; PID reset and PWM stopped\r\n");
        } else {
            ++s_force_stop_failure_count;
            LOG_ERROR("SRP S3 link timeout; PWM stop queue drop\r\n");
        }
    }
    if (s3_service_link_lock() != 0U) {
        if (length != 0U) {
            (void)srp_parser_feed(&s_parser, s_step_bytes, length);
        }
        srp_link_tick(&s_link, now_ms);
        if (s_bus_off_recovery_pending != 0U &&
            (uint32_t)(now_ms - s_bus_off_recovery_at_ms) < UINT32_C(0x80000000)) {
            srp_link_recover(&s_link);
            s_bus_off_recovery_pending = 0U;
            s_bus_off_latched = 0U;
            LOG_INFO("SRP link recovered after UART2 reset\r\n");
        }
        s3_service_link_unlock();
    }

    if (s_baud_change_pending != 0U &&
        (uint32_t)(now_ms - s_baud_change_due_ms) < UINT32_C(0x80000000)) {
        const uint32_t baud_rate = s_baud_change_value;
        s_baud_change_pending = 0U;
        if (uart_link_set_baud_rate(baud_rate) == HAL_OK) {
            (void)snprintf(s_line, sizeof(s_line),
                           "SRP UART2 baud changed to %lu\r\n",
                           (unsigned long)baud_rate);
            LOG_INFO(s_line);
        } else {
            LOG_ERROR("SRP UART2 baud change failed\r\n");
        }
    }

    if (latest_rx_time != 0U && latest_rx_time != s_last_rx_time) {
        s_last_rx_time = latest_rx_time;
        s_link_stale_logged = 0U;
    }
    if (s_last_rx_time != 0U &&
        (uint32_t)(now_ms - s_last_rx_time) >= S3_SERVICE_RX_TIMEOUT_MS &&
        s_link_stale_logged == 0U) {
        LOG_WARN("SRP UART2 receive stale\r\n");
        s_link_stale_logged = 1U;
    }
}

void s3_service_task(void *argument)
{
    TickType_t last_wake;
    uint32_t last_stack_monitor_ms;
    uint32_t last_periodic_status_ms;
    uint32_t last_telemetry_log_ms;

    (void)argument;
    last_wake = xTaskGetTickCount();
    last_stack_monitor_ms = HAL_GetTick();
    last_periodic_status_ms = last_stack_monitor_ms;
    last_telemetry_log_ms = last_stack_monitor_ms;
    (void)snprintf(s_line, sizeof(s_line),
                   "SRP_TELEMETRY task=running period_ms=%u\r\n",
                   (unsigned)S3_SERVICE_PERIODIC_STATUS_PERIOD_MS);
    LOG_INFO(s_line);
    for (;;) {
        const uint32_t loop_start_ms = HAL_GetTick();
        const uint32_t now_ms = loop_start_ms;

        ++s_task_loop_count;

        s3_service_step();
        if ((uint32_t)(now_ms - last_periodic_status_ms) >=
            S3_SERVICE_PERIODIC_STATUS_PERIOD_MS) {
            last_periodic_status_ms = now_ms;
            s3_service_send_periodic_status();
        }
        if ((uint32_t)(now_ms - last_telemetry_log_ms) >=
            S3_SERVICE_TELEMETRY_LOG_PERIOD_MS) {
            uart_link_stats_t uart_stats;

            last_telemetry_log_ms = now_ms;
            uart_link_get_stats(&uart_stats);
            (void)snprintf(s_line, sizeof(s_line),
                           "SRP_TELEMETRY task=alive sync=%u sent=%lu drop=%lu tx=%lu\r\n",
                           (unsigned)s_host_state,
                           (unsigned long)s_periodic_status_sent_count,
                           (unsigned long)s_periodic_status_drop_count,
                           (unsigned long)uart_stats.uart_tx_count);
            LOG_INFO(s_line);
        }
        if ((uint32_t)(HAL_GetTick() - last_stack_monitor_ms) >=
            S3_SERVICE_STACK_MONITOR_PERIOD_MS) {
            last_stack_monitor_ms = HAL_GetTick();
            s3_service_log_stack();
        }
        s_task_last_loop_duration_ms = HAL_GetTick() - loop_start_ms;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1U));
    }
}

void s3_service_start(void)
{
    s3_service_init();
    if (s_initialized == 0U || s_s3_service_task_handle != NULL) {
        return;
    }
    if (xTaskCreate(s3_service_task, "s3_service", S3_SERVICE_STACK_WORDS,
                    NULL, S3_SERVICE_PRIORITY, &s_s3_service_task_handle) != pdPASS) {
        s_s3_service_task_handle = NULL;
        (void)snprintf(s_line, sizeof(s_line),
                       "SRP_TASK_CREATE_FAIL name=s3_service stack_words=%u\r\n",
                       (unsigned)S3_SERVICE_STACK_WORDS);
        LOG_ERROR(s_line);
    } else {
        (void)snprintf(s_line, sizeof(s_line),
                       "SRP_TASK_CREATE_OK name=s3_service stack_words=%u\r\n",
                       (unsigned)S3_SERVICE_STACK_WORDS);
        LOG_INFO(s_line);
    }
}

uint8_t s3_service_is_synced(void)
{
    return (uint8_t)(s_initialized != 0U &&
                     s_host_state == S3_SERVICE_HOST_SYNCED);
}
