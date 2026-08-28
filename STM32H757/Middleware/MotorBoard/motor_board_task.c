#include "motor_board_task.h"

#include <stdbool.h>
#include <ctype.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_timer.h"
#include "log_service.h"
#include "motor_board_protocol.h"
#include "motor_board_transport_uart.h"
#include "pid_controller.h"
#include "srp_registry.h"
#include "srp_wire.h"
#include "s3_service.h"
#include "wheel_control_params.h"

#define MB_TASK_STACK_WORDS UINT16_C(1024)
#define MB_TASK_PRIORITY (tskIDLE_PRIORITY + 2U)
#define MB_TASK_POLL_PERIOD_MS UINT32_C(1)
#define MB_TASK_SEQUENCE_GAP_MS UINT32_C(100)
#define MB_TASK_RESPONSE_TIMEOUT_MS UINT32_C(1000)
#define MB_TASK_FEEDBACK_TIMEOUT_MS UINT32_C(1000)
#define MB_TASK_REPORT_PERIOD_MS UINT32_C(1000)
#define MB_TASK_STATS_PERIOD_MS UINT32_C(5000)
#define MB_TASK_WHEEL_STATUS_PERIOD_MS UINT32_C(50)
#define MB_TASK_POWER_STATUS_PERIOD_MS UINT32_C(500)
/* Valid elapsed time between MSPD feedback frames (s); integer microsecond
 * thresholds below avoid floating-point overflow during timestamp checks. */
#define MB_PID_DT_MIN_SECONDS 0.002f
#define MB_PID_DT_MAX_SECONDS 0.100f
#define MB_PID_DT_MIN_US UINT64_C(2000)
#define MB_PID_DT_MAX_US UINT64_C(100000)
#define MB_MSPD_WATCHDOG_TIMEOUT_US UINT64_C(200000)
#define MB_BATTERY_FRESHNESS_TIMEOUT_US UINT64_C(2000000)
#define MB_WHEEL_COUNT 4U
#define MB_LOG_RAW_CHUNK_LENGTH UINT16_C(64)

#define MB_520_MOTOR_TYPE UINT8_C(1)
#define MB_520_DEAD_ZONE UINT16_C(1600)
#define MB_520_MAGNETIC_LINE_COUNT UINT16_C(11)
#define MB_520_GEAR_RATIO UINT16_C(30)
#define MB_520_WHEEL_DIAMETER_MM UINT16_C(65)

typedef enum {
    MB_SEQUENCE_LINK_PROBE = 0U,
    MB_SEQUENCE_READ_FLASH,
    MB_SEQUENCE_MTYPE,
    MB_SEQUENCE_DEADZONE,
    MB_SEQUENCE_MLINE,
    MB_SEQUENCE_MPHASE,
    MB_SEQUENCE_WDIAMETER,
    MB_SEQUENCE_READ_VOLTAGE,
    MB_SEQUENCE_UPLOAD,
    MB_SEQUENCE_WAIT_FEEDBACK,
    MB_SEQUENCE_RUNNING,
    MB_SEQUENCE_FAILED
} mb_sequence_step_t;

typedef enum {
    MB_CONTROL_LOCKED = 0U,
    MB_CONTROL_CONFIGURING,
    MB_CONTROL_WAIT_FEEDBACK,
    MB_CONTROL_READY,
    MB_CONTROL_FAULT
} mb_control_state_t;

typedef enum {
    MB_WAIT_NONE = 0U,
    MB_WAIT_CONFIG_OK,
    MB_WAIT_FLASH_RESPONSE,
    MB_WAIT_BATTERY_RESPONSE
} mb_wait_response_t;

/* Motor-board order is fixed as M1=RR, M2=RF, M3=LR, M4=LF. The RF encoder
 * is electrically inverted and is calibrated exactly once at the PID input. */
static const char *const MOTOR_POSITION_NAME[4] = { "RR", "RF", "LR", "LF" };
static const int8_t ENCODER_DIR_SIGN[4] = { 1, -1, 1, 1 };
static const float s_wheel_trim[4] = {
    WHEEL_TRIM_M1, WHEEL_TRIM_M2, WHEEL_TRIM_M3, WHEEL_TRIM_M4
};

static TaskHandle_t s_task_handle;
static mb_sequence_step_t s_sequence_step;
static mb_control_state_t s_control_state;
static mb_wait_response_t s_wait_response;
static TickType_t s_next_sequence_tick;
static TickType_t s_response_deadline;
static TickType_t s_feedback_deadline;
static TickType_t s_next_report_tick;
static TickType_t s_next_stats_tick;
static TickType_t s_next_wheel_status_tick;
static TickType_t s_next_power_status_tick;
static mb_protocol_frame_t s_latest_battery;
static mb_protocol_frame_t s_latest_mtep;
static mb_protocol_frame_t s_latest_mspd;
static mb_protocol_frame_t s_latest_mall;
static bool s_have_battery;
static bool s_have_mtep;
static bool s_have_mspd;
static bool s_have_mall;
static float s_target_wheel_speed[MB_WHEEL_COUNT];
static float s_actual_wheel_speed[MB_WHEEL_COUNT];
static pid_controller_t s_wheel_pid[MB_WHEEL_COUNT];
static Ramp_Profile_t s_wheel_ramp[MB_WHEEL_COUNT];
static bool s_motion_forced_stop;
static char s_latest_raw[MB_PROTOCOL_MAX_FRAME_LEN];
static uint32_t s_link_probe_attempts;
static int16_t s_last_logged_pwm[MB_WHEEL_COUNT];
static bool s_have_logged_pwm;
static uint64_t s_last_mspd_timestamp_us;
static bool s_mspd_timestamp_valid;
static uint64_t s_last_mspd_rx_timestamp_us;
static bool s_mspd_rx_timestamp_valid;
static uint32_t s_feedback_valid_count;
static bool s_feedback_first_seen;
static uint64_t s_battery_timestamp_us;
static bool s_battery_timestamp_valid;
static uint32_t s_config_pending_mask;
static bool s_flash_verify_required;

static bool tick_due(TickType_t now, TickType_t due)
{
    return (int32_t)(now - due) >= 0;
}

static bool motor_board_text_contains_ci(const char *text, const char *needle)
{
    size_t text_length;
    size_t needle_length;

    if (text == NULL || needle == NULL) {
        return false;
    }
    text_length = strlen(text);
    needle_length = strlen(needle);
    if (needle_length == 0U || needle_length > text_length) {
        return false;
    }
    for (size_t start = 0U; start + needle_length <= text_length; ++start) {
        size_t offset;

        for (offset = 0U; offset < needle_length; ++offset) {
            if (tolower((unsigned char)text[start + offset]) !=
                tolower((unsigned char)needle[offset])) {
                break;
            }
        }
        if (offset == needle_length) {
            return true;
        }
    }
    return false;
}

static float clamp_wheel_pwm(float value)
{
    if (!isfinite(value)) {
        return 0.0f;
    }
    if (value > WHEEL_PID_MAX_OUT) {
        return WHEEL_PID_MAX_OUT;
    }
    if (value < -WHEEL_PID_MAX_OUT) {
        return -WHEEL_PID_MAX_OUT;
    }
    return value;
}

/* Use the shared monotonic DWT-backed clock. A bad interval establishes a new
 * baseline and suppresses one control update, so a delayed MSPD frame cannot
 * inject an oversized integral or feedback-filter step. */
static bool motor_board_get_mspd_dt(float *dt_seconds)
{
    const uint64_t now_us = bsp_timer_get_us();
    uint64_t delta_us;
    float dt;
    bool valid = false;

    if (dt_seconds == NULL || now_us == 0U) {
        return false;
    }
    taskENTER_CRITICAL();
    if (!s_mspd_timestamp_valid) {
        s_last_mspd_timestamp_us = now_us;
        s_mspd_timestamp_valid = true;
    } else if (now_us <= s_last_mspd_timestamp_us) {
        s_last_mspd_timestamp_us = now_us;
    } else {
        delta_us = now_us - s_last_mspd_timestamp_us;
        s_last_mspd_timestamp_us = now_us;
        if (delta_us <= MB_PID_DT_MAX_US) {
            if (delta_us < MB_PID_DT_MIN_US) {
                delta_us = MB_PID_DT_MIN_US;
            }
            dt = (float)delta_us * 1.0e-6f;
            if (!isfinite(dt) || dt < MB_PID_DT_MIN_SECONDS ||
                dt > MB_PID_DT_MAX_SECONDS) {
                taskEXIT_CRITICAL();
                return false;
            }
            *dt_seconds = dt;
            valid = true;
        }
    }
    taskEXIT_CRITICAL();
    return valid;
}

static void motor_board_reset_pid_history(void)
{
    for (size_t index = 0U; index < MB_WHEEL_COUNT; ++index) {
        pid_controller_reset(&s_wheel_pid[index]);
    }
}

static const char *motor_board_step_name(mb_sequence_step_t step)
{
    switch (step) {
    case MB_SEQUENCE_MTYPE:
        return "mtype";
    case MB_SEQUENCE_DEADZONE:
        return "deadzone";
    case MB_SEQUENCE_MLINE:
        return "mline";
    case MB_SEQUENCE_MPHASE:
        return "mphase";
    case MB_SEQUENCE_WDIAMETER:
        return "wdiameter";
    case MB_SEQUENCE_READ_FLASH:
        return "read_flash";
    case MB_SEQUENCE_READ_VOLTAGE:
        return "read_vol";
    case MB_SEQUENCE_UPLOAD:
        return "upload";
    case MB_SEQUENCE_WAIT_FEEDBACK:
        return "feedback";
    case MB_SEQUENCE_LINK_PROBE:
        return "link_probe";
    case MB_SEQUENCE_RUNNING:
        return "running";
    case MB_SEQUENCE_FAILED:
    default:
        return "failed";
    }
}

static void motor_board_reset_control_state(void)
{
    taskENTER_CRITICAL();
    (void)memset(s_target_wheel_speed, 0, sizeof(s_target_wheel_speed));
    for (size_t index = 0U; index < MB_WHEEL_COUNT; ++index) {
        pid_controller_reset(&s_wheel_pid[index]);
        s_wheel_ramp[index].current_target = 0.0f;
        s_actual_wheel_speed[index] = 0.0f;
    }
    s_last_mspd_timestamp_us = 0U;
    s_mspd_timestamp_valid = false;
    s_feedback_valid_count = 0U;
    s_feedback_first_seen = false;
    s_motion_forced_stop = true;
    taskEXIT_CRITICAL();
}

static uint32_t motor_board_flash_mismatch(const mb_flash_config_t *config)
{
    uint32_t mismatch = 0U;

    if (config == NULL || !config->complete) {
        return MB_FLASH_FIELD_REQUIRED_MASK;
    }
    if (config->motor_type != MB_520_MOTOR_TYPE) {
        mismatch |= MB_FLASH_FIELD_MOTOR_TYPE;
    }
    if (config->dead_zone != MB_520_DEAD_ZONE) {
        mismatch |= MB_FLASH_FIELD_DEAD_ZONE;
    }
    if (config->pulse_line != MB_520_MAGNETIC_LINE_COUNT) {
        mismatch |= MB_FLASH_FIELD_PULSE_LINE;
    }
    if (config->pulse_phase != MB_520_GEAR_RATIO) {
        mismatch |= MB_FLASH_FIELD_PULSE_PHASE;
    }
    if (!isfinite(config->wheel_diameter) ||
        fabsf(config->wheel_diameter - (float)MB_520_WHEEL_DIAMETER_MM) >
            0.01f) {
        mismatch |= MB_FLASH_FIELD_WHEEL_DIAMETER;
    }
    return mismatch;
}

static void motor_board_log(const char *text)
{
    if (text != NULL) {
        LOG_INFO(text);
    }
}

static void motor_board_log_send(const char *command, bool sent)
{
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    (void)snprintf(line, sizeof(line),
                   "[MOTOR_BOARD] tx=%s status=%s",
                   command == NULL ? "?" : command,
                   sent ? "QUEUED" : "DROP");
    if (sent) {
        motor_board_log(line);
    } else {
        LOG_ERROR(line);
    }
}

static void motor_board_log_raw(const char *category, const char *raw)
{
    const char *cursor = raw;
    uint32_t fragment = 0U;

    if (category == NULL || raw == NULL || raw[0] == '\0') {
        return;
    }
    while (*cursor != '\0') {
        char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
        size_t length = strlen(cursor);

        if (length > MB_LOG_RAW_CHUNK_LENGTH) {
            length = MB_LOG_RAW_CHUNK_LENGTH;
        }
        (void)snprintf(line, sizeof(line), "[MOTOR_BOARD] %s[%lu]=%.*s",
                       category, (unsigned long)fragment, (int)length, cursor);
        motor_board_log(line);
        cursor += length;
        ++fragment;
    }
}

static void motor_board_log_unrecognized_raw(const char *category,
                                             const char *raw)
{
    const uint8_t *bytes = (const uint8_t *)raw;
    size_t offset = 0U;

    if (category == NULL || raw == NULL || raw[0] == '\0') {
        return;
    }
    while (raw[offset] != '\0') {
        char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
        char ascii[17];
        size_t length = 0U;
        size_t line_offset;

        while (length < 16U && raw[offset + length] != '\0') {
            const uint8_t byte = bytes[offset + length];
            ascii[length] = (byte >= 0x20U && byte <= 0x7EU) ?
                                (char)byte : '.';
            ++length;
        }
        ascii[length] = '\0';
        line_offset = (size_t)snprintf(line, sizeof(line),
                                       "[MOTOR_BOARD] %s hex=", category);
        if (line_offset >= sizeof(line)) {
            return;
        }
        for (size_t index = 0U;
             index < length && line_offset + 4U < sizeof(line); ++index) {
            const int written = snprintf(&line[line_offset],
                                         sizeof(line) - line_offset,
                                         "%02X%s", bytes[offset + index],
                                         index + 1U == length ? "" : " ");
            if (written < 0) {
                return;
            }
            line_offset += (size_t)written;
        }
        if (line_offset < sizeof(line)) {
            (void)snprintf(&line[line_offset], sizeof(line) - line_offset,
                           " ascii=%s", ascii);
        }
        LOG_WARN(line);
        offset += length;
    }
}

static void motor_board_enter_failed(const char *reason)
{
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
    const char *failed_step = motor_board_step_name(s_sequence_step);
    bool stop_queued;

    motor_board_reset_control_state();
    stop_queued = MB_Protocol_SendPwm(0, 0, 0, 0);
    s_wait_response = MB_WAIT_NONE;
    s_sequence_step = MB_SEQUENCE_FAILED;
    s_control_state = MB_CONTROL_FAULT;
    (void)snprintf(line, sizeof(line),
                   "[MOTOR_BOARD] init failed step=%s reason=%s stop=%s",
                   failed_step,
                   reason == NULL ? "?" : reason,
                   stop_queued ? "QUEUED" : "DROP");
    LOG_ERROR(line);
}

static void motor_board_log_rx_health(const char *reason)
{
    mb_transport_stats_t stats;
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
    const bool active = MB_Transport_Ensure_Rx_Active();
    const uint32_t isr = USART6->ISR;
    const uint32_t cr1 = USART6->CR1;

    MB_Transport_GetStats(&stats);

    (void)snprintf(line, sizeof(line),
                   "[MB6] %.8s a=%u i=%08lX c=%08lX r=%lu b=%u e=%lu",
                   reason == NULL ? "?" : reason,
                   active ? 1U : 0U,
                   (unsigned long)isr,
                   (unsigned long)cr1,
                   (unsigned long)stats.rx_bytes,
                   (unsigned int)stats.rx_buffered,
                   (unsigned long)stats.uart_errors);
    if (active) {
        LOG_WARN(line);
    } else {
        LOG_ERROR(line);
    }
}

static void motor_board_restart_link_probe(TickType_t now, const char *reason)
{
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    motor_board_reset_control_state();
    (void)MB_Protocol_SendPwm(0, 0, 0, 0);
    MB_Transport_ClearRx();
    MB_Protocol_ResetRx();
    s_wait_response = MB_WAIT_NONE;
    s_sequence_step = MB_SEQUENCE_LINK_PROBE;
    s_control_state = MB_CONTROL_LOCKED;
    s_config_pending_mask = 0U;
    s_flash_verify_required = false;
    s_last_mspd_rx_timestamp_us = 0U;
    s_mspd_rx_timestamp_valid = false;
    s_next_sequence_tick = now;
    motor_board_log_rx_health(reason);
    (void)snprintf(line, sizeof(line),
                   "[MOTOR_BOARD] link retry=%lu reason=%s rx/parser reset",
                   (unsigned long)(s_link_probe_attempts + 1U),
                   reason == NULL ? "?" : reason);
    LOG_WARN(line);
}

static bool motor_board_wait_for_response(mb_wait_response_t response,
                                          TickType_t now)
{
    if (response == MB_WAIT_NONE) {
        return true;
    }
    s_wait_response = response;
    s_response_deadline = now + pdMS_TO_TICKS(MB_TASK_RESPONSE_TIMEOUT_MS);
    return true;
}

static void motor_board_select_next_config(TickType_t now)
{
    s_wait_response = MB_WAIT_NONE;
    if ((s_config_pending_mask & MB_FLASH_FIELD_MOTOR_TYPE) != 0U) {
        s_sequence_step = MB_SEQUENCE_MTYPE;
    } else if ((s_config_pending_mask & MB_FLASH_FIELD_DEAD_ZONE) != 0U) {
        s_sequence_step = MB_SEQUENCE_DEADZONE;
    } else if ((s_config_pending_mask & MB_FLASH_FIELD_PULSE_LINE) != 0U) {
        s_sequence_step = MB_SEQUENCE_MLINE;
    } else if ((s_config_pending_mask & MB_FLASH_FIELD_PULSE_PHASE) != 0U) {
        s_sequence_step = MB_SEQUENCE_MPHASE;
    } else if ((s_config_pending_mask & MB_FLASH_FIELD_WHEEL_DIAMETER) != 0U) {
        s_sequence_step = MB_SEQUENCE_WDIAMETER;
    } else if (s_flash_verify_required) {
        s_sequence_step = MB_SEQUENCE_READ_FLASH;
    } else {
        s_sequence_step = MB_SEQUENCE_READ_VOLTAGE;
    }
    s_next_sequence_tick = now + pdMS_TO_TICKS(MB_TASK_SEQUENCE_GAP_MS);
}

static bool motor_board_ack_matches_step(const mb_protocol_frame_t *frame)
{
    const char *raw;

    if (frame == NULL || frame->type != MB_FRAME_OK_ACK) {
        return false;
    }
    raw = frame->response_status[0] == '\0' ? frame->raw :
                                                frame->response_status;
    switch (s_sequence_step) {
    case MB_SEQUENCE_MTYPE:
        return motor_board_text_contains_ci(raw, "mtype") ||
               motor_board_text_contains_ci(raw, "motor_type") ||
               motor_board_text_contains_ci(raw, "motor type");
    case MB_SEQUENCE_DEADZONE:
        return motor_board_text_contains_ci(raw, "deadzone") ||
               motor_board_text_contains_ci(raw, "dead_zone") ||
               motor_board_text_contains_ci(raw, "dead zone");
    case MB_SEQUENCE_MLINE:
        return motor_board_text_contains_ci(raw, "mline") ||
               motor_board_text_contains_ci(raw, "pulse_line") ||
               motor_board_text_contains_ci(raw, "pulse line");
    case MB_SEQUENCE_MPHASE:
        return motor_board_text_contains_ci(raw, "mphase") ||
               motor_board_text_contains_ci(raw, "pulse_phase") ||
               motor_board_text_contains_ci(raw, "pulse phase");
    case MB_SEQUENCE_WDIAMETER:
        return motor_board_text_contains_ci(raw, "wdiameter") ||
               motor_board_text_contains_ci(raw, "wheel_diameter") ||
               motor_board_text_contains_ci(raw, "wheel diameter");
    default:
        return false;
    }
}

static void motor_board_handle_flash_complete(TickType_t now)
{
    mb_flash_config_t config;
    uint32_t mismatch;

    if (!MB_Protocol_GetFlashConfig(&config)) {
        if (config.invalid) {
            motor_board_enter_failed("flash_invalid");
        }
        return;
    }
    mismatch = motor_board_flash_mismatch(&config);
    MB_Protocol_EndReadFlash();
    if (s_flash_verify_required) {
        if (mismatch != 0U) {
            motor_board_enter_failed("flash_verify_mismatch");
            return;
        }
        s_flash_verify_required = false;
        s_config_pending_mask = 0U;
        motor_board_select_next_config(now);
        return;
    }
    s_config_pending_mask = mismatch;
    if (mismatch != 0U) {
        s_flash_verify_required = true;
    }
    motor_board_select_next_config(now);
}

static void motor_board_log_frame_event(const mb_protocol_frame_t *frame)
{
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    if (frame == NULL) {
        return;
    }
    (void)snprintf(s_latest_raw, sizeof(s_latest_raw), "%s", frame->raw);
    switch (frame->type) {
    case MB_FRAME_BATTERY:
        s_latest_battery = *frame;
        s_have_battery = true;
        s_battery_timestamp_us = bsp_timer_get_us();
        s_battery_timestamp_valid = s_battery_timestamp_us != 0U;
        break;
    case MB_FRAME_MTEP:
        s_latest_mtep = *frame;
        s_have_mtep = true;
        break;
    case MB_FRAME_MSPD:
        s_latest_mspd = *frame;
        s_have_mspd = true;
        break;
    case MB_FRAME_MALL:
        s_latest_mall = *frame;
        s_have_mall = true;
        break;
    case MB_FRAME_OK_ACK:
        (void)snprintf(line, sizeof(line), "[MOTOR_BOARD] ok_ack=%.64s", frame->raw);
        motor_board_log(line);
        break;
    case MB_FRAME_FLASH_RAW:
        motor_board_log_raw("flash", frame->raw);
        break;
    case MB_FRAME_NACK:
        (void)snprintf(line, sizeof(line), "[MOTOR_BOARD] nack=%.75s", frame->raw);
        LOG_ERROR(line);
        break;
    case MB_FRAME_INVALID:
        if (strncmp(frame->raw, "hex=", 4U) == 0) {
            (void)snprintf(line, sizeof(line),
                           "[MOTOR_BOARD] raw %.76s", frame->raw);
            LOG_WARN(line);
        } else {
            motor_board_log_unrecognized_raw("invalid", frame->raw);
        }
        break;
    case MB_FRAME_UNKNOWN:
    default:
        motor_board_log_unrecognized_raw("unknown", frame->raw);
        break;
    }
}

static void motor_board_handle_response(const mb_protocol_frame_t *frame,
                                        TickType_t now)
{
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    if (frame == NULL || s_wait_response == MB_WAIT_NONE) {
        return;
    }
    if (frame->type == MB_FRAME_NACK) {
        motor_board_enter_failed("board_nack");
        return;
    }
    switch (s_wait_response) {
    case MB_WAIT_CONFIG_OK:
        if (motor_board_ack_matches_step(frame)) {
            (void)snprintf(line, sizeof(line),
                           "[MOTOR_BOARD] 520 %s=OK status=%.48s",
                           motor_board_step_name(s_sequence_step),
                           frame->response_status[0] == '\0' ? frame->raw :
                                                               frame->response_status);
            motor_board_log(line);
            switch (s_sequence_step) {
            case MB_SEQUENCE_MTYPE:
                s_config_pending_mask &= ~MB_FLASH_FIELD_MOTOR_TYPE;
                break;
            case MB_SEQUENCE_DEADZONE:
                s_config_pending_mask &= ~MB_FLASH_FIELD_DEAD_ZONE;
                break;
            case MB_SEQUENCE_MLINE:
                s_config_pending_mask &= ~MB_FLASH_FIELD_PULSE_LINE;
                break;
            case MB_SEQUENCE_MPHASE:
                s_config_pending_mask &= ~MB_FLASH_FIELD_PULSE_PHASE;
                break;
            case MB_SEQUENCE_WDIAMETER:
                s_config_pending_mask &= ~MB_FLASH_FIELD_WHEEL_DIAMETER;
                break;
            default:
                break;
            }
            motor_board_select_next_config(now);
        } else if (frame->type == MB_FRAME_OK_ACK) {
            LOG_WARN("[MOTOR_BOARD] unassociated config ACK ignored");
        }
        break;
    case MB_WAIT_FLASH_RESPONSE:
        /* read_flash is a multiline response. The first read_flash:OK! line
         * only opens the response; the five required fields complete it. */
        if (frame->type == MB_FRAME_FLASH_RAW) {
            motor_board_handle_flash_complete(now);
        } else if (frame->type == MB_FRAME_OK_ACK) {
            LOG_WARN("[MOTOR_BOARD] read_flash ACK without complete snapshot");
        }
        if (s_wait_response == MB_WAIT_NONE) {
            (void)snprintf(line, sizeof(line),
                           "[MOTOR_BOARD] read_flash response=COMPLETE");
            motor_board_log(line);
        }
        /* A generic OK must never complete a partial read_flash response. */
        /* no further action */
        break;
    case MB_WAIT_BATTERY_RESPONSE:
        if (frame->type == MB_FRAME_BATTERY) {
            (void)snprintf(line, sizeof(line),
                           "[MOTOR_BOARD] read_vol=%.2fV OK",
                           (double)frame->battery_voltage);
            motor_board_log(line);
            s_wait_response = MB_WAIT_NONE;
            s_response_deadline = now;
            s_sequence_step = MB_SEQUENCE_UPLOAD;
            s_next_sequence_tick = now + pdMS_TO_TICKS(MB_TASK_SEQUENCE_GAP_MS);
        }
        break;
    case MB_WAIT_NONE:
    default:
        break;
    }
}

bool motor_board_set_target_wheel_speeds(const float speeds[MB_WHEEL_COUNT])
{
    bool any_nonzero = false;
    bool ready;

    if (speeds == NULL) {
        return false;
    }
    for (size_t index = 0U; index < MB_WHEEL_COUNT; ++index) {
        if (!isfinite(speeds[index]) ||
            fabsf(speeds[index]) > WHEEL_SPEED_MAX_TARGET_LIMIT) {
            return false;
        }
        any_nonzero = any_nonzero || speeds[index] != 0.0f;
    }
    if (!any_nonzero) {
        return motor_board_force_stop();
    }
    taskENTER_CRITICAL();
    ready = s_control_state == MB_CONTROL_READY;
    taskEXIT_CRITICAL();
    if (!ready) {
        LOG_WARN("[MOTOR_BOARD] nonzero target rejected: board not READY");
        return false;
    }
    taskENTER_CRITICAL();
    (void)memcpy(s_target_wheel_speed, speeds, sizeof(s_target_wheel_speed));
    s_motion_forced_stop = false;
    taskEXIT_CRITICAL();
    return true;
}

bool motor_board_update_pid_params(float kp, float ki, float kd,
                                   float max_accel)
{
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
        !isfinite(max_accel) || kp < SRP_PID_KP_MIN || kp > SRP_PID_KP_MAX ||
        ki < SRP_PID_KI_MIN || ki > SRP_PID_KI_MAX ||
        kd < SRP_PID_KD_MIN || kd > SRP_PID_KD_MAX ||
        max_accel < SRP_PID_ACCEL_MIN || max_accel > SRP_PID_ACCEL_MAX) {
        return false;
    }

    taskENTER_CRITICAL();
    for (size_t index = 0U; index < MB_WHEEL_COUNT; ++index) {
        PID_Update_Gains(&s_wheel_pid[index], kp, ki, kd);
        Ramp_Update_Max_Accel(&s_wheel_ramp[index], max_accel);
    }
    taskEXIT_CRITICAL();
    return true;
}

bool motor_board_force_stop(void)
{
    bool sent;

    motor_board_reset_control_state();
    /* Serialize the physical stop after clearing the target, so a stale
     * nonzero PWM cannot follow this stop. */
    taskENTER_CRITICAL();
    sent = MB_Protocol_SendPwm(0, 0, 0, 0);
    taskEXIT_CRITICAL();
    if (!sent) {
        LOG_ERROR("[MOTOR_BOARD] forced stop PWM queue drop");
    }
    return sent;
}

void motor_board_get_actual_wheel_speeds(float speeds[MB_WHEEL_COUNT])
{
    if (speeds != NULL) {
        (void)memcpy(speeds, s_actual_wheel_speed, sizeof(s_actual_wheel_speed));
    }
}

bool motor_board_get_battery_voltage(float *voltage)
{
    const uint64_t now_us = bsp_timer_get_us();

    if (voltage == NULL || !s_have_battery || !s_battery_timestamp_valid ||
        now_us < s_battery_timestamp_us ||
        now_us - s_battery_timestamp_us > MB_BATTERY_FRESHNESS_TIMEOUT_US ||
        !isfinite(s_latest_battery.battery_voltage)) {
        return false;
    }
    *voltage = s_latest_battery.battery_voltage;
    return true;
}

static void motor_board_update_pid(const mb_protocol_frame_t *frame)
{
    int16_t pwm[MB_WHEEL_COUNT];
    float target_speed[MB_WHEEL_COUNT];
    float dt_seconds;
    uint64_t now_us;
    bool dt_valid;
    bool ready_before;
    bool became_ready = false;
    bool sent;

    if (frame == NULL || frame->type != MB_FRAME_MSPD) {
        return;
    }
    for (size_t index = 0U; index < MB_WHEEL_COUNT; ++index) {
        if (!isfinite(frame->speed[index])) {
            return;
        }
    }
    now_us = bsp_timer_get_us();
    if (now_us == 0U) {
        return;
    }
    dt_valid = motor_board_get_mspd_dt(&dt_seconds);
    taskENTER_CRITICAL();
    ready_before = s_control_state == MB_CONTROL_READY;
    if (s_control_state == MB_CONTROL_WAIT_FEEDBACK) {
        if (!s_feedback_first_seen) {
            s_feedback_first_seen = true;
            s_feedback_valid_count = 1U;
        } else if (dt_valid) {
            ++s_feedback_valid_count;
        } else {
            s_feedback_first_seen = false;
            s_feedback_valid_count = 0U;
        }
        if (s_feedback_valid_count >= 2U) {
            s_control_state = MB_CONTROL_READY;
            s_sequence_step = MB_SEQUENCE_RUNNING;
            s_wait_response = MB_WAIT_NONE;
            s_motion_forced_stop = false;
            became_ready = true;
        }
    }
    if (!dt_valid) {
        for (size_t index = 0U; index < MB_WHEEL_COUNT; ++index) {
            s_actual_wheel_speed[index] =
                frame->speed[index] * (float)ENCODER_DIR_SIGN[index];
        }
        motor_board_reset_pid_history();
        taskEXIT_CRITICAL();
        if (became_ready) {
            motor_board_log("[MOTOR_BOARD] state=READY feedback=2");
        }
        return;
    }
    s_last_mspd_rx_timestamp_us = now_us;
    s_mspd_rx_timestamp_valid = true;
    if (!ready_before || s_motion_forced_stop) {
        for (size_t index = 0U; index < MB_WHEEL_COUNT; ++index) {
            s_actual_wheel_speed[index] =
                frame->speed[index] * (float)ENCODER_DIR_SIGN[index];
        }
        taskEXIT_CRITICAL();
        if (became_ready) {
            motor_board_log("[MOTOR_BOARD] state=READY feedback=2");
        }
        return;
    }
    (void)memcpy(target_speed, s_target_wheel_speed, sizeof(target_speed));
    for (size_t index = 0U; index < MB_WHEEL_COUNT; ++index) {
        const float raw_speed = frame->speed[index];
        float actual_speed = raw_speed * (float)ENCODER_DIR_SIGN[index];

        if (!isfinite(actual_speed)) {
            actual_speed = 0.0f;
        }
        if (!isfinite(target_speed[index])) {
            target_speed[index] = 0.0f;
        }
        if (fabsf(target_speed[index]) > 0.0f &&
            fabsf(target_speed[index]) < WHEEL_SPEED_MIN_TARGET_LIMIT) {
            target_speed[index] = 0.0f;
        }
        const float smooth_target = Ramp_Update(&s_wheel_ramp[index],
                                                target_speed[index],
                                                dt_seconds);

        /* PID output is already signed. Trim changes magnitude only; it
         * neither changes direction nor bypasses the final output limit. */
        const float raw_pwm = pid_controller_step(
            &s_wheel_pid[index], smooth_target,
            actual_speed, dt_seconds);
        s_actual_wheel_speed[index] = actual_speed;
        if (!isfinite(raw_pwm)) {
            pwm[index] = 0;
        } else {
            pwm[index] = (int16_t)clamp_wheel_pwm(
                raw_pwm * s_wheel_trim[index]);
        }
    }
    sent = MB_Protocol_SendPwm(pwm[0], pwm[1], pwm[2], pwm[3]);
    taskEXIT_CRITICAL();
    if (sent && (!s_have_logged_pwm ||
                 memcmp(s_last_logged_pwm, pwm, sizeof(pwm)) != 0)) {
        char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

        (void)snprintf(line, sizeof(line),
                       "[MOTOR_CMD] target=[RR %.0f RF %.0f LR %.0f LF %.0f] "
                       "pwm=$pwm:%d,%d,%d,%d#",
                       (double)target_speed[0], (double)target_speed[1],
                       (double)target_speed[2], (double)target_speed[3],
                       (int)pwm[0], (int)pwm[1], (int)pwm[2], (int)pwm[3]);
        LOG_INFO(line);
        (void)memcpy(s_last_logged_pwm, pwm, sizeof(s_last_logged_pwm));
        s_have_logged_pwm = true;
    }
    if (!sent) {
        LOG_WARN("[MOTOR_BOARD] PID PWM queue drop");
    }
}

static void motor_board_check_mspd_watchdog(TickType_t now)
{
    const uint64_t now_us = bsp_timer_get_us();
    bool timed_out = false;

    if (now_us == 0U) {
        return;
    }
    taskENTER_CRITICAL();
    if (s_control_state == MB_CONTROL_READY &&
        s_mspd_rx_timestamp_valid &&
        now_us >= s_last_mspd_rx_timestamp_us &&
        now_us - s_last_mspd_rx_timestamp_us >
            MB_MSPD_WATCHDOG_TIMEOUT_US) {
        timed_out = true;
    }
    taskEXIT_CRITICAL();
    if (timed_out) {
        motor_board_restart_link_probe(now, "mspd_timeout");
    }
}

static void motor_board_send_wheel_status(void)
{
    uint8_t payload[SRP_PAYLOAD_WHEEL_SPEED_STATUS_SIZE];

    srp_wire_write_f32_array_le(payload, s_actual_wheel_speed, MB_WHEEL_COUNT);
    (void)s3_service_send_message(SRP_PRIORITY_TELEMETRY,
                                  SRP_MSG_ID_WHEEL_SPEED_STATUS,
                                  SRP_FLAG_STREAM_DATA, payload,
                                  sizeof(payload));
}

static void motor_board_send_power_status(void)
{
    float voltage;
    uint8_t payload[SRP_PAYLOAD_POWER_STATUS_SIZE];

    if (!motor_board_get_battery_voltage(&voltage)) {
        return;
    }
    srp_wire_write_f32_le(payload, voltage);
    (void)s3_service_send_message(SRP_PRIORITY_TELEMETRY,
                                  SRP_MSG_ID_POWER_STATUS,
                                  SRP_FLAG_STREAM_DATA, payload,
                                  sizeof(payload));
}

static void motor_board_log_snapshot(void)
{
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];
    float voltage;

    if (motor_board_get_battery_voltage(&voltage)) {
        (void)snprintf(line, sizeof(line), "[MOTOR_BOARD] Battery=%.2fV",
                       (double)voltage);
        motor_board_log(line);
    } else if (s_have_battery) {
        motor_board_log("[MOTOR_BOARD] Battery=STALE");
    }
    if (s_have_mtep) {
        (void)snprintf(line, sizeof(line),
                       "[MOTOR_BOARD] MTEP M1(%s)=%ld M2(%s)=%ld "
                       "M3(%s)=%ld M4(%s)=%ld",
                       MOTOR_POSITION_NAME[0],
                       (long)s_latest_mtep.pulse[0],
                       MOTOR_POSITION_NAME[1],
                       (long)s_latest_mtep.pulse[1],
                       MOTOR_POSITION_NAME[2],
                       (long)s_latest_mtep.pulse[2],
                       MOTOR_POSITION_NAME[3],
                       (long)s_latest_mtep.pulse[3]);
        motor_board_log(line);
    }
    if (s_have_mspd) {
        (void)snprintf(line, sizeof(line),
                       "[MOTOR_BOARD] MSPD M1(%s)=%.2f M2(%s)=%.2f "
                       "M3(%s)=%.2f M4(%s)=%.2f",
                       MOTOR_POSITION_NAME[0],
                       (double)s_latest_mspd.speed[0],
                       MOTOR_POSITION_NAME[1],
                       (double)s_latest_mspd.speed[1],
                       MOTOR_POSITION_NAME[2],
                       (double)s_latest_mspd.speed[2],
                       MOTOR_POSITION_NAME[3],
                       (double)s_latest_mspd.speed[3]);
        motor_board_log(line);
    }
    if (s_have_mall) {
        (void)snprintf(line, sizeof(line),
                       "[MOTOR_BOARD] MAll M1=%ld M2=%ld M3=%ld M4=%ld",
                       (long)s_latest_mall.pulse[0],
                       (long)s_latest_mall.pulse[1],
                       (long)s_latest_mall.pulse[2],
                       (long)s_latest_mall.pulse[3]);
        motor_board_log(line);
    }
    if (s_latest_raw[0] != '\0') {
        (void)snprintf(line, sizeof(line), "[MOTOR_BOARD] raw=%.76s",
                       s_latest_raw);
        motor_board_log(line);
    }
}

static void motor_board_log_stats(void)
{
    mb_transport_stats_t transport_stats;
    mb_protocol_stats_t protocol_stats;
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    MB_Transport_GetStats(&transport_stats);
    MB_Protocol_GetStats(&protocol_stats);
    (void)snprintf(line, sizeof(line),
                   "[MOTOR_BOARD] stats rx=%lu tx=%lu ro=%lu to=%lu ue=%lu "
                   "fr=%lu inv=%lu",
                   (unsigned long)transport_stats.rx_bytes,
                   (unsigned long)transport_stats.tx_bytes,
                   (unsigned long)transport_stats.rx_overflow,
                   (unsigned long)transport_stats.tx_overflow,
                   (unsigned long)transport_stats.uart_errors,
                   (unsigned long)protocol_stats.frames,
                   (unsigned long)protocol_stats.invalid_frames);
    motor_board_log(line);
}

static void motor_board_run_sequence(TickType_t now)
{
    bool sent;

    if (s_wait_response != MB_WAIT_NONE) {
        if (tick_due(now, s_response_deadline)) {
            motor_board_restart_link_probe(now, "response_timeout");
        }
        return;
    }
    if (!tick_due(now, s_next_sequence_tick)) {
        return;
    }
    switch (s_sequence_step) {
    case MB_SEQUENCE_LINK_PROBE:
        ++s_link_probe_attempts;
        sent = MB_Protocol_SendPwm(0, 0, 0, 0);
        motor_board_log_send("$pwm:0,0,0,0#", sent);
        if (sent) {
            s_control_state = MB_CONTROL_CONFIGURING;
            s_sequence_step = MB_SEQUENCE_READ_FLASH;
            s_next_sequence_tick = now + pdMS_TO_TICKS(MB_TASK_SEQUENCE_GAP_MS);
        } else {
            s_next_sequence_tick =
                now + pdMS_TO_TICKS(MB_TASK_RESPONSE_TIMEOUT_MS);
        }
        break;
    case MB_SEQUENCE_MTYPE:
        sent = MB_Protocol_SendMotorType(MB_520_MOTOR_TYPE);
        motor_board_log_send("$mtype:1#", sent);
        if (sent) {
            (void)motor_board_wait_for_response(MB_WAIT_CONFIG_OK, now);
        } else {
            motor_board_enter_failed("tx_drop");
        }
        break;
    case MB_SEQUENCE_DEADZONE:
        sent = MB_Protocol_SendDeadzone(MB_520_DEAD_ZONE);
        motor_board_log_send("$deadzone:1600#", sent);
        if (sent) {
            (void)motor_board_wait_for_response(MB_WAIT_CONFIG_OK, now);
        } else {
            motor_board_enter_failed("tx_drop");
        }
        break;
    case MB_SEQUENCE_MLINE:
        sent = MB_Protocol_SendMagneticLine(MB_520_MAGNETIC_LINE_COUNT);
        motor_board_log_send("$mline:11#", sent);
        if (sent) {
            (void)motor_board_wait_for_response(MB_WAIT_CONFIG_OK, now);
        } else {
            motor_board_enter_failed("tx_drop");
        }
        break;
    case MB_SEQUENCE_MPHASE:
        sent = MB_Protocol_SendGearRatio(MB_520_GEAR_RATIO);
        motor_board_log_send("$mphase:30#", sent);
        if (sent) {
            (void)motor_board_wait_for_response(MB_WAIT_CONFIG_OK, now);
        } else {
            motor_board_enter_failed("tx_drop");
        }
        break;
    case MB_SEQUENCE_WDIAMETER:
        sent = MB_Protocol_SendWheelDiameter(MB_520_WHEEL_DIAMETER_MM);
        motor_board_log_send("$wdiameter:65#", sent);
        if (sent) {
            (void)motor_board_wait_for_response(MB_WAIT_CONFIG_OK, now);
        } else {
            motor_board_enter_failed("tx_drop");
        }
        break;
    case MB_SEQUENCE_READ_FLASH:
        sent = MB_Protocol_SendReadFlash();
        motor_board_log_send("$read_flash#", sent);
        if (sent) {
            (void)motor_board_wait_for_response(MB_WAIT_FLASH_RESPONSE, now);
        } else {
            motor_board_enter_failed("tx_drop");
        }
        break;
    case MB_SEQUENCE_READ_VOLTAGE:
        sent = MB_Protocol_SendReadVoltage();
        motor_board_log_send("$read_vol#", sent);
        if (sent) {
            (void)motor_board_wait_for_response(MB_WAIT_BATTERY_RESPONSE, now);
        } else {
            motor_board_enter_failed("tx_drop");
        }
        break;
    case MB_SEQUENCE_UPLOAD:
        sent = MB_Protocol_SendUpload(false, true, true);
        motor_board_log_send("$upload:0,1,1#", sent);
        if (sent) {
            s_control_state = MB_CONTROL_WAIT_FEEDBACK;
            s_sequence_step = MB_SEQUENCE_WAIT_FEEDBACK;
            s_feedback_deadline = now + pdMS_TO_TICKS(MB_TASK_FEEDBACK_TIMEOUT_MS);
            s_feedback_valid_count = 0U;
            s_feedback_first_seen = false;
            s_mspd_rx_timestamp_valid = false;
        } else {
            motor_board_enter_failed("tx_drop");
        }
        break;
    case MB_SEQUENCE_WAIT_FEEDBACK:
        if (tick_due(now, s_feedback_deadline)) {
            motor_board_restart_link_probe(now, "feedback_timeout");
        }
        break;
    case MB_SEQUENCE_RUNNING:
    case MB_SEQUENCE_FAILED:
    default:
        break;
    }
}

void motor_board_task(void *argument)
{
    TickType_t now;
    mb_protocol_frame_t frame;

    (void)argument;
    MB_Transport_ClearRx();
    s_sequence_step = MB_SEQUENCE_LINK_PROBE;
    s_wait_response = MB_WAIT_NONE;
    s_control_state = MB_CONTROL_LOCKED;
    s_have_battery = false;
    s_have_mtep = false;
    s_have_mspd = false;
    s_have_mall = false;
    s_latest_raw[0] = '\0';
    s_link_probe_attempts = 0U;
    s_motion_forced_stop = true;
    s_last_mspd_timestamp_us = 0U;
    s_mspd_timestamp_valid = false;
    s_last_mspd_rx_timestamp_us = 0U;
    s_mspd_rx_timestamp_valid = false;
    s_feedback_valid_count = 0U;
    s_feedback_first_seen = false;
    s_have_logged_pwm = false;
    (void)memset(s_last_logged_pwm, 0, sizeof(s_last_logged_pwm));
    now = xTaskGetTickCount();
    s_next_sequence_tick = now;
    s_next_report_tick = now + pdMS_TO_TICKS(MB_TASK_REPORT_PERIOD_MS);
    s_next_stats_tick = now + pdMS_TO_TICKS(MB_TASK_STATS_PERIOD_MS);
    s_next_wheel_status_tick = now + pdMS_TO_TICKS(MB_TASK_WHEEL_STATUS_PERIOD_MS);
    s_next_power_status_tick = now + pdMS_TO_TICKS(MB_TASK_POWER_STATUS_PERIOD_MS);
    (void)memset(s_target_wheel_speed, 0, sizeof(s_target_wheel_speed));
    (void)memset(s_actual_wheel_speed, 0, sizeof(s_actual_wheel_speed));
    s_config_pending_mask = 0U;
    s_flash_verify_required = false;
    s_feedback_deadline = now;
    s_battery_timestamp_us = 0U;
    s_battery_timestamp_valid = false;
    for (size_t index = 0U; index < MB_WHEEL_COUNT; ++index) {
        s_wheel_ramp[index].current_target = 0.0f;
        s_wheel_ramp[index].max_accel = WHEEL_RAMP_MAX_ACCEL;
        pid_controller_init(&s_wheel_pid[index], WHEEL_PID_KP, WHEEL_PID_KI,
                            WHEEL_PID_KD, WHEEL_PID_MAX_OUT,
                            WHEEL_PID_MAX_IOUT, WHEEL_SPEED_DEADBAND);
    }
    for (;;) {
        now = xTaskGetTickCount();
        while (MB_Protocol_Poll(&frame)) {
            motor_board_log_frame_event(&frame);
            motor_board_handle_response(&frame, now);
            motor_board_update_pid(&frame);
            now = xTaskGetTickCount();
        }
        motor_board_check_mspd_watchdog(now);
        motor_board_run_sequence(now);
        if (tick_due(now, s_next_wheel_status_tick)) {
            s_next_wheel_status_tick = now + pdMS_TO_TICKS(MB_TASK_WHEEL_STATUS_PERIOD_MS);
            motor_board_send_wheel_status();
        }
        if (tick_due(now, s_next_power_status_tick)) {
            s_next_power_status_tick = now + pdMS_TO_TICKS(MB_TASK_POWER_STATUS_PERIOD_MS);
            motor_board_send_power_status();
        }
        if (tick_due(now, s_next_report_tick)) {
            s_next_report_tick = now + pdMS_TO_TICKS(MB_TASK_REPORT_PERIOD_MS);
            motor_board_log_snapshot();
        }
        if (tick_due(now, s_next_stats_tick)) {
            s_next_stats_tick = now + pdMS_TO_TICKS(MB_TASK_STATS_PERIOD_MS);
            motor_board_log_stats();
        }
        vTaskDelay(pdMS_TO_TICKS(MB_TASK_POLL_PERIOD_MS));
    }
}

void motor_board_task_start(void)
{
    if (!MB_Transport_IsReady() || s_task_handle != NULL) {
        return;
    }
    if (xTaskCreate(motor_board_task, "motor_board", MB_TASK_STACK_WORDS,
                    NULL, MB_TASK_PRIORITY, &s_task_handle) != pdPASS) {
        s_task_handle = NULL;
        LOG_ERROR("[MOTOR_BOARD] task create failed");
    }
}
