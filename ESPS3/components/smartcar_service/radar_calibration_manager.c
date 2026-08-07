#include "radar_calibration_manager.h"

#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "frame.h"
#include "radar_control.h"
#include "s3_ble.h"
#include "stm_uart.h"

static const char *TAG = "RADAR_CAL";
static const uint8_t s_levels[] = {0U, 20U, 40U, 60U, 80U, 100U};

#define RADAR_CAL_ACK_TIMEOUT_MS UINT32_C(500)
#define RADAR_CAL_MAX_RETRY UINT8_C(3)
#define RADAR_CAL_STATIC_EVENT_TIMEOUT_MS UINT32_C(75000)
#define RADAR_CAL_VIBRATION_EVENT_TIMEOUT_MS UINT32_C(25000)
#define RADAR_CAL_COMPLETE_TIMEOUT_MS UINT32_C(5000)
/* Reject prior-stage id=2 retries while leaving margin for the first sample
 * at the 2-second boundary. STM still enforces 2 seconds + 1000 samples. */
#define RADAR_CAL_VIBRATION_EVENT_NOT_BEFORE_MS UINT32_C(11000)
#define STM_BOOT_STATE_WAIT_RADAR_ZERO UINT8_C(1)

typedef enum
{
    RADAR_WAIT_STATIC_DONE = 0,
    RADAR_WAIT_VIBRATION_DONE,
    RADAR_WAIT_CAL_COMPLETE
} radar_wait_event_t;

static radar_calibration_state_t s_state;
static radar_wait_event_t s_wait_event;
static uint8_t s_level_index;
static uint8_t s_pwm;
static bool s_initialized;
static bool s_done;
static bool s_stm_boot_ready;
static uint8_t s_ack_retry_count;
static uint64_t s_ack_deadline_us;
static uint64_t s_event_deadline_us;
static uint64_t s_event_not_before_us;
static uint8_t s_last_ack_event_id;
static bool s_last_ack_valid;
static bool s_early_event_logged;

static uint64_t now_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

static void enter_error(const char *message)
{
    char line[96];

    s_state = RADAR_CAL_ERROR;
    s_done = false;
    s_stm_boot_ready = false;
    (void)radar_control_set_calibration_pwm(0U);
    if (message != NULL) {
        ESP_LOGE(TAG, "%s", message);
        (void)snprintf(line, sizeof(line), "RADAR CAL ERROR %s", message);
        (void)s3_log_error(line);
    } else {
        (void)s3_log_error("RADAR CAL ERROR");
    }
}

static bool send_ready_frame(void)
{
    uint8_t payload[1] = {s_pwm};
    uint8_t frame[SC_FRAME_MAX_SIZE];
    uint16_t frame_length = 0U;

    if (sc_frame_encode(SC_TYPE_RADAR_PWM_READY, payload, sizeof(payload),
                        frame, sizeof(frame), &frame_length) != 0) {
        return false;
    }
    const int sent = stm_uart_send(frame, frame_length);
    ESP_LOGI(TAG, "RADAR_PWM_READY speed=%u sent=%d", (unsigned)s_pwm, sent);
    char line[64];
    (void)snprintf(line, sizeof(line), "RADAR_PWM_READY TX speed=%u",
                   (unsigned)s_pwm);
    (void)s3_log_info(line);
    return sent == (int)frame_length;
}

static bool send_event_ack(uint8_t event_id, uint8_t result)
{
    uint8_t payload[2] = {event_id, result};
    uint8_t frame[SC_FRAME_MAX_SIZE];
    uint16_t frame_length = 0U;

    ESP_LOGI(TAG, "CAL_ACK_BUILD id=%d result=%d",
             (int)event_id, (int)result);
    if (sc_frame_encode(SC_TYPE_CAL_EVENT_ACK, payload, sizeof(payload),
                        frame, sizeof(frame), &frame_length) != 0) {
        return false;
    }
    const int sent = stm_uart_send(frame, frame_length);
    ESP_LOGI(TAG, "CAL_ACK_TX id=%d result=%d sent=%d",
             (int)event_id, (int)result, sent);
    {
        char line[64];
        (void)snprintf(line, sizeof(line), "CAL_ACK_TX id=%d result=%d",
                       (int)event_id, (int)result);
        (void)s3_log_info(line);
    }
    return sent == (int)frame_length;
}

static void arm_ready_wait(void)
{
    s_state = RADAR_WAIT_ACK;
    s_ack_retry_count = 0U;
    s_ack_deadline_us = now_us() +
                        ((uint64_t)RADAR_CAL_ACK_TIMEOUT_MS * 1000ULL);
    s_wait_event = s_level_index == 0U ? RADAR_WAIT_STATIC_DONE
                                       : RADAR_WAIT_VIBRATION_DONE;
    s_event_deadline_us = now_us() +
                          ((uint64_t)(s_level_index == 0U
                                          ? RADAR_CAL_STATIC_EVENT_TIMEOUT_MS
                                          : RADAR_CAL_VIBRATION_EVENT_TIMEOUT_MS) *
                           1000ULL);
    (void)s3_log_info("WAIT_RADAR_ACK");
}

void radar_calibration_manager_init(void)
{
    s_state = RADAR_CAL_IDLE;
    s_wait_event = RADAR_WAIT_STATIC_DONE;
    s_level_index = 0U;
    s_pwm = 0U;
    s_done = false;
    s_stm_boot_ready = false;
    s_ack_retry_count = 0U;
    s_ack_deadline_us = 0U;
    s_event_deadline_us = 0U;
    s_event_not_before_us = 0U;
    s_last_ack_event_id = 0U;
    s_last_ack_valid = false;
    s_early_event_logged = false;
    s_initialized = true;
    ESP_LOGI(TAG, "IMU calibration manager start");
    (void)s3_log_info("RADAR CAL WAIT STM_BOOT_READY");
}

void radar_calibration_manager_step(void)
{
    if (!s_initialized || s_done) {
        return;
    }
    if (!s_stm_boot_ready) {
        return;
    }
    if (s_state == RADAR_WAIT_ACK &&
        now_us() >= s_ack_deadline_us) {
        if (s_ack_retry_count < RADAR_CAL_MAX_RETRY) {
            ++s_ack_retry_count;
            (void)send_ready_frame();
            s_ack_deadline_us = now_us() +
                                ((uint64_t)RADAR_CAL_ACK_TIMEOUT_MS * 1000ULL);
        } else {
            enter_error("RADAR_PWM_ACK timeout");
        }
        return;
    }
    if (s_state == RADAR_WAIT_EVENT &&
        now_us() >= s_event_deadline_us) {
        enter_error("CAL event timeout");
        return;
    }
    switch (s_state) {
    case RADAR_CAL_IDLE:
        s_state = RADAR_SET_PWM;
        break;
    case RADAR_SET_PWM:
        {
            char line[48];
            (void)snprintf(line, sizeof(line), "RADAR CAL PWM=%u",
                           (unsigned)s_pwm);
            (void)s3_log_info(line);
        }
        if (!radar_control_set_calibration_pwm(s_pwm)) {
            enter_error("PWM apply failed");
            return;
        }
        arm_ready_wait();
        (void)send_ready_frame();
        break;
    case RADAR_NEXT_LEVEL:
        if (s_level_index + 1U >= sizeof(s_levels)) {
            s_state = RADAR_CAL_DONE;
            s_done = true;
            ESP_LOGI(TAG, "RADAR_CAL_COMPLETE pwm=%u", (unsigned)s_pwm);
            return;
        }
        ++s_level_index;
        s_pwm = s_levels[s_level_index];
        s_state = RADAR_SET_PWM;
        break;
    case RADAR_WAIT_ACK:
    case RADAR_WAIT_EVENT:
    case RADAR_CAL_DONE:
    case RADAR_CAL_ERROR:
    default:
        break;
    }
}

void radar_calibration_manager_on_frame(uint8_t type,
                                        const uint8_t *payload,
                                        uint16_t length)
{
    if (!s_initialized || payload == NULL) {
        return;
    }
    if (type == SC_TYPE_STM_BOOT_READY && length == 2U) {
        const uint8_t state = payload[0];
        const uint8_t result = payload[1];
        char line[80];

        (void)snprintf(line, sizeof(line),
                       "STM_BOOT_READY RX state=%u result=%u",
                       (unsigned)state, (unsigned)result);
        (void)s3_log_info(line);
        if (state != STM_BOOT_STATE_WAIT_RADAR_ZERO || result != 0U) {
            (void)snprintf(line, sizeof(line),
                           "STM_BOOT_READY rejected state=%u result=%u",
                           (unsigned)state, (unsigned)result);
            (void)s3_log_error(line);
            return;
        }
        if (!s_stm_boot_ready) {
            s_stm_boot_ready = true;
            (void)s3_log_info("RADAR CAL START");
        }
        return;
    }
    if (type == SC_TYPE_RADAR_PWM_ACK && length == 2U &&
        s_state == RADAR_WAIT_ACK) {
        const uint8_t speed = payload[0];
        const uint8_t result = payload[1];
        char line[64];
        (void)snprintf(line, sizeof(line),
                       "RADAR_PWM_ACK RX speed=%u result=%u",
                       (unsigned)speed, (unsigned)result);
        (void)s3_log_info(line);
        if (speed != s_pwm || result != 0U) {
            enter_error("RADAR_PWM_ACK rejected");
            return;
        }
        s_state = RADAR_WAIT_EVENT;
        s_early_event_logged = false;
        s_event_not_before_us =
            s_level_index == 0U
                ? 0U
                : now_us() +
                      ((uint64_t)RADAR_CAL_VIBRATION_EVENT_NOT_BEFORE_MS *
                       1000ULL);
        s_event_deadline_us = now_us() +
                              ((uint64_t)(s_level_index == 0U
                                              ? RADAR_CAL_STATIC_EVENT_TIMEOUT_MS
                                              : RADAR_CAL_VIBRATION_EVENT_TIMEOUT_MS) *
                               1000ULL);
        ESP_LOGI(TAG, "RADAR_PWM_ACK speed=%u", (unsigned)speed);
        return;
    }
    if (type == SC_TYPE_CAL_EVENT && length == 1U) {
        const uint8_t event_id = payload[0];
        char line[48];

        (void)snprintf(line, sizeof(line), "CAL_EVENT_RX id=%u",
                       (unsigned)event_id);
        ESP_LOGI(TAG, "%s", line);
        (void)s3_log_info(line);

        if (s_wait_event == RADAR_WAIT_VIBRATION_DONE &&
            event_id == SC_CAL_EVENT_VIBRATION_STEP_DONE &&
            s_state == RADAR_WAIT_EVENT &&
            now_us() < s_event_not_before_us) {
            if (!s_early_event_logged) {
                ESP_LOGW(TAG, "CAL_EVENT_DUPLICATE id=2 pwm=%u reacked",
                         (unsigned)s_pwm);
                (void)s3_log_info("CAL_EVENT_DUPLICATE id=2 reacked");
                s_early_event_logged = true;
            }
            (void)send_event_ack(event_id, CAL_ACK_OK);
            return;
        }

        if (s_wait_event == RADAR_WAIT_STATIC_DONE &&
            event_id == SC_CAL_EVENT_STATIC_CAL_DONE &&
            s_pwm == RADAR_MIN_SPEED &&
            s_state == RADAR_WAIT_EVENT &&
            (!s_last_ack_valid || s_last_ack_event_id != event_id)) {
            if (!send_event_ack(event_id, CAL_ACK_OK)) {
                return;
            }
            ESP_LOGI(TAG, "CAL_EVENT_ACCEPT id=1");
            (void)s3_log_info("CAL_EVENT_ACCEPT id=1");
            s_last_ack_event_id = event_id;
            s_last_ack_valid = true;
            s_stm_boot_ready = true;
            s_state = RADAR_NEXT_LEVEL;
            return;
        }
        if (s_wait_event == RADAR_WAIT_VIBRATION_DONE &&
            event_id == SC_CAL_EVENT_VIBRATION_STEP_DONE &&
            s_state == RADAR_WAIT_EVENT &&
            now_us() >= s_event_not_before_us) {
            if (!send_event_ack(event_id, CAL_ACK_OK)) {
                return;
            }
            s_last_ack_event_id = event_id;
            s_last_ack_valid = true;
            if (s_level_index + 1U >= sizeof(s_levels)) {
                s_wait_event = RADAR_WAIT_CAL_COMPLETE;
                s_event_deadline_us = now_us() +
                                      ((uint64_t)RADAR_CAL_COMPLETE_TIMEOUT_MS *
                                       1000ULL);
            } else {
                s_state = RADAR_NEXT_LEVEL;
            }
            return;
        }
        if (s_wait_event == RADAR_WAIT_CAL_COMPLETE &&
            event_id == SC_CAL_EVENT_COMPLETE &&
            s_state == RADAR_WAIT_EVENT) {
            if (!send_event_ack(event_id, CAL_ACK_OK)) {
                return;
            }
            s_last_ack_event_id = event_id;
            s_last_ack_valid = true;
            s_pwm = 100U;
            if (!radar_control_set_calibration_pwm(s_pwm)) {
                enter_error("final PWM apply failed");
                return;
            }
            radar_control_release_calibration_lock();
            s_state = RADAR_CAL_DONE;
            s_done = true;
            ESP_LOGI(TAG, "RADAR_CAL_COMPLETE pwm=100");
            return;
        }
        if (s_last_ack_valid && s_last_ack_event_id == event_id) {
            (void)send_event_ack(event_id, CAL_ACK_OK);
            return;
        }
        {
            const bool radar_calibration_active =
                radar_control_is_calibration_active();
            const char *reason = "event invalid";
            char error_line[96];

            if (!s_initialized) {
                reason = "radar not initialized";
            } else if (!s_stm_boot_ready) {
                reason = "STM_BOOT_READY not set";
            } else if (s_state != RADAR_WAIT_EVENT) {
                reason = "invalid state";
            } else if (s_pwm > RADAR_MAX_SPEED ||
                       !radar_calibration_active) {
                reason = "pwm unavailable";
            }
            ESP_LOGE(TAG,
                     "CAL_ACK_ERROR reason=%s event_id=%u state=%u "
                     "wait_event=%u stm_boot_ready=%u pwm=%u radar_active=%u",
                     reason, (unsigned)event_id, (unsigned)s_state,
                     (unsigned)s_wait_event, (unsigned)s_stm_boot_ready,
                     (unsigned)s_pwm, (unsigned)radar_calibration_active);
            (void)snprintf(error_line, sizeof(error_line),
                           "CAL_ACK_ERROR reason=%s event=%u state=%u wait=%u",
                           reason, (unsigned)event_id, (unsigned)s_state,
                           (unsigned)s_wait_event);
            (void)s3_log_error(error_line);
        }
        (void)send_event_ack(event_id, CAL_ACK_ERROR);
    }
}

radar_calibration_state_t radar_calibration_manager_get_state(void)
{
    return s_state;
}

uint8_t radar_calibration_manager_get_pwm(void)
{
    return s_pwm;
}

bool radar_calibration_manager_is_done(void)
{
    return s_done;
}
