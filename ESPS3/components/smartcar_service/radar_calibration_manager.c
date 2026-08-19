#include "radar_calibration_manager.h"

#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "frame.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "radar_control.h"
#include "s3_ble.h"
#include "stm_uart.h"

static const char *TAG = "RADAR_CAL";
/* Vibration levels use the contract's zero-based index: 0..4 -> 20..100%.
 * PWM=0 is the separate pre-calibration synchronization level. */
static const uint8_t pwm_steps[] = {20U, 40U, 60U, 80U, 100U};

#define RADAR_CAL_ACK_TIMEOUT_MS UINT32_C(500)
#define RADAR_CAL_MAX_RETRY UINT8_C(3)
#define RADAR_CAL_STATIC_EVENT_TIMEOUT_MS UINT32_C(75000)
/* Keep the S3 event deadline aligned with the STM32 22 s per-level budget. */
#define RADAR_CAL_VIBRATION_EVENT_TIMEOUT_MS UINT32_C(22000)
#define RADAR_CAL_COMPLETE_TIMEOUT_MS UINT32_C(5000)
/* Start the guard when a vibration level is selected. Keep one second of
 * UART/task scheduling margin below STM32's 2 s settle plus 10 s window. */
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
/* All vibration levels use the same event id. This tick marks the current
 * level so a retransmitted previous id=2 cannot advance the next level. */
static TickType_t s_last_step_start_tick;

static uint64_t now_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

static void reset_tracking(void)
{
    s_state = RADAR_WAIT_SYNC;
    s_wait_event = RADAR_WAIT_STATIC_DONE;
    s_level_index = 0U;
    s_pwm = 0U;
    s_done = false;
    s_stm_boot_ready = false;
    s_ack_retry_count = 0U;
    s_ack_deadline_us = 0U;
    s_event_deadline_us = 0U;
    s_last_step_start_tick = 0U;
}

static void enter_sync_wait(const char *message)
{
    char line[96];
    const radar_calibration_state_t from_state = s_state;
    const radar_control_state_t control_state = radar_control_get_state();

    scbp_pending_tx_clear();
    reset_tracking();
    ESP_LOGI(TAG, "RADAR_WAIT_SYNC_ENTER from_state=%u reason=%s",
             (unsigned)from_state, message != NULL ? message : "unspecified");
    if (control_state != RADAR_CONTROL_WAIT_STM_QUERY) {
        radar_control_set_imu_cal_done(false);
        (void)radar_control_set_calibration_pwm(0U);
    }
    radar_control_release_calibration_lock();
    if (message != NULL) {
        ESP_LOGW(TAG, "%s; returning to WAIT_SYNC", message);
        (void)snprintf(line, sizeof(line), "RADAR CAL WAIT_SYNC %s", message);
        (void)s3_log_info(line);
    } else {
        (void)s3_log_info("RADAR CAL WAIT_SYNC");
    }
}

static bool send_ready_frame(void)
{
    uint8_t payload[1] = {s_pwm};
    uint8_t frame[SC_FRAME_MAX_SIZE];
    uint16_t frame_length = 0U;
    const int encode_ret =
        sc_frame_encode(SC_TYPE_RADAR_PWM_READY, payload, sizeof(payload),
                        frame, sizeof(frame), &frame_length);

    ESP_LOGI(TAG, "READY_ENCODE ret=%d length=%u", encode_ret,
             (unsigned)frame_length);
    if (encode_ret != 0) {
        return false;
    }
    const int sent = stm_uart_send(frame, frame_length);
    ESP_LOGI(TAG, "READY_UART_TX ret=%d bytes=%u", sent,
             (unsigned)frame_length);
    if (sent != (int)frame_length) {
        /* sc_frame_encode records the ACK transaction before the UART write;
         * never let a failed write be matched by a later stale ACK. */
        scbp_pending_tx_clear();
    }
    ESP_LOGI(TAG, "RADAR_PWM_READY speed=%u sent=%d", (unsigned)s_pwm, sent);
    char line[64];
    (void)snprintf(line, sizeof(line), "RADAR_PWM_READY TX speed=%u",
                   (unsigned)s_pwm);
    (void)s3_log_info(line);
    return sent == (int)frame_length;
}

static bool ensure_calibration_control_ready(void)
{
    /* A valid CAL_EVENT is an explicit recovery boundary. Re-arm the existing
     * radar-control gate instead of silently rejecting a stale local state. */
    if (!radar_control_is_calibration_active()) {
        radar_control_set_imu_cal_done(false);
        radar_control_handle_pwm_ready_query();
    }
    return radar_control_get_state() == RADAR_CONTROL_WAIT_IMU_CAL ||
           radar_control_is_calibration_active();
}

static bool send_complete_event(void)
{
    const uint8_t event_id = SC_CAL_EVENT_COMPLETE;
    uint8_t frame[SC_FRAME_MAX_SIZE];
    uint16_t frame_length = 0U;
    const int encode_ret = sc_frame_encode(SC_TYPE_CAL_EVENT, &event_id, 1U,
                                           frame, sizeof(frame),
                                           &frame_length);
    int sent;

    if (encode_ret != 0) {
        ESP_LOGE(TAG, "CAL_EVENT_TX id=3 encode=%d", encode_ret);
        return false;
    }
    sent = stm_uart_send(frame, frame_length);
    ESP_LOGI(TAG, "CAL_EVENT_TX id=3 sent=%d", sent);
    (void)s3_log_info("CAL_EVENT_TX id=3 COMPLETE");
    return sent == (int)frame_length;
}

static void arm_ready_wait(void)
{
    s_state = RADAR_WAIT_ACK;
    s_ack_retry_count = 0U;
    s_ack_deadline_us = now_us() +
                        ((uint64_t)RADAR_CAL_ACK_TIMEOUT_MS * 1000ULL);
    /* The event kind is selected by the transition that armed this level;
     * index zero is a valid vibration level after STATIC_CAL_DONE. */
    s_event_deadline_us = now_us() +
                          ((uint64_t)(s_wait_event == RADAR_WAIT_STATIC_DONE
                                          ? RADAR_CAL_STATIC_EVENT_TIMEOUT_MS
                                          : RADAR_CAL_VIBRATION_EVENT_TIMEOUT_MS) *
                           1000ULL);
    (void)s3_log_info("WAIT_RADAR_ACK");
}

static bool start_current_level(void)
{
    const radar_control_state_t before_state = radar_control_get_state();
    const bool pwm_applied = radar_control_set_calibration_pwm(s_pwm);
    const radar_control_state_t after_state = radar_control_get_state();
    char line[48];

    (void)snprintf(line, sizeof(line), "RADAR CAL PWM=%u", (unsigned)s_pwm);
    (void)s3_log_info(line);
    ESP_LOGI(TAG, "RADAR_PWM_APPLY before=%u after=%u ret=%u",
             (unsigned)before_state, (unsigned)after_state,
             (unsigned)pwm_applied);
    if (!pwm_applied) {
        return false;
    }
    arm_ready_wait();
    return send_ready_frame();
}

static bool advance_to_next_level(void)
{
    if (s_level_index + 1U >= sizeof(pwm_steps)) {
        s_state = RADAR_CAL_DONE;
        s_done = true;
        ESP_LOGI(TAG, "RADAR_CAL_COMPLETE pwm=%u", (unsigned)s_pwm);
        return true;
    }
    ++s_level_index;
    s_pwm = pwm_steps[s_level_index];
    s_last_step_start_tick = xTaskGetTickCount();
    s_state = RADAR_SET_PWM;
    return start_current_level();
}

static bool vibration_step_time_protected(void)
{
    const TickType_t guard_ticks =
        pdMS_TO_TICKS(RADAR_CAL_VIBRATION_EVENT_NOT_BEFORE_MS);

    return guard_ticks != 0U &&
           (TickType_t)(xTaskGetTickCount() - s_last_step_start_tick) <
               guard_ticks;
}

void radar_calibration_manager_init(void)
{
    reset_tracking();
    s_initialized = true;
    ESP_LOGI(TAG, "RADAR_MANAGER_INIT state=%u", (unsigned)s_state);
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
            if (!send_ready_frame()) {
                enter_sync_wait("RADAR_PWM_READY TX failed");
                return;
            }
            s_ack_deadline_us = now_us() +
                                ((uint64_t)RADAR_CAL_ACK_TIMEOUT_MS * 1000ULL);
        } else {
            enter_sync_wait("RADAR_PWM_ACK timeout");
        }
        return;
    }
    if (s_state == RADAR_WAIT_EVENT &&
        now_us() >= s_event_deadline_us) {
        enter_sync_wait("CAL event timeout");
        return;
    }
    switch (s_state) {
    case RADAR_WAIT_SYNC:
        break;
    case RADAR_SET_PWM:
        if (!start_current_level()) {
            enter_sync_wait("RADAR_PWM_READY TX failed");
        }
        break;
    case RADAR_NEXT_LEVEL:
        if (!advance_to_next_level()) {
            enter_sync_wait("RADAR_PWM_READY TX failed");
        }
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
        const radar_calibration_state_t old_state = s_state;
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
        if (s_state != RADAR_WAIT_SYNC || s_stm_boot_ready) {
            /* STM emits BOOT_READY again only for a fresh sync or a recovery
             * boundary. Reset the S3-side transaction before consuming it. */
            enter_sync_wait("STM_BOOT_READY recovery");
        }
        s_stm_boot_ready = true;
        /* BOOT_READY is the synchronization boundary. The radar controller
         * leaves BOOT/WAIT_STM_QUERY and the manager sends PWM=0 next. */
        radar_control_handle_pwm_ready_query();
        s_state = RADAR_SET_PWM;
        ESP_LOGI(TAG,
                 "RADAR_BOOT_ACCEPT old_state=%u new_state=%u pwm=%u",
                 (unsigned)old_state, (unsigned)s_state, (unsigned)s_pwm);
        (void)s3_log_info("RADAR CAL START");
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
            enter_sync_wait("RADAR_PWM_ACK rejected");
            return;
        }
        s_state = RADAR_WAIT_EVENT;
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

        if (event_id == SC_CAL_EVENT_STATIC_CAL_DONE) {
            if (s_state != RADAR_WAIT_EVENT ||
                s_wait_event != RADAR_WAIT_STATIC_DONE || s_pwm != 0U) {
                /* command_bridge already returned the transport ACK. Do not
                 * replay the static transition for a stale id=1 retry. */
                ESP_LOGW(TAG, "CAL_EVENT_DUPLICATE id=1 ignored state=%u pwm=%u",
                         (unsigned)s_state, (unsigned)s_pwm);
                return;
            }
            s_stm_boot_ready = true;
            if (!ensure_calibration_control_ready()) {
                ESP_LOGW(TAG, "CAL_EVENT id=1 control rearm pending");
            }
            /* Index 0 is the first vibration level (20%). The zero-duty
             * synchronization level is not part of the vibration index. */
            s_level_index = 0U;
            s_last_step_start_tick = xTaskGetTickCount();
            s_pwm = pwm_steps[s_level_index];
            s_state = RADAR_SET_PWM;
            s_wait_event = RADAR_WAIT_VIBRATION_DONE;
            ESP_LOGI(TAG, "CAL_EVENT_ACCEPT id=1 pwm=%u index=0",
                     (unsigned)s_pwm);
            (void)s3_log_info("CAL_EVENT_ACCEPT id=1");
            if (!start_current_level()) {
                enter_sync_wait("RADAR_PWM_READY TX failed after id=1");
            }
            return;
        }
        if (event_id == SC_CAL_EVENT_VIBRATION_STEP_DONE) {
            if (s_done) {
                ESP_LOGI(TAG, "VIB Step ignored after completion");
                return;
            }
            if (s_state != RADAR_WAIT_EVENT ||
                s_wait_event != RADAR_WAIT_VIBRATION_DONE) {
                /* The transport layer re-ACKs a retry, but the calibration
                 * state advances only from the matching wait state. */
                ESP_LOGW(TAG, "CAL_EVENT_DUPLICATE id=2 ignored state=%u index=%u",
                         (unsigned)s_state, (unsigned)s_level_index);
                return;
            }
            if (vibration_step_time_protected()) {
                /* id=2 is reused for every level. A retry from the prior
                 * level must not be mistaken for the current completion. */
                ESP_LOGW(TAG,
                         "CAL_EVENT_DUPLICATE id=2 time-protected index=%u",
                         (unsigned)s_level_index);
                return;
            }
            s_stm_boot_ready = true;
            if (!ensure_calibration_control_ready()) {
                ESP_LOGW(TAG, "CAL_EVENT id=2 control rearm pending");
            }
            /* Every vibration completion uses id=2. The level index, not the
             * event ID, identifies progress through 20/40/60/80/100%. */
            ++s_level_index;
            if (s_level_index >= sizeof(pwm_steps)) {
                radar_control_set_imu_cal_done(true);
                if (!radar_control_is_running()) {
                    enter_sync_wait("radar control completion failed");
                    return;
                }
                s_state = RADAR_CAL_DONE;
                s_done = true;
                s_pwm = 0U;
                ESP_LOGI(TAG, "RADAR_CAL_COMPLETE pwm=0");
                (void)s3_log_info("RADAR_CAL_COMPLETE pwm=0");
                if (!send_complete_event()) {
                    ESP_LOGW(TAG, "CAL_EVENT_TX id=3 failed");
                }
                return;
            }
            s_pwm = pwm_steps[s_level_index];
            s_state = RADAR_SET_PWM;
            s_wait_event = RADAR_WAIT_VIBRATION_DONE;
            ESP_LOGI(TAG, "VIB Step %d Done -> Next Speed=%d",
                     (int)s_level_index - 1, (int)s_pwm);
            ESP_LOGI(TAG, "CAL_EVENT_ACCEPT id=2 pwm=%u index=%u",
                     (unsigned)s_pwm, (unsigned)s_level_index);
            if (!start_current_level()) {
                enter_sync_wait("RADAR_PWM_READY TX failed after id=2");
            }
            return;
        }
        if (event_id == SC_CAL_EVENT_COMPLETE) {
            if (s_done) {
                ESP_LOGI(TAG, "CAL_EVENT_COMPLETE duplicate");
                return;
            }
            s_stm_boot_ready = true;
            if (!ensure_calibration_control_ready()) {
                ESP_LOGW(TAG, "CAL_EVENT id=3 control rearm pending");
            }
            s_pwm = 100U;
            if (!radar_control_set_calibration_pwm(s_pwm)) {
                enter_sync_wait("final PWM apply failed");
                return;
            }
            radar_control_release_calibration_lock();
            radar_control_set_imu_cal_done(true);
            if (!radar_control_is_running()) {
                enter_sync_wait("radar control completion failed");
                return;
            }
            s_state = RADAR_CAL_DONE;
            s_done = true;
            ESP_LOGI(TAG, "RADAR_CAL_COMPLETE pwm=100");
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
