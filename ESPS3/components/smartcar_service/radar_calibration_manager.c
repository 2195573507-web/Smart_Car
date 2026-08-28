#include "radar_calibration_manager.h"

#include <stdio.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "radar_control.h"
#include "s3_ble.h"

static const char *TAG = "RADAR_CAL";

#define RADAR_CAL_STATIC_EVENT_TIMEOUT_MS UINT32_C(75000)
#define STM_BOOT_STATE_WAIT_RADAR_ZERO UINT8_C(1)

static radar_calibration_state_t s_state;
static uint8_t s_pwm;
static bool s_initialized;
static bool s_done;
static bool s_stm_boot_ready;
static uint64_t s_static_event_deadline_us;
static radar_calibration_send_ready_t s_send_ready;
static void *s_transport_context;

static uint64_t now_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

static void reset_tracking(void)
{
    s_state = RADAR_WAIT_SYNC;
    s_pwm = 0U;
    s_done = false;
    s_stm_boot_ready = false;
    s_static_event_deadline_us = 0U;
}

static void enter_sync_wait(const char *reason)
{
    const radar_calibration_state_t from_state = s_state;

    reset_tracking();
    if (radar_control_get_state() != RADAR_CONTROL_WAIT_STM_QUERY) {
        radar_control_set_imu_cal_done(false);
        (void)radar_control_set_calibration_pwm(0U);
    }
    radar_control_release_calibration_lock();
    ESP_LOGW(TAG, "RADAR_WAIT_SYNC from=%u reason=%s", (unsigned)from_state,
             reason != NULL ? reason : "unspecified");
    (void)s3_log_info("RADAR CAL WAIT_SYNC");
}

static bool start_zero_pwm(void)
{
    if (s_send_ready == NULL || !radar_control_set_calibration_pwm(0U)) {
        return false;
    }
    s_pwm = 0U;
    s_state = RADAR_WAIT_ACK;
    s_static_event_deadline_us = now_us() +
        ((uint64_t)RADAR_CAL_STATIC_EVENT_TIMEOUT_MS * UINT64_C(1000));
    if (s_send_ready(s_pwm, s_transport_context) != 0) {
        return false;
    }
    ESP_LOGI(TAG, "RADAR_PWM_READY TX speed=0");
    return true;
}

void radar_calibration_manager_init(void)
{
    reset_tracking();
    s_initialized = true;
    (void)s3_log_info("RADAR CAL WAIT STM_BOOT_READY");
}

void radar_calibration_manager_set_transport(radar_calibration_send_ready_t send_ready,
                                             void *context)
{
    s_send_ready = send_ready;
    s_transport_context = context;
}

void radar_calibration_manager_step(void)
{
    if (!s_initialized || s_done || !s_stm_boot_ready) {
        return;
    }
    if (s_state == RADAR_SET_PWM) {
        if (!start_zero_pwm()) {
            enter_sync_wait("RADAR_PWM_READY send failed");
        }
        return;
    }
    if ((s_state == RADAR_WAIT_ACK || s_state == RADAR_WAIT_STATIC_EVENT) &&
        now_us() >= s_static_event_deadline_us) {
        enter_sync_wait("STATIC_CAL_DONE timeout");
    }
}

bool radar_calibration_manager_on_boot_ready(const uint8_t *payload, uint8_t length)
{
    if (!s_initialized || payload == NULL ||
        length != SRP_PAYLOAD_BOOT_READY_SIZE ||
        payload[0] != STM_BOOT_STATE_WAIT_RADAR_ZERO || payload[1] != 0U) {
        return false;
    }
    if (s_stm_boot_ready) {
        /* BOOT_READY is retransmitted until its transport ACK arrives. */
        return true;
    }
    if (s_state != RADAR_WAIT_SYNC) {
        return false;
    }
    s_stm_boot_ready = true;
    radar_control_handle_pwm_ready_query();
    s_state = RADAR_SET_PWM;
    (void)s3_log_info("RADAR CAL START");
    return true;
}

bool radar_calibration_manager_on_cal_event(const uint8_t *payload, uint8_t length)
{
    if (!s_initialized || payload == NULL ||
        length != SRP_PAYLOAD_CAL_EVENT_SIZE ||
        payload[0] != SRP_CAL_EVENT_STATIC_DONE) {
        return false;
    }
    if (s_done) {
        return true;
    }
    if (s_state != RADAR_WAIT_STATIC_EVENT || s_pwm != 0U) {
        return false;
    }
    radar_control_release_calibration_lock();
    radar_control_set_imu_cal_done(true);
    if (!radar_control_is_running()) {
        enter_sync_wait("radar control completion failed");
        return false;
    }
    s_state = RADAR_CAL_DONE;
    s_done = true;
    (void)s3_log_info("STATIC_CAL_DONE accepted; radar control released");
    return true;
}

void radar_calibration_manager_on_ready_response(srp_link_tx_result_t result,
                                                 uint8_t status_code)
{
    if (!s_initialized || s_state != RADAR_WAIT_ACK) {
        return;
    }
    if (result == SRP_LINK_TX_OK && status_code == SRP_FAST_RESP_OK) {
        s_state = RADAR_WAIT_STATIC_EVENT;
        (void)s3_log_info("RADAR_PWM_READY ACK; WAIT STATIC_CAL_DONE");
        return;
    }
    enter_sync_wait("RADAR_PWM_READY transaction failed");
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
