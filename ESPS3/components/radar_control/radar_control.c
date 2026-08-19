#include "radar_control.h"

#include <stdbool.h>
#include <stdio.h>

#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "s3_ble.h"

/* These values match the existing M_CTR setup in main/radar/radar_uart.c. */
#define RADAR_CONTROL_PWM_MODE LEDC_LOW_SPEED_MODE
#define RADAR_CONTROL_PWM_CHANNEL LEDC_CHANNEL_0
#define RADAR_CONTROL_PWM_DUTY_RESOLUTION_BITS 10U
#define RADAR_CONTROL_PWM_DUTY_MAX \
    ((1U << RADAR_CONTROL_PWM_DUTY_RESOLUTION_BITS) - 1U)
#define RADAR_CONTROL_SPEED_PERCENT_BASE 100U
static const char *TAG = "RADAR_CONTROL";

static uint8_t s_speed_percent;
static uint8_t s_calibration_pwm;
static volatile bool s_imu_cal_done;
static volatile radar_control_state_t s_state = RADAR_CONTROL_WAIT_STM_QUERY;
static bool s_calibration_active;
static bool s_initialized;
static SemaphoreHandle_t s_lock;

static bool radar_control_lock(void)
{
    return s_lock != NULL &&
           xSemaphoreTake(s_lock, pdMS_TO_TICKS(20U)) == pdTRUE;
}

static void radar_control_unlock(void)
{
    if (s_lock != NULL) {
        (void)xSemaphoreGive(s_lock);
    }
}

static uint32_t speed_to_duty(uint8_t percent)
{
    return (RADAR_CONTROL_PWM_DUTY_MAX * (uint32_t)percent) /
           RADAR_CONTROL_SPEED_PERCENT_BASE;
}

static esp_err_t apply_speed(void)
{
    const uint32_t duty = speed_to_duty(s_speed_percent);
    esp_err_t ret = ledc_set_duty(RADAR_CONTROL_PWM_MODE,
                                  RADAR_CONTROL_PWM_CHANNEL,
                                  duty);
    if (ret == ESP_OK) {
        ret = ledc_update_duty(RADAR_CONTROL_PWM_MODE,
                               RADAR_CONTROL_PWM_CHANNEL);
    }
    return ret;
}

void radar_control_init(void)
{
    if (s_initialized) {
        return;
    }

    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        ESP_LOGE(TAG, "RADAR CONTROL mutex creation failed");
        return;
    }
    (void)xSemaphoreTake(s_lock, portMAX_DELAY);
    s_speed_percent = RADAR_MIN_SPEED;
    s_calibration_pwm = RADAR_MIN_SPEED;
    s_imu_cal_done = false;
    s_state = RADAR_CONTROL_WAIT_STM_QUERY;
    s_calibration_active = false;
    s_initialized = true;
    if (apply_speed() != ESP_OK) {
        ESP_LOGE(TAG, "M_CTR initial duty update failed");
    }
    (void)xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "RADAR CONTROL READY state=WAIT_STM_QUERY speed=%u",
             (unsigned)s_speed_percent);
}

bool radar_control_set_speed(uint8_t percent)
{
    bool accepted = false;

    ESP_LOGI(TAG, "input speed=%u", (unsigned)percent);
    ESP_LOGI(TAG, "calculated duty=%u",
             (unsigned)(percent <= RADAR_MAX_SPEED ? speed_to_duty(percent) : 0U));
    if (percent > RADAR_MAX_SPEED || !radar_control_lock()) {
        return false;
    }
    if (s_initialized && s_state == RADAR_CONTROL_RUNNING &&
        s_imu_cal_done && !s_calibration_active) {
        const uint8_t previous_speed = s_speed_percent;
        s_speed_percent = percent;
        if (apply_speed() == ESP_OK) {
            accepted = true;
        } else {
            s_speed_percent = previous_speed;
            (void)apply_speed();
        }
    }
    radar_control_unlock();
    return accepted;
}

void radar_control_set_imu_cal_done(bool done)
{
    if (!radar_control_lock()) {
        return;
    }
    if (!s_initialized) {
        radar_control_unlock();
        return;
    }

    if (!done) {
        if (s_state == RADAR_CONTROL_WAIT_STM_QUERY) {
            radar_control_unlock();
            return;
        }
        s_speed_percent = RADAR_MIN_SPEED;
        s_calibration_active = false;
        s_calibration_pwm = RADAR_MIN_SPEED;
        if (apply_speed() != ESP_OK) {
            ESP_LOGE(TAG, "IMU calibration reset PWM update failed");
        }
        s_imu_cal_done = false;
        s_state = RADAR_CONTROL_WAIT_IMU_CAL;
        radar_control_unlock();
        return;
    }

    /* A completion event is valid only after the accepted STM boot query has
     * released BOOT/WAIT_STM_QUERY. This rejects stale or out-of-order events. */
    if (s_state != RADAR_CONTROL_WAIT_IMU_CAL || s_imu_cal_done) {
        radar_control_unlock();
        return;
    }

    /* Calibration completion releases the App speed-control gate. */
    s_calibration_active = false;
    s_calibration_pwm = RADAR_MIN_SPEED;
    s_speed_percent = RADAR_MIN_SPEED;
    if (apply_speed() != ESP_OK) {
        /* Keep the controller non-running when the requested safe zero duty
         * could not be committed. */
        ESP_LOGE(TAG, "IMU calibration completion PWM update failed");
        radar_control_unlock();
        return;
    }
    s_imu_cal_done = true;
    s_state = RADAR_CONTROL_CAL_DONE;
    ESP_LOGI(TAG, "CAL_DONE_RECEIVED");
    /* Do not reintroduce the old automatic 0-to-100 percent ramp here. The
     * next accepted App command owns the first nonzero duty. */
    s_state = RADAR_CONTROL_RUNNING;
    ESP_LOGI(TAG, "RADAR CONTROL RUNNING speed=%u",
             (unsigned)s_speed_percent);
    radar_control_unlock();
}

bool radar_control_set_calibration_pwm(uint8_t percent)
{
    char line[48];
    uint8_t previous_speed;
    uint8_t previous_calibration_pwm;
    bool previous_calibration_active;

    (void)snprintf(line, sizeof(line), "RADAR SET PWM speed=%u",
                   (unsigned)percent);
    (void)s3_log_info(line);
    if (percent > RADAR_MAX_SPEED || !radar_control_lock()) {
        (void)s3_log_error("PWM RESULT FAIL");
        return false;
    }
    if (!s_initialized || s_state != RADAR_CONTROL_WAIT_IMU_CAL ||
        s_imu_cal_done) {
        radar_control_unlock();
        (void)s3_log_error("PWM RESULT FAIL");
        return false;
    }

    previous_speed = s_speed_percent;
    previous_calibration_pwm = s_calibration_pwm;
    previous_calibration_active = s_calibration_active;
    s_calibration_pwm = percent;
    s_speed_percent = percent;
    if (apply_speed() != ESP_OK) {
        s_speed_percent = previous_speed;
        s_calibration_pwm = previous_calibration_pwm;
        s_calibration_active = previous_calibration_active;
        (void)apply_speed();
        (void)s3_log_error("PWM RESULT FAIL");
        radar_control_unlock();
        return false;
    }
    s_calibration_active = true;
    (void)s3_log_info("PWM RESULT OK");
    radar_control_unlock();
    return true;
}

void radar_control_release_calibration_lock(void)
{
    if (!radar_control_lock()) {
        return;
    }
    s_calibration_active = false;
    radar_control_unlock();
}

bool radar_control_is_calibration_active(void)
{
    bool active = false;
    if (radar_control_lock()) {
        active = s_initialized && s_calibration_active;
        radar_control_unlock();
    }
    return active;
}

bool radar_control_is_running(void)
{
    bool running = false;
    if (radar_control_lock()) {
        running = s_initialized && s_state == RADAR_CONTROL_RUNNING;
        radar_control_unlock();
    }
    return running;
}

radar_control_state_t radar_control_get_state(void)
{
    radar_control_state_t state = RADAR_CONTROL_WAIT_STM_QUERY;
    if (radar_control_lock()) {
        state = s_state;
        radar_control_unlock();
    }
    return state;
}

uint8_t radar_control_get_speed(void)
{
    uint8_t speed = RADAR_MIN_SPEED;
    if (radar_control_lock()) {
        speed = s_speed_percent;
        radar_control_unlock();
    }
    return speed;
}

uint8_t radar_control_get_calibration_pwm(void)
{
    uint8_t pwm = RADAR_MIN_SPEED;
    if (radar_control_lock()) {
        pwm = s_calibration_pwm;
        radar_control_unlock();
    }
    return pwm;
}

void radar_control_handle_pwm_ready_query(void)
{
    if (!radar_control_lock()) {
        return;
    }
    if (s_initialized && s_state == RADAR_CONTROL_WAIT_STM_QUERY) {
        s_speed_percent = RADAR_MIN_SPEED;
        if (apply_speed() != ESP_OK) {
            ESP_LOGE(TAG, "BOOT zero-duty update failed");
            radar_control_unlock();
            return;
        }
        s_state = RADAR_CONTROL_WAIT_IMU_CAL;
        ESP_LOGI(TAG, "PWM_READY");
        ESP_LOGI(TAG, "WAIT_IMU_CAL");
    }
    radar_control_unlock();
}
