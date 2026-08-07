#include "radar_control.h"

#include <stdbool.h>
#include <stdio.h>

#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "s3_ble.h"

/* These values match the existing M_CTR setup in main/radar/radar_uart.c. */
#define RADAR_CONTROL_PWM_MODE LEDC_LOW_SPEED_MODE
#define RADAR_CONTROL_PWM_CHANNEL LEDC_CHANNEL_0
#define RADAR_CONTROL_PWM_DUTY_RESOLUTION_BITS 10U
#define RADAR_CONTROL_PWM_DUTY_MAX \
    ((1U << RADAR_CONTROL_PWM_DUTY_RESOLUTION_BITS) - 1U)
#define RADAR_CONTROL_SPEED_PERCENT_BASE 100U
#define RADAR_CONTROL_SOFT_START_STEP_PERCENT 5U
#define RADAR_CONTROL_SOFT_START_INTERVAL_MS 500U
#define RADAR_CONTROL_TASK_STACK_SIZE 2048U
#define RADAR_CONTROL_TASK_PRIORITY 6U

static const char *TAG = "RADAR_CONTROL";

static uint8_t s_speed_percent;
static uint8_t s_calibration_pwm;
static volatile bool s_imu_cal_done;
static volatile radar_control_state_t s_state = RADAR_CONTROL_WAIT_STM_QUERY;
static bool s_calibration_active;
static bool s_initialized;
static TaskHandle_t s_task;

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

static void radar_control_task(void *context)
{
    (void)context;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(RADAR_CONTROL_SOFT_START_INTERVAL_MS));

        if (s_state == RADAR_CONTROL_WAIT_IMU_CAL) {
            if (s_imu_cal_done) {
                s_state = RADAR_CONTROL_SOFT_START;
                ESP_LOGI(TAG, "RAMP_START");
            }
            continue;
        }

        if (s_state != RADAR_CONTROL_SOFT_START) {
            continue;
        }

        if (!s_imu_cal_done) {
            s_speed_percent = RADAR_MIN_SPEED;
            (void)apply_speed();
            s_state = RADAR_CONTROL_WAIT_IMU_CAL;
            continue;
        }

        if (s_speed_percent < RADAR_MAX_SPEED) {
            uint16_t next_speed = (uint16_t)s_speed_percent +
                                  RADAR_CONTROL_SOFT_START_STEP_PERCENT;
            if (next_speed > RADAR_MAX_SPEED) {
                next_speed = RADAR_MAX_SPEED;
            }
            s_speed_percent = (uint8_t)next_speed;
            (void)apply_speed();
            ESP_LOGI(TAG, "PWM=%u%%", (unsigned)s_speed_percent);
        }

        if (s_speed_percent >= RADAR_MAX_SPEED) {
            s_state = RADAR_CONTROL_RUNNING;
            ESP_LOGI(TAG, "RADAR CONTROL RUNNING speed=%u",
                     (unsigned)s_speed_percent);
        }
    }
}

void radar_control_init(void)
{
    if (s_initialized) {
        return;
    }

    s_speed_percent = RADAR_MIN_SPEED;
    s_imu_cal_done = false;
    s_state = RADAR_CONTROL_WAIT_STM_QUERY;
    s_initialized = true;
    if (apply_speed() != ESP_OK) {
        ESP_LOGE(TAG, "M_CTR initial duty update failed");
    }
    if (xTaskCreate(radar_control_task,
                    "radar_control",
                    RADAR_CONTROL_TASK_STACK_SIZE,
                    NULL,
                    RADAR_CONTROL_TASK_PRIORITY,
                    &s_task) != pdPASS) {
        s_task = NULL;
        ESP_LOGE(TAG, "RADAR CONTROL task creation failed");
        return;
    }
    ESP_LOGI(TAG, "RADAR CONTROL READY state=WAIT_STM_QUERY speed=%u",
             (unsigned)s_speed_percent);
}

void radar_control_set_speed(uint8_t percent)
{
    const uint8_t input_speed = percent;
    if (percent > RADAR_MAX_SPEED) {
        percent = RADAR_MAX_SPEED;
    }

    ESP_LOGI(TAG, "input speed=%u", (unsigned)input_speed);
    ESP_LOGI(TAG, "calculated duty=%u",
             (unsigned)speed_to_duty(percent));
    if (s_initialized && s_state == RADAR_CONTROL_RUNNING &&
        !s_calibration_active) {
        s_speed_percent = percent;
        apply_speed();
    }
}

void radar_control_set_imu_cal_done(bool done)
{
    const bool was_done = s_imu_cal_done;

    s_imu_cal_done = done;
    if (!s_initialized) {
        return;
    }

    if (!done) {
        if (s_state == RADAR_CONTROL_WAIT_STM_QUERY) {
            return;
        }
        s_speed_percent = RADAR_MIN_SPEED;
        s_calibration_active = false;
        s_calibration_pwm = RADAR_MIN_SPEED;
        (void)apply_speed();
        s_state = RADAR_CONTROL_WAIT_IMU_CAL;
        return;
    }
    if (was_done) {
        return;
    }

    /* Calibration completion releases the startup gate. The normal soft-start
     * then owns the motor again. */
    s_calibration_active = false;
    s_calibration_pwm = RADAR_MIN_SPEED;
    s_speed_percent = RADAR_MIN_SPEED;
    (void)apply_speed();
    if (s_state == RADAR_CONTROL_WAIT_IMU_CAL) {
        s_state = RADAR_CONTROL_SOFT_START;
        ESP_LOGI(TAG, "CAL_DONE_RECEIVED");
    }
}

bool radar_control_set_calibration_pwm(uint8_t percent)
{
    char line[48];

    (void)snprintf(line, sizeof(line), "RADAR SET PWM speed=%u",
                   (unsigned)percent);
    (void)s3_log_info(line);
    if (!s_initialized || percent > RADAR_MAX_SPEED) {
        (void)s3_log_error("PWM RESULT FAIL");
        return false;
    }

    s_calibration_active = true;
    s_calibration_pwm = percent;
    s_speed_percent = percent;
    if (apply_speed() != ESP_OK) {
        (void)s3_log_error("PWM RESULT FAIL");
        return false;
    }
    (void)s3_log_info("PWM RESULT OK");
    return true;
}

void radar_control_release_calibration_lock(void)
{
    s_calibration_active = false;
}

bool radar_control_is_calibration_active(void)
{
    return s_initialized && s_calibration_active;
}

bool radar_control_is_running(void)
{
    return s_initialized && s_state == RADAR_CONTROL_RUNNING;
}

radar_control_state_t radar_control_get_state(void)
{
    return s_state;
}

uint8_t radar_control_get_speed(void)
{
    return s_speed_percent;
}

uint8_t radar_control_get_calibration_pwm(void)
{
    return s_calibration_pwm;
}

void radar_control_handle_pwm_ready_query(void)
{
    if (!s_initialized) {
        return;
    }
    if (s_state == RADAR_CONTROL_WAIT_STM_QUERY) {
        s_state = RADAR_CONTROL_WAIT_IMU_CAL;
        s_speed_percent = RADAR_MIN_SPEED;
        (void)apply_speed();
        ESP_LOGI(TAG, "PWM_READY");
        ESP_LOGI(TAG, "WAIT_IMU_CAL");
    }
}
