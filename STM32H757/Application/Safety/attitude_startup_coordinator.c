#include "attitude_startup_coordinator.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "stm32h7xx_hal.h"

#include "attitude.h"
#include "imu_boot_manager.h"
#include "imu_filter.h"
#include "imu_manager.h"
#include "log_service.h"
#include "motor_board_task.h"

#ifndef SMARTCAR_MOTOR_BOARD_ONLY
#define SMARTCAR_MOTOR_BOARD_ONLY 0
#endif

#define ATTITUDE_STARTUP_STACK_WORDS UINT16_C(1024)
#define ATTITUDE_STARTUP_PRIORITY (tskIDLE_PRIORITY + 3U)
#define ATTITUDE_STARTUP_PERIOD_MS UINT32_C(20)
#define ATTITUDE_STARTUP_LOG_PERIOD_MS UINT32_C(1000)
#define ATTITUDE_STARTUP_STABLE_CYCLES UINT8_C(5)

volatile uint8_t g_attitude_is_ready;

static TaskHandle_t s_task_handle;

static void startup_log_locked_state(uint32_t now_ms)
{
    static uint32_t last_log_ms;
    imu_boot_status_t status = {0};
    char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

    if ((uint32_t)(now_ms - last_log_ms) < ATTITUDE_STARTUP_LOG_PERIOD_MS) {
        return;
    }
    last_log_ms = now_ms;
    imu_boot_manager_get_status(&status);
    (void)snprintf(line, sizeof(line),
                   "[ATTITUDE_STARTUP] LOCKED state=%u phase=%u progress=%u "
                   "error=%u reason=%s",
                   (unsigned)status.state, (unsigned)status.phase,
                   (unsigned)status.progress, (unsigned)status.error,
                   status.error_reason == NULL ? "NONE" : status.error_reason);
    if (status.error != (uint8_t)IMU_ERROR_NONE) {
        LOG_ERROR(line);
    } else {
        LOG_INFO(line);
    }
}

static bool attitude_lifecycle_is_ready(void)
{
    if (imu_boot_manager_is_error() != 0U ||
        imu_boot_manager_is_ready() == 0U || imu_is_ready() == 0U ||
        imu_filter_is_ready() == 0U || attitude_zero_is_ready() == 0U ||
        attitude_get_status() != AHRS_READY) {
        return false;
    }
    return true;
}

static bool attitude_updates_are_advancing(uint32_t *last_lsm_updates,
                                           uint32_t *last_bmi_updates,
                                           bool *have_baseline)
{
    imu_sensor_stats_t lsm_stats = {0};
    imu_sensor_stats_t bmi_stats = {0};

    if (last_lsm_updates == NULL || last_bmi_updates == NULL ||
        have_baseline == NULL ||
        imu_get_lsm303_stats(&lsm_stats) != BSP_STATUS_OK ||
        imu_get_bmi323_stats(&bmi_stats) != BSP_STATUS_OK) {
        return false;
    }
    if (*have_baseline &&
        (lsm_stats.update_count <= *last_lsm_updates ||
         bmi_stats.update_count <= *last_bmi_updates)) {
        *last_lsm_updates = lsm_stats.update_count;
        *last_bmi_updates = bmi_stats.update_count;
        return false;
    }
    *last_lsm_updates = lsm_stats.update_count;
    *last_bmi_updates = bmi_stats.update_count;
    *have_baseline = true;
    return true;
}

void attitude_startup_coordinator_task(void *argument)
{
    TickType_t last_wake;
    uint32_t last_lsm_updates = 0U;
    uint32_t last_bmi_updates = 0U;
    uint8_t stable_cycles = 0U;
    bool have_baseline = false;

    (void)argument;
    g_attitude_is_ready = 0U;
    last_wake = xTaskGetTickCount();
    for (;;) {
        const uint32_t now_ms = HAL_GetTick();

        if (g_attitude_is_ready != 0U) {
            vTaskDelayUntil(&last_wake,
                            pdMS_TO_TICKS(ATTITUDE_STARTUP_PERIOD_MS));
            continue;
        }

#if SMARTCAR_MOTOR_BOARD_ONLY
        /* This task is not started for the motor-board-only image. Keep the
         * guard here so an accidental call can never unlock that image. */
        (void)motor_board_force_stop();
        startup_log_locked_state(now_ms);
#else
        if (!attitude_lifecycle_is_ready() ||
            !attitude_updates_are_advancing(&last_lsm_updates,
                                            &last_bmi_updates,
                                            &have_baseline)) {
            stable_cycles = 0U;
            have_baseline = false;
            (void)motor_board_force_stop();
            startup_log_locked_state(now_ms);
        } else if (stable_cycles < ATTITUDE_STARTUP_STABLE_CYCLES) {
            ++stable_cycles;
        }

        if (stable_cycles >= ATTITUDE_STARTUP_STABLE_CYCLES) {
            /* The task itself clears its internal stop latch during its
             * initialization. It is deliberately created only here, after
             * the attitude gate, so no command can produce PWM earlier. */
            g_attitude_is_ready = 1U;
            motor_board_task_start();
            if (xTaskGetHandle("motor_board") == NULL) {
                g_attitude_is_ready = 0U;
                stable_cycles = 0U;
                have_baseline = false;
                (void)motor_board_force_stop();
                LOG_ERROR("[ATTITUDE_STARTUP] MOTOR_UNLOCK failed task_missing");
            } else {
                LOG_INFO("[ATTITUDE_STARTUP] READY; MOTOR_UNLOCK task_started");
            }
        }
#endif
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(ATTITUDE_STARTUP_PERIOD_MS));
    }
}

void attitude_startup_coordinator_start(void)
{
#if SMARTCAR_MOTOR_BOARD_ONLY
    g_attitude_is_ready = 0U;
    return;
#else
    if (s_task_handle != NULL) {
        return;
    }
    g_attitude_is_ready = 0U;
    if (xTaskCreate(attitude_startup_coordinator_task,
                    "attitude_gate", ATTITUDE_STARTUP_STACK_WORDS, NULL,
                    ATTITUDE_STARTUP_PRIORITY, &s_task_handle) != pdPASS) {
        s_task_handle = NULL;
        (void)motor_board_force_stop();
        LOG_ERROR("[ATTITUDE_STARTUP] coordinator task create failed");
    }
#endif
}
