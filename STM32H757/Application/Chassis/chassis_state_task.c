#include "chassis_state_task.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal.h"

#include "attitude_startup_coordinator.h"
#include "chassis_odometry.h"
#include "chassis_state_payload.h"
#include "dual_ahrs.h"
#include "log_service.h"
#include "motor_board_task.h"
#include "s3_service.h"
#include "srp_registry.h"

/* 底盘状态与里程计发布实现；创建人：待确认（当前维护人：Zhiqin）。 */

#define CHASSIS_STATE_TASK_STACK_WORDS UINT16_C(256)
#define CHASSIS_STATE_TASK_PRIORITY (tskIDLE_PRIORITY + 1U)
#define CHASSIS_STATE_TASK_PERIOD_MS UINT32_C(50)
static TaskHandle_t s_task_handle;

void chassis_state_task(void *argument)
{
    TickType_t last_wake;
    chassis_odometry_state_t odometry;
    uint32_t last_processed_sequence = 0U;
    bool have_processed_sequence = false;

    (void)argument;
    chassis_odometry_init(&odometry);
    last_wake = xTaskGetTickCount();

    for (;;) {
        motor_board_wheel_speed_snapshot_t wheel = {0};
        uint8_t payload[SRP_PAYLOAD_CHASSIS_STATE_SIZE];
        uint8_t flags = 0U;
        float primary_yaw_rad = odometry.yaw_rad;
        float gyro_z_rad_s = 0.0f;
        const uint32_t now_ms = HAL_GetTick();
        const bool have_wheel =
            motor_board_get_actual_wheel_speed_snapshot(&wheel);
        const bool wheel_fresh =
            have_wheel &&
            (uint32_t)(now_ms - wheel.timestamp_ms) <=
                CHASSIS_ODOMETRY_MAX_SAMPLE_INTERVAL_MS;
        const bool attitude_fresh =
            g_attitude_is_ready != 0U &&
            dual_ahrs_get_heading_state(&primary_yaw_rad,
                                        &gyro_z_rad_s) != 0U;
        bool publish = false;

        (void)gyro_z_rad_s;
        if (attitude_fresh) {
            flags |= SRP_CHASSIS_STATE_FLAG_ATTITUDE_READY;
        }

        if (wheel_fresh && attitude_fresh &&
            chassis_state_sequence_is_new(wheel.sequence,
                                           last_processed_sequence,
                                           have_processed_sequence)) {
            const chassis_odometry_result_t result = chassis_odometry_update(
                &odometry, wheel.speed_mm_s, primary_yaw_rad,
                wheel.timestamp_ms);

            last_processed_sequence = wheel.sequence;
            have_processed_sequence = true;
            if (result != CHASSIS_ODOMETRY_RESULT_INVALID && odometry.valid) {
                flags |= SRP_CHASSIS_STATE_FLAG_ODOMETRY_VALID;
            }
            publish = true;
        } else if (!wheel_fresh || !attitude_fresh) {
            chassis_odometry_invalidate(&odometry);
            have_processed_sequence = false;
            publish = true;
        }

        if (publish) {
            if (chassis_state_pack_payload(payload, sizeof(payload), &odometry,
                                           flags,
                                           have_wheel ? wheel.timestamp_ms : 0U)) {
                s3_service_send_chassis_state(payload, sizeof(payload));
            }
        }
        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(CHASSIS_STATE_TASK_PERIOD_MS));
    }
}

void chassis_state_task_start(void)
{
    if (s_task_handle != NULL) {
        return;
    }
    if (xTaskCreate(chassis_state_task, "chassis_state",
                    CHASSIS_STATE_TASK_STACK_WORDS, NULL,
                    CHASSIS_STATE_TASK_PRIORITY, &s_task_handle) != pdPASS) {
        s_task_handle = NULL;
        LOG_ERROR("[CHASSIS_STATE] task create failed");
    } else {
        LOG_INFO("[CHASSIS_STATE] task started");
    }
}
