#include "chassis_task.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "attitude_startup_coordinator.h"
#include "chassis_kinematics.h"
#include "dual_ahrs.h"
#include "imu_boot_manager.h"
#include "log_service.h"
#include "motor_board_task.h"
#include "s3_service.h"

#define CHASSIS_TASK_STACK_WORDS UINT16_C(512)
#define CHASSIS_TASK_PRIORITY (tskIDLE_PRIORITY + 3U)
#define CHASSIS_TASK_PERIOD_MS UINT32_C(10)
#define CHASSIS_TASK_PERIOD_S 0.010f
#define CHASSIS_HEADING_CONTROL_DT_MIN_S 0.001f
#define CHASSIS_HEADING_CONTROL_DT_MAX_S 0.100f
#define CHASSIS_HEADING_LOG_PERIOD_MS UINT32_C(500)
#define CHASSIS_OUTPUT_LOG_PERIOD_MS UINT32_C(100)
#define CHASSIS_MAX_ACCEL_MM_S2 (400.0f)
#define CHASSIS_HEADING_KP 0.28f /* rad/s per degree */
#define CHASSIS_HEADING_KI 0.085f /* rad/s per degree-second */
#define CHASSIS_HEADING_KD 0.006f /* dimensionless for gyro-z in rad/s */
#define CHASSIS_HEADING_MAX_ALPHA_RAD_S2 5.0f /* rad/s^2 */
#define CHASSIS_HEADING_W_FF_DEFAULT_RAD_S 0.0f /* rad/s */
#define CHASSIS_HEADING_I_MAX_DEG_S 8.0f /* degree-second */
#define CHASSIS_HEADING_I_ERROR_MAX_DEG 15.0f
#define CHASSIS_HEADING_MAX_CORRECTION_RAD_S 2.0f /* rad/s */
#define CHASSIS_MIN_SAFE_WHEEL_SPEED_MM_S 80.0f
#define CHASSIS_RAD_TO_DEG 57.2957795130823208768f

static TaskHandle_t s_task_handle;
static float s_target_linear_mm_s;
static float s_target_angular_rad_s;
static float s_heading_target_linear_mm_s;
static float s_heading_target_yaw_deg;
static bool s_heading_target_active;
static float s_heading_integral;
static float s_heading_w_prev_rad_s;
static float s_heading_w_ff_rad_s = CHASSIS_HEADING_W_FF_DEFAULT_RAD_S;
static float s_ramped_v = 0.0f;
static uint32_t s_stop_sequence;

static float chassis_sanitize_control_dt(float dt_s)
{
    if (!isfinite(dt_s) || dt_s <= 0.0f) {
        return CHASSIS_TASK_PERIOD_S;
    }
    if (dt_s < CHASSIS_HEADING_CONTROL_DT_MIN_S) {
        return CHASSIS_HEADING_CONTROL_DT_MIN_S;
    }
    if (dt_s > CHASSIS_HEADING_CONTROL_DT_MAX_S) {
        return CHASSIS_HEADING_CONTROL_DT_MAX_S;
    }
    return dt_s;
}

static float chassis_ramp_linear_velocity(float target_v_mm_s, float dt_s)
{
    float max_delta;
    float delta;

    if (!isfinite(s_ramped_v)) {
        s_ramped_v = 0.0f;
    }
    if (!isfinite(target_v_mm_s)) {
        s_ramped_v = 0.0f;
        return 0.0f;
    }
    if (!isfinite(dt_s) || dt_s <= 0.0f) {
        return s_ramped_v;
    }
    max_delta = CHASSIS_MAX_ACCEL_MM_S2 * dt_s;
    if (!isfinite(max_delta)) {
        return s_ramped_v;
    }
    delta = target_v_mm_s - s_ramped_v;
    if (delta > max_delta) {
        s_ramped_v += max_delta;
    } else if (delta < -max_delta) {
        s_ramped_v -= max_delta;
    } else {
        s_ramped_v = target_v_mm_s;
    }
    return s_ramped_v;
}

/* Limit angular-velocity slew using the actual task interval. The stored
 * value is the last applied correction in rad/s, while alpha is rad/s^2. */
static float chassis_heading_apply_slew(float raw_rad_s, float dt_s)
{
    float previous_rad_s;
    float max_delta_rad_s;
    float delta_rad_s;
    float applied_rad_s;

    if (!isfinite(raw_rad_s) || !isfinite(dt_s) || dt_s <= 0.0f) {
        taskENTER_CRITICAL();
        s_heading_w_prev_rad_s = 0.0f;
        taskEXIT_CRITICAL();
        return 0.0f;
    }
    max_delta_rad_s = CHASSIS_HEADING_MAX_ALPHA_RAD_S2 * dt_s;
    if (!isfinite(max_delta_rad_s) || max_delta_rad_s < 0.0f) {
        taskENTER_CRITICAL();
        s_heading_w_prev_rad_s = 0.0f;
        taskEXIT_CRITICAL();
        return 0.0f;
    }

    taskENTER_CRITICAL();
    previous_rad_s = s_heading_w_prev_rad_s;
    if (!isfinite(previous_rad_s)) {
        previous_rad_s = 0.0f;
    }
    delta_rad_s = raw_rad_s - previous_rad_s;
    if (!isfinite(delta_rad_s)) {
        applied_rad_s = 0.0f;
    } else if (delta_rad_s > max_delta_rad_s) {
        applied_rad_s = previous_rad_s + max_delta_rad_s;
    } else if (delta_rad_s < -max_delta_rad_s) {
        applied_rad_s = previous_rad_s - max_delta_rad_s;
    } else {
        applied_rad_s = raw_rad_s;
    }
    if (!isfinite(applied_rad_s)) {
        applied_rad_s = 0.0f;
    }
    s_heading_w_prev_rad_s = applied_rad_s;
    taskEXIT_CRITICAL();
    return applied_rad_s;
}

static bool chassis_output_gate_is_open(void)
{
    return s3_service_is_synced() != 0U &&
           imu_boot_manager_is_ready() != 0U &&
           g_attitude_is_ready != 0U &&
           xTaskGetHandle("motor_board") != NULL;
}

static void chassis_apply_min_safe_wheel_speed(
    float linear_mm_s, float wheel_speed[CHASSIS_WHEEL_COUNT])
{
    float left_speed;
    float right_speed;
    float deficit;

    if (wheel_speed == NULL ||
        fabsf(linear_mm_s) <= CHASSIS_MIN_SAFE_WHEEL_SPEED_MM_S) {
        return;
    }

    left_speed = wheel_speed[CHASSIS_WHEEL_LR];
    right_speed = wheel_speed[CHASSIS_WHEEL_RR];
    if (linear_mm_s > 0.0f) {
        if (left_speed < CHASSIS_MIN_SAFE_WHEEL_SPEED_MM_S) {
            deficit = CHASSIS_MIN_SAFE_WHEEL_SPEED_MM_S - left_speed;
            left_speed = CHASSIS_MIN_SAFE_WHEEL_SPEED_MM_S;
            right_speed += deficit;
        } else if (right_speed < CHASSIS_MIN_SAFE_WHEEL_SPEED_MM_S) {
            deficit = CHASSIS_MIN_SAFE_WHEEL_SPEED_MM_S - right_speed;
            right_speed = CHASSIS_MIN_SAFE_WHEEL_SPEED_MM_S;
            left_speed += deficit;
        }
    } else {
        if (left_speed > -CHASSIS_MIN_SAFE_WHEEL_SPEED_MM_S) {
            deficit = CHASSIS_MIN_SAFE_WHEEL_SPEED_MM_S + left_speed;
            left_speed = -CHASSIS_MIN_SAFE_WHEEL_SPEED_MM_S;
            right_speed -= deficit;
        } else if (right_speed > -CHASSIS_MIN_SAFE_WHEEL_SPEED_MM_S) {
            deficit = CHASSIS_MIN_SAFE_WHEEL_SPEED_MM_S + right_speed;
            right_speed = -CHASSIS_MIN_SAFE_WHEEL_SPEED_MM_S;
            left_speed -= deficit;
        }
    }

    /* Apply equal signed changes to both sides, preserving their differential. */
    wheel_speed[CHASSIS_WHEEL_RR] = right_speed;
    wheel_speed[CHASSIS_WHEEL_RF] = right_speed;
    wheel_speed[CHASSIS_WHEEL_LR] = left_speed;
    wheel_speed[CHASSIS_WHEEL_LF] = left_speed;
}

bool chassis_task_set_velocity(float v_mm_s, float w_rad_s)
{
    float wheel_speed[CHASSIS_WHEEL_COUNT];

    if (!chassis_kinematics_compute(v_mm_s, w_rad_s, wheel_speed)) {
        return false;
    }
    if ((fabsf(v_mm_s) > 0.0f || fabsf(w_rad_s) > 0.0f) &&
        !chassis_output_gate_is_open()) {
        LOG_WARN("[CHASSIS_GATE] nonzero command rejected while locked");
        return false;
    }

    taskENTER_CRITICAL();
    s_target_linear_mm_s = v_mm_s;
    s_target_angular_rad_s = w_rad_s;
    s_heading_target_active = false;
    s_heading_target_linear_mm_s = 0.0f;
    s_heading_integral = 0.0f;
    s_heading_w_prev_rad_s = 0.0f;
    taskEXIT_CRITICAL();
    return true;
}

void chassis_task_force_stop(void)
{
    taskENTER_CRITICAL();
    s_target_linear_mm_s = 0.0f;
    s_target_angular_rad_s = 0.0f;
    s_heading_target_linear_mm_s = 0.0f;
    s_heading_target_yaw_deg = 0.0f;
    s_heading_target_active = false;
    s_heading_integral = 0.0f;
    s_heading_w_prev_rad_s = 0.0f;
    s_heading_w_ff_rad_s = CHASSIS_HEADING_W_FF_DEFAULT_RAD_S;
    s_ramped_v = 0.0f;
    ++s_stop_sequence;
    taskEXIT_CRITICAL();
}

void chassis_task_set_heading_target(float v_mm_s, float target_yaw_deg)
{
    if (!chassis_task_heading_target_is_admissible(v_mm_s, target_yaw_deg)) {
        return;
    }
    target_yaw_deg = fmodf(target_yaw_deg, 360.0f);
    if (target_yaw_deg >= 180.0f) {
        target_yaw_deg -= 360.0f;
    }
    while (target_yaw_deg < -180.0f) {
        target_yaw_deg += 360.0f;
    }
    if (fabsf(v_mm_s) <= 0.001f) {
        /* A zero Target Yaw frame is the local stop/clear operation. It must
         * never leave an active controller that can inject angular velocity
         * while the requested linear speed is zero. */
        chassis_task_force_stop();
        return;
    }
    taskENTER_CRITICAL();
    s_target_linear_mm_s = 0.0f;
    s_target_angular_rad_s = 0.0f;
    s_heading_target_linear_mm_s = v_mm_s;
    s_heading_target_yaw_deg = target_yaw_deg;
    s_heading_target_active = true;
    /* Target Yaw frames are refreshed at transport rate. Preserve the slew
     * state so repeated frames do not weaken an active heading correction. */
    taskEXIT_CRITICAL();
}

void chassis_task_set_heading_feedforward(float w_ff_rad_s)
{
    if (!isfinite(w_ff_rad_s) ||
        fabsf(w_ff_rad_s) > CHASSIS_HEADING_MAX_CORRECTION_RAD_S) {
        return;
    }
    taskENTER_CRITICAL();
    s_heading_w_ff_rad_s = w_ff_rad_s;
    taskEXIT_CRITICAL();
}

bool chassis_task_heading_target_is_admissible(float v_mm_s,
                                               float target_yaw_deg)
{
    return isfinite(v_mm_s) && isfinite(target_yaw_deg) &&
           fabsf(v_mm_s) <= CHASSIS_WHEEL_SPEED_LIMIT_MM_S &&
           target_yaw_deg >= -180.0f && target_yaw_deg <= 180.0f &&
           (v_mm_s == 0.0f || chassis_output_gate_is_open());
}

void chassis_task(void *argument)
{
    TickType_t last_wake;
    TickType_t last_control_tick;
    float previous_wheel_speed[CHASSIS_WHEEL_COUNT] = {0.0f};
    bool was_gate_open = false;

    (void)argument;
    taskENTER_CRITICAL();
    s_target_linear_mm_s = 0.0f;
    s_target_angular_rad_s = 0.0f;
    s_heading_target_linear_mm_s = 0.0f;
    s_heading_target_yaw_deg = 0.0f;
    s_heading_target_active = false;
    s_heading_integral = 0.0f;
    s_heading_w_prev_rad_s = 0.0f;
    s_heading_w_ff_rad_s = CHASSIS_HEADING_W_FF_DEFAULT_RAD_S;
    s_ramped_v = 0.0f;
    taskEXIT_CRITICAL();
    last_wake = xTaskGetTickCount();
    last_control_tick = last_wake;

    for (;;) {
        float linear_mm_s;
        float angular_rad_s;
        float heading_target_yaw_deg;
        float wheel_speed[CHASSIS_WHEEL_COUNT];
        float current_yaw_rad = 0.0f;
        float gyro_z_rad_s = 0.0f;
        float heading_error_deg = 0.0f;
        float heading_correction_raw_rad_s = 0.0f;
        float heading_correction_rad_s = 0.0f;
        float heading_w_ff_rad_s = CHASSIS_HEADING_W_FF_DEFAULT_RAD_S;
        const TickType_t control_tick = xTaskGetTickCount();
        float control_dt_s =
            (float)(control_tick - last_control_tick) /
            (float)configTICK_RATE_HZ;
        bool heading_active;
        uint32_t stop_sequence;
        bool output_changed;
        static TickType_t next_heading_log;
        static TickType_t next_output_log;
        const bool gate_open = chassis_output_gate_is_open();

        taskENTER_CRITICAL();
        linear_mm_s = s_target_linear_mm_s;
        angular_rad_s = s_target_angular_rad_s;
        heading_target_yaw_deg = s_heading_target_yaw_deg;
        heading_active = s_heading_target_active;
        if (heading_active) {
            linear_mm_s = s_heading_target_linear_mm_s;
            angular_rad_s = 0.0f;
        }
        stop_sequence = s_stop_sequence;
        taskEXIT_CRITICAL();
        last_control_tick = control_tick;
        control_dt_s = chassis_sanitize_control_dt(control_dt_s);

        if (!gate_open) {
            (void)memset(wheel_speed, 0, sizeof(wheel_speed));
            if (was_gate_open || linear_mm_s != 0.0f || angular_rad_s != 0.0f) {
                LOG_WARN("[CHASSIS_GATE] output forced to zero");
            }
            /* A revoked admission is a stop-and-clear event. Do not resume an
             * old operator target automatically when the link or IMU recovers. */
            taskENTER_CRITICAL();
            s_target_linear_mm_s = 0.0f;
            s_target_angular_rad_s = 0.0f;
            s_heading_target_linear_mm_s = 0.0f;
            s_heading_target_yaw_deg = 0.0f;
            s_heading_target_active = false;
            s_heading_w_prev_rad_s = 0.0f;
            s_ramped_v = 0.0f;
            taskEXIT_CRITICAL();
            s_heading_integral = 0.0f;
            was_gate_open = false;
        } else {
            linear_mm_s = chassis_ramp_linear_velocity(linear_mm_s,
                                                        control_dt_s);
            if (heading_active) {
                if (dual_ahrs_get_heading_state(&current_yaw_rad,
                                                &gyro_z_rad_s) == 0U) {
                    (void)memset(wheel_speed, 0, sizeof(wheel_speed));
                    taskENTER_CRITICAL();
                    s_target_linear_mm_s = 0.0f;
                    s_target_angular_rad_s = 0.0f;
                    s_heading_target_linear_mm_s = 0.0f;
                    s_heading_target_yaw_deg = 0.0f;
                    s_heading_target_active = false;
                    s_heading_w_prev_rad_s = 0.0f;
                    s_ramped_v = 0.0f;
                    taskEXIT_CRITICAL();
                    s_heading_integral = 0.0f;
                    LOG_WARN("[HEADING_CTRL] attitude invalid; target cleared");
                    was_gate_open = false;
                } else {
                    const float current_yaw_deg =
                        current_yaw_rad * CHASSIS_RAD_TO_DEG;

                    heading_error_deg = current_yaw_deg - heading_target_yaw_deg;
                    while (heading_error_deg > 180.0f) {
                        heading_error_deg -= 360.0f;
                    }
                    while (heading_error_deg < -180.0f) {
                        heading_error_deg += 360.0f;
                    }
                    if (!isfinite(s_heading_integral)) {
                        s_heading_integral = 0.0f;
                    }
                    if (fabsf(linear_mm_s) > 0.001f &&
                        fabsf(heading_error_deg) < CHASSIS_HEADING_I_ERROR_MAX_DEG) {
                        s_heading_integral +=
                            heading_error_deg * control_dt_s;
                        if (s_heading_integral > CHASSIS_HEADING_I_MAX_DEG_S) {
                            s_heading_integral = CHASSIS_HEADING_I_MAX_DEG_S;
                        } else if (s_heading_integral < -CHASSIS_HEADING_I_MAX_DEG_S) {
                            s_heading_integral = -CHASSIS_HEADING_I_MAX_DEG_S;
                        }
                    } else {
                        s_heading_integral = 0.0f;
                    }
                    /* Positive local heading error requests the established
                     * positive correction: RR/RF faster and LR/LF slower. */
                    taskENTER_CRITICAL();
                    heading_w_ff_rad_s = s_heading_w_ff_rad_s;
                    taskEXIT_CRITICAL();
                    /* DualAHRS supplies body-frame gyro-Z. Under the local
                     * Body-X-forward/Body-Y-right convention, positive yaw
                     * rate is opposed by a negative correction command. */
                    heading_correction_raw_rad_s =
                        heading_w_ff_rad_s +
                        CHASSIS_HEADING_KP * heading_error_deg +
                        CHASSIS_HEADING_KI * s_heading_integral +
                        (-CHASSIS_HEADING_KD * gyro_z_rad_s);
                    if (!isfinite(heading_correction_raw_rad_s)) {
                        s_heading_integral = 0.0f;
                        s_heading_w_prev_rad_s = 0.0f;
                        heading_correction_raw_rad_s = 0.0f;
                    }
                    {
                        const float max_wheel_delta_mm_s =
                            CHASSIS_WHEEL_SPEED_LIMIT_MM_S - fabsf(linear_mm_s);
                        float max_correction_rad_s = max_wheel_delta_mm_s /
                                                     (0.5f * CHASSIS_TRACK_WIDTH_MM);

                        if (max_correction_rad_s >
                            CHASSIS_HEADING_MAX_CORRECTION_RAD_S) {
                            max_correction_rad_s =
                                CHASSIS_HEADING_MAX_CORRECTION_RAD_S;
                        }
                        if (max_correction_rad_s < 0.0f) {
                            max_correction_rad_s = 0.0f;
                        }
                        if (heading_correction_raw_rad_s >
                            max_correction_rad_s) {
                            heading_correction_raw_rad_s =
                                max_correction_rad_s;
                        } else if (heading_correction_raw_rad_s <
                                   -max_correction_rad_s) {
                            heading_correction_raw_rad_s =
                                -max_correction_rad_s;
                        }
                    }
                    heading_correction_rad_s =
                        chassis_heading_apply_slew(heading_correction_raw_rad_s,
                                                   control_dt_s);
                    angular_rad_s = heading_correction_rad_s;
                    if (!chassis_kinematics_compute(linear_mm_s, angular_rad_s,
                                                    wheel_speed)) {
                        s_heading_integral = 0.0f;
                        s_heading_w_prev_rad_s = 0.0f;
                        (void)memset(wheel_speed, 0, sizeof(wheel_speed));
                        LOG_WARN("[HEADING_CTRL] kinematics rejected target; output zero");
                    } else {
                        chassis_apply_min_safe_wheel_speed(linear_mm_s, wheel_speed);
                        was_gate_open = true;
                    }
                    if ((TickType_t)(xTaskGetTickCount() - next_heading_log) <
                        (TickType_t)0x80000000U) {
                        char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

                        (void)snprintf(line, sizeof(line),
                                       "[HEADING_CTRL] target_yaw=%.2f, cur_yaw=%.2f, err=%.2f, w_corr=%.4f",
                                       (double)heading_target_yaw_deg,
                                       (double)current_yaw_deg,
                                       (double)heading_error_deg,
                                       (double)heading_correction_rad_s);
                        LOG_INFO(line);
                        next_heading_log = xTaskGetTickCount() +
                                           pdMS_TO_TICKS(CHASSIS_HEADING_LOG_PERIOD_MS);
                    }
                }
            } else if (!chassis_kinematics_compute(linear_mm_s, angular_rad_s,
                                                   wheel_speed)) {
                s_heading_integral = 0.0f;
                s_heading_w_prev_rad_s = 0.0f;
                (void)memset(wheel_speed, 0, sizeof(wheel_speed));
                LOG_WARN("[CHASSIS_KIN] invalid target forced to zero");
            } else {
                s_heading_integral = 0.0f;
                s_heading_w_prev_rad_s = 0.0f;
                was_gate_open = true;
            }
        }

        output_changed = memcmp(previous_wheel_speed, wheel_speed,
                                sizeof(wheel_speed)) != 0;
        if (output_changed) {
            const TickType_t now = xTaskGetTickCount();

            if ((TickType_t)(now - next_output_log) <
                (TickType_t)0x80000000U) {
                char line[SMARTCAR_LOG_MAX_PAYLOAD + 1U];

                (void)snprintf(line, sizeof(line),
                               "[CHASSIS_OUT] target=[RR %.1f RF %.1f LR %.1f LF %.1f]",
                               (double)wheel_speed[CHASSIS_WHEEL_RR],
                               (double)wheel_speed[CHASSIS_WHEEL_RF],
                               (double)wheel_speed[CHASSIS_WHEEL_LR],
                               (double)wheel_speed[CHASSIS_WHEEL_LF]);
                LOG_INFO(line);
                next_output_log = now +
                                  pdMS_TO_TICKS(CHASSIS_OUTPUT_LOG_PERIOD_MS);
            }
            (void)memcpy(previous_wheel_speed, wheel_speed,
                         sizeof(previous_wheel_speed));
        }
        if (output_changed) {
            bool target_accepted = true;

            taskENTER_CRITICAL();
            if (stop_sequence == s_stop_sequence &&
                chassis_output_gate_is_open()) {
                target_accepted = motor_board_set_target_wheel_speeds(wheel_speed);
            } else {
                /* A hard stop raced this cycle. Keep its zeroed ramp state and
                 * do not leave a stale nonzero tuple for the next iteration. */
                s_ramped_v = 0.0f;
                (void)memset(previous_wheel_speed, 0,
                             sizeof(previous_wheel_speed));
            }
            taskEXIT_CRITICAL();
            if (!target_accepted) {
                LOG_WARN("[CHASSIS_OUT] MotorBoard target rejected");
            }
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CHASSIS_TASK_PERIOD_MS));
    }
}

void chassis_task_start(void)
{
    if (s_task_handle != NULL) {
        return;
    }
    if (xTaskCreate(chassis_task, "chassis_task", CHASSIS_TASK_STACK_WORDS,
                    NULL, CHASSIS_TASK_PRIORITY, &s_task_handle) != pdPASS) {
        s_task_handle = NULL;
        LOG_ERROR("[CHASSIS_TASK] task create failed");
    } else {
        LOG_INFO("[CHASSIS_TASK] task started");
    }
}
