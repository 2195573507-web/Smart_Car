#include "pid_controller.h"
#include "wheel_control_params.h"

#include <math.h>
#include <stddef.h>

static float clamp_float(float value, float limit)
{
    if (limit < 0.0f) {
        limit = -limit;
    }
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

float Ramp_Update(Ramp_Profile_t *ramp, float final_target, float dt_seconds)
{
    float step;

    if (ramp == NULL) {
        return 0.0f;
    }
    if (!isfinite(ramp->current_target) || !isfinite(ramp->max_accel) ||
        !isfinite(final_target) || !isfinite(dt_seconds) || dt_seconds <= 0.0f) {
        return ramp->current_target;
    }
    step = fabsf(ramp->max_accel) * dt_seconds;
    if (final_target > ramp->current_target) {
        ramp->current_target += step;
        if (ramp->current_target > final_target) {
            ramp->current_target = final_target;
        }
    } else if (final_target < ramp->current_target) {
        ramp->current_target -= step;
        if (ramp->current_target < final_target) {
            ramp->current_target = final_target;
        }
    }
    return ramp->current_target;
}

void Ramp_Update_Max_Accel(Ramp_Profile_t *ramp, float max_accel)
{
    if (ramp == NULL || !isfinite(max_accel) || max_accel < 0.0f) {
        return;
    }
    ramp->max_accel = max_accel;
}

void pid_controller_init(pid_controller_t *pid, float kp, float ki, float kd,
                         float max_out, float max_iout, float deadband)
{
    if (pid == NULL) {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_out = fabsf(max_out);
    pid->max_iout = fabsf(max_iout);
    pid->deadband = fabsf(deadband);
    pid->feedback_alpha = WHEEL_FEEDBACK_LPF_ALPHA;
    pid_controller_reset(pid);
}

void PID_Update_Gains(PID_Controller_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL || !isfinite(kp) || !isfinite(ki) || !isfinite(kd)) {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_controller_reset(pid_controller_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->filtered_feedback = 0.0f;
    pid->previous_filtered_feedback = 0.0f;
    pid->output = 0.0f;
    pid->initialized = 0U;
}

float pid_controller_step_with_feedforward(pid_controller_t *pid,
                                            float target, float actual,
                                            float dt_seconds)
{
    float smooth_target = target;
    float error;
    float output;

    if (pid == NULL || !isfinite(smooth_target) || !isfinite(actual) ||
        !isfinite(dt_seconds) || dt_seconds <= 0.0f) {
        return 0.0f;
    }

    /* A stopped target must clear the stored integral and produce no PWM. */
    if (fabsf(smooth_target) < 1.0f) {
        pid->integral = 0.0f;
        pid->previous_error = 0.0f;
        pid->output = 0.0f;
        return 0.0f;
    }

    /* Establish a feedback baseline before filtering so reset does not inject
     * a transient from the zero-initialized state. */
    if (pid->initialized == 0U) {
        pid->filtered_feedback = actual;
        pid->previous_filtered_feedback = actual;
        pid->initialized = 1U;
    } else {
        pid->previous_filtered_feedback = pid->filtered_feedback;
        pid->filtered_feedback =
            pid->feedback_alpha * actual +
            (1.0f - pid->feedback_alpha) * pid->filtered_feedback;
    }

    error = smooth_target - pid->filtered_feedback;
    if (fabsf(error) < pid->deadband) {
        error = 0.0f;
    }

    /* Conditional integration: hold the integral while the previous output
     * is saturated and the current error would drive it further into the
     * same saturation.  An error in the opposite direction can still unwind
     * the stored integral immediately. */
    if (!(fabsf(pid->output) >= pid->max_out &&
          (error * pid->output) > 0.0f)) {
        pid->integral = clamp_float(
            pid->integral + pid->ki * error * dt_seconds, pid->max_iout);
    }

    /* Blend linear feedforward with a signed, speed-proportional Coulomb
     * compensation.  The ratio saturates at the configured ramp velocity so
     * the compensation reaches its peak without introducing a step. */
    const float friction_ratio = clamp_float(
        smooth_target / WHEEL_FRICTION_RAMP_VELOCITY, 1.0f);
    const float ff = (smooth_target * WHEEL_FEEDFORWARD_KV) +
                     (friction_ratio * WHEEL_FRICTION_COMP_PWM);
    const float total_out = ff + (pid->kp * error) + pid->integral;

    output = clamp_float(total_out, pid->max_out);
    pid->previous_error = error;
    pid->output = output;
    return output;
}

float pid_controller_step(pid_controller_t *pid, float target, float actual,
                          float dt_seconds)
{
    return pid_controller_step_with_feedforward(pid, target, actual,
                                                 dt_seconds);
}
