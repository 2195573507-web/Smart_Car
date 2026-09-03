#include "pid_controller.h"

/* 轮速斜坡与 PID 实现；创建人：待确认（当前维护人：Zhiqin）。 */
#include "wheel_control_params.h"

#include <math.h>
#include <stddef.h>

/**
 * @brief 将标量限制在对称区间 [-|limit|, |limit|] 内。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] value 待限制的标量。
 * @param[in] limit 对称上限；有限负值按绝对值处理。
 * @return value 超过正/负限时返回对应边界，否则返回 value；value 或 limit 为 NaN 时比较
 *         不成立并原样返回 value，函数无独立失败码。
 * 调用方式：仅 PID 步进路径用于积分、摩擦补偿比例和最终输出限幅。
 * 线程约束：纯计算、不阻塞、不获取 mutex 且可重入，禁止 ISR 调用；参数按值传递，
 *           不访问共享状态或涉及对象所有权。
 */
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

/** 按最大加速度和动态 dt 推进目标斜坡。 */
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

/** 更新斜坡最大加速度；非法值保持原配置。 */
void Ramp_Update_Max_Accel(Ramp_Profile_t *ramp, float max_accel)
{
    if (ramp == NULL || !isfinite(max_accel) || max_accel < 0.0f) {
        return;
    }
    ramp->max_accel = max_accel;
}

/** 初始化 PID 参数、限幅和反馈滤波状态。 */
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

/** 在线更新 PID 增益并保留积分/反馈历史。 */
void PID_Update_Gains(PID_Controller_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL || !isfinite(kp) || !isfinite(ki) || !isfinite(kd)) {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/** 清除积分、微分和输出历史。 */
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

/** 执行一次内置速度前馈、P/I、死区和抗积分饱和步进。 */
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

/** 兼容入口；当前与内置前馈步进的行为完全一致。 */
float pid_controller_step(pid_controller_t *pid, float target, float actual,
                          float dt_seconds)
{
    return pid_controller_step_with_feedforward(pid, target, actual,
                                                 dt_seconds);
}
