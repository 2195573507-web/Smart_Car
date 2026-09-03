#include "attitude.h"

/* 兼容姿态状态实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <math.h>

#include "imu_calibration.h"
#include "imu_filter.h"

static attitude_state_t attitude_state;
static ahrs_state_t ahrs_state;
static uint8_t attitude_zero_active;
static uint8_t attitude_zero_ready;
static uint16_t attitude_zero_sample_count;
static float attitude_zero_roll_sum;
static float attitude_zero_pitch_sum;
static float attitude_zero_yaw_sin_sum;
static float attitude_zero_yaw_cos_sum;
static float attitude_roll_offset;
static float attitude_pitch_offset;
static float attitude_yaw_offset;

#define ATTITUDE_PI 3.14159265358979323846f
#define ATTITUDE_TWO_PI (2.0f * ATTITUDE_PI)

/**
 * @brief 将有限航向角折返到兼容姿态实现使用的 (-pi, pi] 区间。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * @param[in] angle 待折返的航向角，单位 rad；当前调用点传入由 `atan2f()` 及
 *                  已折返历史值形成的有界角度。
 * @return 有限输入返回 (-pi, pi] 内等价角；NaN 原样返回；正负无穷会使当前
 *         `while` 实现无法结束，调用方必须保证输入有限。
 * 调用方式：仅由 `attitude_update()` 在零位修正、航向增量和状态写回时同步调用。
 * 线程约束：纯计算、不读写共享状态、不获取 mutex 且不访问外设；有限输入下不阻塞，
 *           但循环次数随输入幅值增长，正负无穷会永久阻塞。禁止从 ISR 调用，输入所有权始终归调用方。
 */
static float yaw_wrap(float angle)
{
    while (angle > ATTITUDE_PI) {
        angle -= ATTITUDE_TWO_PI;
    }
    while (angle <= -ATTITUDE_PI) {
        angle += ATTITUDE_TWO_PI;
    }
    return angle;
}

/**
 * @brief 原子语义地完成兼容姿态零位窗口并把 AHRS 状态标记为 READY。
 * @author 创建人：待确认（当前维护人：Zhiqin）。
 * @date 2026-08-31（静态函数契约补充）。
 * 传入参数：无。
 * @return 无返回值；函数无失败分支，写入 `attitude_zero_active=0`、
 *         `attitude_zero_ready=1` 和 `ahrs_state=AHRS_READY`。
 * 调用方式：仅由 `attitude_update()` 在零位累计完成或已有零位后刷新状态时调用。
 * 线程约束：读写本文件静态状态，无内部 mutex、非线程安全且不阻塞；必须由 IMU
 *           单 writer 在任务上下文调用，禁止 ISR/并发调用，不转移任何对象所有权。
 */
static void attitude_mark_ready(void)
{
    attitude_zero_active = 0U;
    attitude_zero_ready = 1U;
    ahrs_state = AHRS_READY;
}

/** 初始化兼容姿态状态和零位累计器。 */
void attitude_init(void)
{
    attitude_state = (attitude_state_t){0};
    ahrs_state = AHRS_WAIT_CAL;
    attitude_zero_active = 0U;
    attitude_zero_ready = 0U;
    attitude_zero_sample_count = 0U;
    attitude_zero_roll_sum = 0.0f;
    attitude_zero_pitch_sum = 0.0f;
    attitude_zero_yaw_sin_sum = 0.0f;
    attitude_zero_yaw_cos_sum = 0.0f;
    attitude_roll_offset = 0.0f;
    attitude_pitch_offset = 0.0f;
    attitude_yaw_offset = 0.0f;
}

/** 清除零位窗口和 READY 状态。 */
void attitude_zero_reset(void)
{
    attitude_zero_active = 0U;
    attitude_zero_ready = 0U;
    attitude_zero_sample_count = 0U;
    attitude_zero_roll_sum = 0.0f;
    attitude_zero_pitch_sum = 0.0f;
    attitude_zero_yaw_sin_sum = 0.0f;
    attitude_zero_yaw_cos_sum = 0.0f;
    attitude_roll_offset = 0.0f;
    attitude_pitch_offset = 0.0f;
    attitude_yaw_offset = 0.0f;
    attitude_state = (attitude_state_t){0};
    ahrs_state = AHRS_WAIT_CAL;
}

/** 开启零位窗口；实际采样仍由生产生命周期门控。 */
void attitude_zero_init(void)
{
    if (attitude_zero_active != 0U || attitude_zero_ready != 0U) {
        return;
    }
    attitude_zero_active = 1U;
    attitude_zero_sample_count = 0U;
    attitude_zero_roll_sum = 0.0f;
    attitude_zero_pitch_sum = 0.0f;
    attitude_zero_yaw_sin_sum = 0.0f;
    attitude_zero_yaw_cos_sum = 0.0f;
    ahrs_state = AHRS_WAIT_CAL;
}

/** 查询零位是否已经被生产路径确认。 */
uint8_t attitude_zero_is_ready(void)
{
    return attitude_zero_ready;
}

/** 兼容入口：不允许绕过生产零位采样窗口。 */
uint8_t attitude_zero_capture_current(void)
{
    /* Keep the legacy symbol for source compatibility, but never let a
     * caller bypass the production zero-reference sample window. */
    return attitude_zero_ready;
}

/** 消费滤波结果并更新兼容姿态状态。 */
void attitude_update(void)
{
    const imu_filtered_data_t filtered = imu_filter_get_output();
    const float roll = atan2f(filtered.ay, filtered.az);
    const float pitch = atan2f(-filtered.ax,
                               sqrtf((filtered.ay * filtered.ay) +
                                     (filtered.az * filtered.az)));
    /* imu_manager supplies LSM303 data in the vehicle Body Frame. */
    const float yaw = atan2f(filtered.my, filtered.mx);

    if (imu_calibration_is_complete() == 0U || filtered.online == 0U ||
        !isfinite(filtered.ax) || !isfinite(filtered.ay) ||
        !isfinite(filtered.az) || !isfinite(filtered.mx) ||
        !isfinite(filtered.my) || !isfinite(filtered.mz)) {
        return;
    }

    if (attitude_zero_ready == 0U && attitude_zero_active == 0U) {
        attitude_zero_init();
    }

    if (attitude_zero_active != 0U) {
        attitude_zero_roll_sum += roll;
        attitude_zero_pitch_sum += pitch;
        attitude_zero_yaw_sin_sum += sinf(yaw);
        attitude_zero_yaw_cos_sum += cosf(yaw);
        ++attitude_zero_sample_count;
        if (attitude_zero_sample_count >= ATTITUDE_ZERO_SAMPLE_COUNT) {
            const float sample_count = (float)attitude_zero_sample_count;
            attitude_roll_offset = attitude_zero_roll_sum / sample_count;
            attitude_pitch_offset = attitude_zero_pitch_sum / sample_count;
            attitude_yaw_offset = atan2f(attitude_zero_yaw_sin_sum,
                                         attitude_zero_yaw_cos_sum);
            attitude_mark_ready();
            attitude_state.roll = 0.0f;
            attitude_state.pitch = 0.0f;
            attitude_state.yaw = 0.0f;
        }
        return;
    }
    if (attitude_zero_ready == 0U) {
        return;
    }

    attitude_mark_ready();

    {
        const float corrected_roll = roll - attitude_roll_offset;
        const float corrected_pitch = pitch - attitude_pitch_offset;
        const float corrected_yaw =
            yaw_wrap(yaw - attitude_yaw_offset);
        const float yaw_delta =
            yaw_wrap(corrected_yaw - attitude_state.yaw);

        attitude_state.roll = (0.9f * attitude_state.roll) +
                              (0.1f * corrected_roll);
        attitude_state.pitch = (0.9f * attitude_state.pitch) +
                               (0.1f * corrected_pitch);
        attitude_state.yaw = yaw_wrap(attitude_state.yaw +
                                      (0.05f * yaw_delta));
    }
    ahrs_state = AHRS_READY;
}

attitude_state_t attitude_get_state(void)
{
    return attitude_state;
}

ahrs_state_t attitude_get_status(void)
{
    return ahrs_state;
}
