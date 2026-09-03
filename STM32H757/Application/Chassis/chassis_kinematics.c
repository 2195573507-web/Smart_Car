#include "chassis_kinematics.h"

/* 四轮差速运动学实现；创建人：待确认（当前维护人：Zhiqin）。 */

#include <math.h>
#include <stddef.h>

/** 实现头文件约定的 RR/RF/LR/LF 四轮差速换算和越界拒绝。 */
bool chassis_kinematics_compute(float linear_mm_s, float angular_rad_s,
                                float wheel_speed[CHASSIS_WHEEL_COUNT])
{
    const float half_track = 0.5f * CHASSIS_TRACK_WIDTH_MM;
    float left_speed;
    float right_speed;

    if (wheel_speed == NULL || !isfinite(linear_mm_s) ||
        !isfinite(angular_rad_s)) {
        return false;
    }

    /* Positive angular speed is a counterclockwise (left) turn: slow the
     * left wheels and accelerate the right wheels. */
    left_speed = linear_mm_s - angular_rad_s * half_track;
    right_speed = linear_mm_s + angular_rad_s * half_track;
    if (!isfinite(left_speed) || !isfinite(right_speed) ||
        fabsf(left_speed) > CHASSIS_WHEEL_SPEED_LIMIT_MM_S ||
        fabsf(right_speed) > CHASSIS_WHEEL_SPEED_LIMIT_MM_S) {
        return false;
    }

    wheel_speed[CHASSIS_WHEEL_RR] = right_speed;
    wheel_speed[CHASSIS_WHEEL_RF] = right_speed;
    wheel_speed[CHASSIS_WHEEL_LR] = left_speed;
    wheel_speed[CHASSIS_WHEEL_LF] = left_speed;
    return true;
}
