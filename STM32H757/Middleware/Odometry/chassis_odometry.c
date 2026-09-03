#include "chassis_odometry.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* 基于四轮平均速度和主 AHRS 航向的平面里程计实现；
 * 创建人：待确认（当前维护人：Zhiqin）。 */

void chassis_odometry_init(chassis_odometry_state_t *state)
{
    if (state != NULL) {
        (void)memset(state, 0, sizeof(*state));
    }
}

void chassis_odometry_invalidate(chassis_odometry_state_t *state)
{
    if (state != NULL) {
        state->has_time_anchor = false;
        state->valid = false;
    }
}

chassis_odometry_result_t chassis_odometry_update(
    chassis_odometry_state_t *state,
    const float wheel_speed_mm_s[CHASSIS_ODOMETRY_WHEEL_COUNT],
    float primary_yaw_rad,
    uint32_t sample_timestamp_ms)
{
    float speed_sum = 0.0f;
    float center_speed_mm_s;
    float distance_mm;
    float next_x_mm;
    float next_y_mm;
    float next_total_distance_m;
    uint32_t elapsed_ms;

    if (state == NULL || wheel_speed_mm_s == NULL ||
        !isfinite(primary_yaw_rad)) {
        chassis_odometry_invalidate(state);
        return CHASSIS_ODOMETRY_RESULT_INVALID;
    }
    for (size_t index = 0U; index < CHASSIS_ODOMETRY_WHEEL_COUNT; ++index) {
        if (!isfinite(wheel_speed_mm_s[index])) {
            chassis_odometry_invalidate(state);
            return CHASSIS_ODOMETRY_RESULT_INVALID;
        }
        speed_sum += wheel_speed_mm_s[index];
    }
    if (!isfinite(speed_sum) || !isfinite(state->x_mm) ||
        !isfinite(state->y_mm) || !isfinite(state->total_distance_m) ||
        state->total_distance_m < 0.0f) {
        chassis_odometry_init(state);
        return CHASSIS_ODOMETRY_RESULT_INVALID;
    }

    if (!state->has_time_anchor) {
        state->last_sample_timestamp_ms = sample_timestamp_ms;
        state->yaw_rad = primary_yaw_rad;
        state->has_time_anchor = true;
        state->valid = true;
        return CHASSIS_ODOMETRY_RESULT_ANCHORED;
    }

    elapsed_ms = sample_timestamp_ms - state->last_sample_timestamp_ms;
    state->last_sample_timestamp_ms = sample_timestamp_ms;
    state->yaw_rad = primary_yaw_rad;
    if (elapsed_ms == 0U ||
        elapsed_ms > CHASSIS_ODOMETRY_MAX_SAMPLE_INTERVAL_MS) {
        state->valid = false;
        return CHASSIS_ODOMETRY_RESULT_INVALID;
    }

    center_speed_mm_s = speed_sum / (float)CHASSIS_ODOMETRY_WHEEL_COUNT;
    distance_mm = center_speed_mm_s * ((float)elapsed_ms / 1000.0f);
    next_x_mm = state->x_mm + distance_mm * cosf(primary_yaw_rad);
    next_y_mm = state->y_mm + distance_mm * sinf(primary_yaw_rad);
    next_total_distance_m = state->total_distance_m + fabsf(distance_mm) / 1000.0f;
    if (!isfinite(center_speed_mm_s) || !isfinite(distance_mm) ||
        !isfinite(next_x_mm) || !isfinite(next_y_mm) ||
        !isfinite(next_total_distance_m)) {
        chassis_odometry_invalidate(state);
        return CHASSIS_ODOMETRY_RESULT_INVALID;
    }

    state->x_mm = next_x_mm;
    state->y_mm = next_y_mm;
    state->total_distance_m = next_total_distance_m;
    state->valid = true;
    return CHASSIS_ODOMETRY_RESULT_UPDATED;
}
