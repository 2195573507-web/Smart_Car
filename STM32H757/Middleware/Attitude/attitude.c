#include "attitude.h"

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

static void attitude_mark_ready(void)
{
    attitude_zero_active = 0U;
    attitude_zero_ready = 1U;
    ahrs_state = AHRS_READY;
}

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

uint8_t attitude_zero_is_ready(void)
{
    return attitude_zero_ready;
}

uint8_t attitude_zero_capture_current(void)
{
    const imu_filtered_data_t filtered = imu_filter_get_output();
    const float roll = atan2f(filtered.ay, filtered.az);
    const float pitch = atan2f(-filtered.ax,
                               sqrtf((filtered.ay * filtered.ay) +
                                     (filtered.az * filtered.az)));
    const float yaw = atan2f(-filtered.my, filtered.mx);

    if (imu_calibration_is_complete() == 0U || filtered.online == 0U ||
        !isfinite(filtered.ax) || !isfinite(filtered.ay) ||
        !isfinite(filtered.az) || !isfinite(filtered.mx) ||
        !isfinite(filtered.my) || !isfinite(filtered.mz)) {
        return 0U;
    }
    attitude_roll_offset = roll;
    attitude_pitch_offset = pitch;
    attitude_yaw_offset = yaw;
    attitude_zero_sample_count = 0U;
    attitude_zero_roll_sum = 0.0f;
    attitude_zero_pitch_sum = 0.0f;
    attitude_zero_yaw_sin_sum = 0.0f;
    attitude_zero_yaw_cos_sum = 0.0f;
    attitude_state.roll = 0.0f;
    attitude_state.pitch = 0.0f;
    attitude_state.yaw = 0.0f;
    attitude_mark_ready();
    return 1U;
}

void attitude_update(void)
{
    const imu_filtered_data_t filtered = imu_filter_get_output();
    const float roll = atan2f(filtered.ay, filtered.az);
    const float pitch = atan2f(-filtered.ax,
                               sqrtf((filtered.ay * filtered.ay) +
                                     (filtered.az * filtered.az)));
    /* The LSM303 Y axis is mirrored in the vehicle horizontal plane. Keep
     * the legacy path aligned with the body-frame map used by DualAHRS. */
    const float yaw = atan2f(-filtered.my, filtered.mx);

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

    /* Keep the externally visible state coherent if a fast-zero path marks
     * the reference ready without entering the 500-sample accumulator. */
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
