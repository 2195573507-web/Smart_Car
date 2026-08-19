#include "dual_ahrs.h"

#include <math.h>
#include <string.h>

#define DUAL_AHRS_MIN_DT 0.0005f
#define DUAL_AHRS_MAX_DT 0.0200f
#define DUAL_AHRS_LSM_STALE_US UINT64_C(250000)
#define DUAL_AHRS_PRIMARY_STALE_US 50000.0f
#define DUAL_AHRS_MAG_ALPHA 0.20f
#define DUAL_AHRS_PRIMARY_KP 1.8f
#define DUAL_AHRS_PRIMARY_KI 0.035f
#define DUAL_AHRS_BIQUAD_NOTCH_HZ 60.0f
#define DUAL_AHRS_BIQUAD_NOTCH_Q 2.0f
#define DUAL_AHRS_BIQUAD_LPF_HZ 90.0f
#define DUAL_AHRS_SAMPLE_HZ 400.0f

typedef struct
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float x1;
    float x2;
    float y1;
    float y2;
} dual_ahrs_biquad_t;

typedef struct
{
    dual_ahrs_biquad_t notch[6];
    dual_ahrs_biquad_t lpf[6];
    uint8_t decimation_phase;
    uint64_t last_bmi_timestamp_us;
    uint64_t last_lsm_timestamp_us;
    uint64_t last_lsm_input_timestamp_us;
    uint64_t last_primary_mag_timestamp_us;
    uint64_t last_mag_reference_timestamp_us;
    dual_ahrs_quaternion_t primary_q;
    dual_ahrs_vector3_t primary_integral;
    dual_ahrs_attitude_t primary;
    dual_ahrs_attitude_t redundant;
    dual_ahrs_vector3_t redundant_accel;
    dual_ahrs_vector3_t redundant_mag;
    dual_ahrs_vector3_t accel_history[5];
    dual_ahrs_vector3_t mag_history[5];
    uint8_t history_count;
    uint8_t history_index;
    float mag_reference_norm;
    dual_ahrs_bias_t bias;
    imu_leveling_state_t leveling_bmi;
    imu_leveling_state_t leveling_lsm;
    float local_gravity_mps2;
    float primary_yaw_offset;
    float redundant_yaw_offset;
    uint8_t primary_yaw_offset_valid;
    uint8_t redundant_yaw_offset_valid;
    uint8_t primary_zero_pending;
    uint8_t redundant_zero_pending;
    uint8_t bias_valid;
    uint32_t sample_sequence;
    dual_ahrs_output_t output;
    dual_ahrs_state_t state;
} dual_ahrs_context_t;

static dual_ahrs_context_t s_dual;

static void reset_runtime_state(void)
{
    uint8_t index;

    for (index = 0U; index < 6U; ++index) {
        s_dual.notch[index].x1 = 0.0f;
        s_dual.notch[index].x2 = 0.0f;
        s_dual.notch[index].y1 = 0.0f;
        s_dual.notch[index].y2 = 0.0f;
        s_dual.lpf[index].x1 = 0.0f;
        s_dual.lpf[index].x2 = 0.0f;
        s_dual.lpf[index].y1 = 0.0f;
        s_dual.lpf[index].y2 = 0.0f;
    }
    s_dual.decimation_phase = 0U;
    s_dual.last_bmi_timestamp_us = 0U;
    s_dual.last_lsm_timestamp_us = 0U;
    s_dual.last_lsm_input_timestamp_us = 0U;
    s_dual.last_primary_mag_timestamp_us = 0U;
    s_dual.last_mag_reference_timestamp_us = 0U;
    s_dual.primary_q = (dual_ahrs_quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    s_dual.primary_integral = (dual_ahrs_vector3_t){0.0f, 0.0f, 0.0f};
    s_dual.primary = (dual_ahrs_attitude_t){0};
    s_dual.primary.quaternion = s_dual.primary_q;
    s_dual.redundant = (dual_ahrs_attitude_t){0};
    s_dual.redundant.quaternion =
        (dual_ahrs_quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    s_dual.redundant_accel = (dual_ahrs_vector3_t){0.0f, 0.0f, 0.0f};
    s_dual.redundant_mag = (dual_ahrs_vector3_t){0.0f, 0.0f, 0.0f};
    (void)memset(s_dual.accel_history, 0, sizeof(s_dual.accel_history));
    (void)memset(s_dual.mag_history, 0, sizeof(s_dual.mag_history));
    s_dual.history_count = 0U;
    s_dual.history_index = 0U;
    s_dual.mag_reference_norm = 0.0f;
    s_dual.primary_yaw_offset = 0.0f;
    s_dual.redundant_yaw_offset = 0.0f;
    s_dual.primary_yaw_offset_valid = 0U;
    s_dual.redundant_yaw_offset_valid = 0U;
    s_dual.primary_zero_pending = 1U;
    s_dual.redundant_zero_pending = 1U;
    s_dual.sample_sequence = 0U;
    (void)memset(&s_dual.output, 0, sizeof(s_dual.output));
    s_dual.output.schema = DUAL_AHRS_SCHEMA;
    s_dual.output.primary.quaternion = s_dual.primary_q;
    s_dual.output.redundant.quaternion = s_dual.redundant.quaternion;
}

static uint8_t vector_is_finite(dual_ahrs_vector3_t value)
{
    return (uint8_t)(isfinite(value.x) != 0 && isfinite(value.y) != 0 &&
                     isfinite(value.z) != 0);
}

static dual_ahrs_vector3_t level_vector(const imu_leveling_state_t *leveling,
                                        dual_ahrs_vector3_t vector)
{
    const float input[3] = {vector.x, vector.y, vector.z};
    float output[3];

    imu_leveling_rotate_vector(leveling, input, output);
    return (dual_ahrs_vector3_t){output[0], output[1], output[2]};
}

static float clamp01(float value)
{
    if (!isfinite(value) || value <= 0.0f) {
        return 0.0f;
    }
    return value >= 1.0f ? 1.0f : value;
}

static float vector_norm(dual_ahrs_vector3_t value)
{
    return sqrtf(value.x * value.x + value.y * value.y + value.z * value.z);
}

static dual_ahrs_vector3_t vector_scale(dual_ahrs_vector3_t value, float scale)
{
    dual_ahrs_vector3_t result = {value.x * scale, value.y * scale,
                                  value.z * scale};
    return result;
}

static dual_ahrs_vector3_t vector_cross(dual_ahrs_vector3_t a,
                                        dual_ahrs_vector3_t b)
{
    dual_ahrs_vector3_t result = {a.y * b.z - a.z * b.y,
                                  a.z * b.x - a.x * b.z,
                                  a.x * b.y - a.y * b.x};
    return result;
}

static dual_ahrs_vector3_t vector_normalize(dual_ahrs_vector3_t value,
                                            uint8_t *valid)
{
    const float norm = vector_norm(value);
    if (valid != NULL) {
        *valid = (isfinite(norm) && norm > 1.0e-6f) ? 1U : 0U;
    }
    if (!isfinite(norm) || norm <= 1.0e-6f) {
        return (dual_ahrs_vector3_t){0.0f, 0.0f, 0.0f};
    }
    return vector_scale(value, 1.0f / norm);
}

float dual_ahrs_wrap_pi(float angle)
{
    if (!isfinite(angle)) {
        return 0.0f;
    }
    while (angle > DUAL_AHRS_PI) {
        angle -= DUAL_AHRS_TWO_PI;
    }
    while (angle <= -DUAL_AHRS_PI) {
        angle += DUAL_AHRS_TWO_PI;
    }
    return angle;
}

float delta_roll(float primary, float redundant)
{
    return dual_ahrs_wrap_pi(primary - redundant);
}

float delta_pitch(float primary, float redundant)
{
    return dual_ahrs_wrap_pi(primary - redundant);
}

float delta_yaw(float primary, float redundant)
{
    return dual_ahrs_wrap_pi(primary - redundant);
}

float gravity_confidence(const dual_ahrs_vector3_t *accel,
                         float radar_pwm_percent)
{
    float local_gravity = s_dual.local_gravity_mps2;
    float norm_error;
    float pwm_penalty;

    if (accel == NULL) {
        return 0.0f;
    }
    if (!isfinite(local_gravity) || local_gravity < IMU_LEVELING_G_MIN ||
        local_gravity > IMU_LEVELING_G_MAX) {
        local_gravity = IMU_LEVELING_G_DEFAULT_MPS2;
    }
    norm_error = fabsf(vector_norm(*accel) - local_gravity) / local_gravity;
    pwm_penalty = fmaxf(0.0f, fminf(radar_pwm_percent, 100.0f)) * 0.0035f;
    return clamp01(1.0f - (4.0f * norm_error) - pwm_penalty);
}

float mag_confidence(const dual_ahrs_vector3_t *mag)
{
    const float norm = mag == NULL ? 0.0f : vector_norm(*mag);
    float deviation;

    if (!isfinite(norm) || norm <= 1.0e-6f) {
        return 0.0f;
    }
    if (s_dual.mag_reference_norm <= 1.0e-6f) {
        return 1.0f;
    }
    deviation = fabsf(norm - s_dual.mag_reference_norm) /
                s_dual.mag_reference_norm;
    return clamp01(1.0f - (3.0f * deviation));
}

static dual_ahrs_biquad_t make_notch(float frequency, float q)
{
    const float omega = 2.0f * DUAL_AHRS_PI * frequency / DUAL_AHRS_SAMPLE_HZ;
    const float alpha = sinf(omega) / (2.0f * q);
    const float cos_omega = cosf(omega);
    const float a0 = 1.0f + alpha;
    dual_ahrs_biquad_t filter = {
        .b0 = 1.0f / a0,
        .b1 = -2.0f * cos_omega / a0,
        .b2 = 1.0f / a0,
        .a1 = -2.0f * cos_omega / a0,
        .a2 = (1.0f - alpha) / a0,
    };
    return filter;
}

static dual_ahrs_biquad_t make_lpf(float frequency)
{
    const float omega = 2.0f * DUAL_AHRS_PI * frequency / DUAL_AHRS_SAMPLE_HZ;
    const float cos_omega = cosf(omega);
    const float sin_omega = sinf(omega);
    const float alpha = sin_omega / (2.0f * 0.70710678f);
    const float a0 = 1.0f + alpha;
    dual_ahrs_biquad_t filter = {
        .b0 = ((1.0f - cos_omega) * 0.5f) / a0,
        .b1 = (1.0f - cos_omega) / a0,
        .b2 = ((1.0f - cos_omega) * 0.5f) / a0,
        .a1 = (-2.0f * cos_omega) / a0,
        .a2 = (1.0f - alpha) / a0,
    };
    return filter;
}

static float biquad_apply(dual_ahrs_biquad_t *filter, float input)
{
    float output;

    if (filter == NULL || !isfinite(input)) {
        return 0.0f;
    }
    output = filter->b0 * input + filter->b1 * filter->x1 +
             filter->b2 * filter->x2 - filter->a1 * filter->y1 -
             filter->a2 * filter->y2;
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;
    return output;
}

static dual_ahrs_vector3_t filter_vector(dual_ahrs_vector3_t value,
                                          dual_ahrs_biquad_t *notch,
                                          dual_ahrs_biquad_t *lpf)
{
    dual_ahrs_vector3_t result;
    result.x = biquad_apply(&lpf[0], biquad_apply(&notch[0], value.x));
    result.y = biquad_apply(&lpf[1], biquad_apply(&notch[1], value.y));
    result.z = biquad_apply(&lpf[2], biquad_apply(&notch[2], value.z));
    return result;
}

static dual_ahrs_quaternion_t quaternion_normalize(dual_ahrs_quaternion_t q)
{
    const float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (!isfinite(norm) || norm <= 1.0e-6f) {
        return (dual_ahrs_quaternion_t){1.0f, 0.0f, 0.0f, 0.0f};
    }
    q.w /= norm;
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    return q;
}

static dual_ahrs_attitude_t quaternion_to_attitude(dual_ahrs_quaternion_t q)
{
    dual_ahrs_attitude_t result;
    const float sin_pitch = 2.0f * (q.w * q.y - q.z * q.x);
    result.quaternion = q;
    result.roll = atan2f(2.0f * (q.w * q.x + q.y * q.z),
                         1.0f - 2.0f * (q.x * q.x + q.y * q.y));
    result.pitch = asinf(fmaxf(-1.0f, fminf(1.0f, sin_pitch)));
    result.yaw = atan2f(2.0f * (q.w * q.z + q.x * q.y),
                        1.0f - 2.0f * (q.y * q.y + q.z * q.z));
    result.valid = 1U;
    return result;
}

static dual_ahrs_quaternion_t euler_to_quaternion(float roll, float pitch,
                                                   float yaw)
{
    const float cr = cosf(roll * 0.5f);
    const float sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f);
    const float sy = sinf(yaw * 0.5f);
    dual_ahrs_quaternion_t q = {
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    };
    return quaternion_normalize(q);
}

static uint8_t attitude_from_accel_mag(dual_ahrs_vector3_t accel,
                                       dual_ahrs_vector3_t mag,
                                       dual_ahrs_attitude_t *attitude)
{
    dual_ahrs_vector3_t accel_unit;
    uint8_t accel_valid;
    uint8_t mag_valid;
    float horizontal_x;
    float horizontal_y;

    if (attitude == NULL) {
        return 0U;
    }
    accel_unit = vector_normalize(accel, &accel_valid);
    (void)vector_normalize(mag, &mag_valid);
    if (accel_valid == 0U || mag_valid == 0U) {
        return 0U;
    }

    attitude->roll = atan2f(accel_unit.y, accel_unit.z);
    attitude->pitch = atan2f(-accel_unit.x,
                             sqrtf(accel_unit.y * accel_unit.y +
                                   accel_unit.z * accel_unit.z));
    horizontal_x = mag.x * cosf(attitude->pitch) +
                   mag.z * sinf(attitude->pitch);
    horizontal_y = mag.x * sinf(attitude->roll) * sinf(attitude->pitch) +
                   mag.y * cosf(attitude->roll) -
                   mag.z * sinf(attitude->roll) * cosf(attitude->pitch);
    if (!isfinite(horizontal_x) || !isfinite(horizontal_y) ||
        (horizontal_x * horizontal_x + horizontal_y * horizontal_y) <=
            1.0e-12f) {
        return 0U;
    }

    /* The input magnetic Y axis has already been mirrored into the vehicle
     * convention, so this is the same tilt-compensated heading as the
     * redundant estimator and the Mahony magnetic reference. */
    attitude->yaw = dual_ahrs_wrap_pi(atan2f(-horizontal_y, horizontal_x));
    attitude->quaternion = euler_to_quaternion(attitude->roll,
                                                attitude->pitch,
                                                attitude->yaw);
    attitude->valid = 1U;
    return 1U;
}

static void seed_biquad_constant(dual_ahrs_biquad_t *filter, float value)
{
    if (filter == NULL || !isfinite(value)) {
        return;
    }
    filter->x1 = value;
    filter->x2 = value;
    filter->y1 = value;
    filter->y2 = value;
}

static void seed_primary_filters(dual_ahrs_vector3_t accel,
                                 dual_ahrs_vector3_t gyro)
{
    const float value[6] = {accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z};
    uint8_t index;

    for (index = 0U; index < 6U; ++index) {
        seed_biquad_constant(&s_dual.notch[index], value[index]);
        seed_biquad_constant(&s_dual.lpf[index], value[index]);
    }
}

static void initialize_static_attitudes(const dual_ahrs_input_t *input)
{
    dual_ahrs_attitude_t attitude;
    uint8_t index;

    if (input == NULL || input->lsm_mag_valid == 0U ||
        input->lsm_timestamp_us == 0U) {
        return;
    }

    if (s_dual.primary_yaw_offset_valid == 0U &&
        input->bmi_accel_valid != 0U && input->bmi_gyro_valid != 0U &&
        input->bmi_timestamp_us != 0U &&
        attitude_from_accel_mag(input->bmi_accel, input->mag, &attitude) != 0U) {
        s_dual.primary_q = attitude.quaternion;
        s_dual.primary = attitude;
        s_dual.primary_integral = (dual_ahrs_vector3_t){0.0f, 0.0f, 0.0f};
        s_dual.last_bmi_timestamp_us = input->bmi_timestamp_us;
        s_dual.last_primary_mag_timestamp_us = input->lsm_timestamp_us;
        seed_primary_filters(input->bmi_accel, input->gyro);
    }

    if (s_dual.redundant_yaw_offset_valid == 0U &&
        input->lsm_accel_valid != 0U &&
        attitude_from_accel_mag(input->lsm_accel, input->mag, &attitude) != 0U) {
        s_dual.redundant = attitude;
        s_dual.redundant_accel = input->lsm_accel;
        s_dual.redundant_mag = input->mag;
        for (index = 0U; index < 5U; ++index) {
            s_dual.accel_history[index] = input->lsm_accel;
            s_dual.mag_history[index] = input->mag;
        }
        s_dual.history_count = 5U;
        s_dual.history_index = 0U;
        s_dual.last_lsm_input_timestamp_us = input->lsm_timestamp_us;
        s_dual.last_lsm_timestamp_us = input->lsm_timestamp_us;
    }
}

static float median5(const float *values)
{
    float sorted[5];
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < 5U; ++i) {
        sorted[i] = values[i];
    }
    for (i = 1U; i < 5U; ++i) {
        const float value = sorted[i];
        j = i;
        while (j > 0U && sorted[j - 1U] > value) {
            sorted[j] = sorted[j - 1U];
            --j;
        }
        sorted[j] = value;
    }
    return sorted[2];
}

static dual_ahrs_vector3_t median_vector(const dual_ahrs_vector3_t *history)
{
    float x[5];
    float y[5];
    float z[5];
    uint8_t i;
    for (i = 0U; i < 5U; ++i) {
        x[i] = history[i].x;
        y[i] = history[i].y;
        z[i] = history[i].z;
    }
    return (dual_ahrs_vector3_t){median5(x), median5(y), median5(z)};
}

static void update_redundant(const dual_ahrs_input_t *input)
{
    dual_ahrs_vector3_t accel;
    dual_ahrs_vector3_t mag;
    dual_ahrs_vector3_t accel_unit;
    uint8_t accel_valid;
    uint8_t mag_valid;
    float horizontal_x;
    float horizontal_y;

    if (input == NULL || input->lsm_accel_valid == 0U ||
        input->lsm_mag_valid == 0U || input->lsm_timestamp_us == 0U) {
        return;
    }
    if (input->lsm_timestamp_us != 0U &&
        input->lsm_timestamp_us == s_dual.last_lsm_input_timestamp_us) {
        return;
    }
    s_dual.last_lsm_input_timestamp_us = input->lsm_timestamp_us;
    s_dual.accel_history[s_dual.history_index] = input->lsm_accel;
    s_dual.mag_history[s_dual.history_index] = input->mag;
    s_dual.history_index = (uint8_t)((s_dual.history_index + 1U) % 5U);
    if (s_dual.history_count < 5U) {
        ++s_dual.history_count;
    }
    if (s_dual.history_count < 5U) {
        return;
    }
    accel = median_vector(s_dual.accel_history);
    mag = median_vector(s_dual.mag_history);
    s_dual.redundant_accel.x = (1.0f - DUAL_AHRS_MAG_ALPHA) *
                                   s_dual.redundant_accel.x +
                               DUAL_AHRS_MAG_ALPHA * accel.x;
    s_dual.redundant_accel.y = (1.0f - DUAL_AHRS_MAG_ALPHA) *
                                   s_dual.redundant_accel.y +
                               DUAL_AHRS_MAG_ALPHA * accel.y;
    s_dual.redundant_accel.z = (1.0f - DUAL_AHRS_MAG_ALPHA) *
                                   s_dual.redundant_accel.z +
                               DUAL_AHRS_MAG_ALPHA * accel.z;
    s_dual.redundant_mag.x = (1.0f - DUAL_AHRS_MAG_ALPHA) *
                                 s_dual.redundant_mag.x +
                             DUAL_AHRS_MAG_ALPHA * mag.x;
    s_dual.redundant_mag.y = (1.0f - DUAL_AHRS_MAG_ALPHA) *
                                 s_dual.redundant_mag.y +
                             DUAL_AHRS_MAG_ALPHA * mag.y;
    s_dual.redundant_mag.z = (1.0f - DUAL_AHRS_MAG_ALPHA) *
                                 s_dual.redundant_mag.z +
                             DUAL_AHRS_MAG_ALPHA * mag.z;
    accel_unit = vector_normalize(s_dual.redundant_accel, &accel_valid);
    (void)vector_normalize(s_dual.redundant_mag, &mag_valid);
    if (accel_valid == 0U || mag_valid == 0U) {
        return;
    }
    s_dual.redundant.roll = atan2f(accel_unit.y, accel_unit.z);
    s_dual.redundant.pitch = atan2f(-accel_unit.x,
                                    sqrtf(accel_unit.y * accel_unit.y +
                                          accel_unit.z * accel_unit.z));
    horizontal_x = s_dual.redundant_mag.x * cosf(s_dual.redundant.pitch) +
                   s_dual.redundant_mag.z * sinf(s_dual.redundant.pitch);
    horizontal_y = s_dual.redundant_mag.x * sinf(s_dual.redundant.roll) *
                       sinf(s_dual.redundant.pitch) +
                   s_dual.redundant_mag.y * cosf(s_dual.redundant.roll) -
                   s_dual.redundant_mag.z * sinf(s_dual.redundant.roll) *
                       cosf(s_dual.redundant.pitch);
    /* LSM303 magnetic Y is opposite to the vehicle yaw convention. */
    s_dual.redundant.yaw = dual_ahrs_wrap_pi(atan2f(-horizontal_y,
                                                    horizontal_x));
    s_dual.redundant.quaternion = euler_to_quaternion(
        s_dual.redundant.roll, s_dual.redundant.pitch, s_dual.redundant.yaw);
    s_dual.redundant.valid = 1U;
    s_dual.last_lsm_timestamp_us = input->lsm_timestamp_us;
}

static void update_primary(const dual_ahrs_input_t *input,
                           dual_ahrs_vector3_t accel,
                           dual_ahrs_vector3_t gyro)
{
    dual_ahrs_vector3_t mag;
    dual_ahrs_vector3_t accel_unit;
    dual_ahrs_vector3_t mag_unit;
    dual_ahrs_vector3_t gravity;
    dual_ahrs_vector3_t magnetic;
    dual_ahrs_vector3_t error;
    uint8_t accel_valid;
    uint8_t mag_valid;
    float dt;
    float half_dt;
    dual_ahrs_quaternion_t q;
    dual_ahrs_quaternion_t previous_q;

    if (input == NULL || input->bmi_accel_valid == 0U ||
        input->bmi_gyro_valid == 0U || input->bmi_timestamp_us == 0U) {
        return;
    }
    if (s_dual.last_bmi_timestamp_us == 0U) {
        s_dual.last_bmi_timestamp_us = input->bmi_timestamp_us;
        return;
    }
    if (input->bmi_timestamp_us <= s_dual.last_bmi_timestamp_us) {
        return;
    }
    dt = (float)(input->bmi_timestamp_us - s_dual.last_bmi_timestamp_us) /
         1000000.0f;
    s_dual.last_bmi_timestamp_us = input->bmi_timestamp_us;
    if (dt < DUAL_AHRS_MIN_DT || dt > DUAL_AHRS_MAX_DT) {
        dt = 1.0f / DUAL_AHRS_SAMPLE_HZ;
    }
    mag = input->mag;
    accel_unit = vector_normalize(accel, &accel_valid);
    mag_unit = vector_normalize(mag, &mag_valid);
    mag_valid = (uint8_t)(mag_valid != 0U && input->lsm_mag_valid != 0U &&
                          input->lsm_timestamp_us != 0U &&
                          input->lsm_timestamp_us >
                              s_dual.last_primary_mag_timestamp_us);
    if (accel_valid == 0U) {
        return;
    }

    q = s_dual.primary_q;
    previous_q = q;
    gravity = (dual_ahrs_vector3_t){
        2.0f * (q.x * q.z - q.w * q.y),
        2.0f * (q.w * q.x + q.y * q.z),
        q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z};
    error = vector_scale(vector_cross(accel_unit, gravity),
                         gravity_confidence(&accel,
                                             (float)input->radar_pwm_percent));
    if (mag_valid != 0U) {
        const float hx = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
        const float hy = 2.0f * (q.x * q.y - q.w * q.z);
        const float hz = 2.0f * (q.x * q.z + q.w * q.y);
        const float magnetic_weight = mag_confidence(&mag);
        magnetic = vector_normalize((dual_ahrs_vector3_t){hx, hy, hz}, NULL);
        const dual_ahrs_vector3_t magnetic_error = vector_cross(
            mag_unit, magnetic);
        /* R_level and gravity own roll/pitch. Feeding the magnetic vertical
         * component back into X/Y would reintroduce static tilt after
         * leveling, so the magnetometer corrects heading only. */
        error.z += magnetic_error.z * magnetic_weight;
        s_dual.last_primary_mag_timestamp_us = input->lsm_timestamp_us;
    }
    s_dual.primary_integral.x += DUAL_AHRS_PRIMARY_KI * error.x * dt;
    s_dual.primary_integral.y += DUAL_AHRS_PRIMARY_KI * error.y * dt;
    s_dual.primary_integral.z += DUAL_AHRS_PRIMARY_KI * error.z * dt;
    s_dual.primary_integral.x = fmaxf(-0.25f, fminf(0.25f,
                                                    s_dual.primary_integral.x));
    s_dual.primary_integral.y = fmaxf(-0.25f, fminf(0.25f,
                                                    s_dual.primary_integral.y));
    s_dual.primary_integral.z = fmaxf(-0.25f, fminf(0.25f,
                                                    s_dual.primary_integral.z));
    {
        const float gx = gyro.x + DUAL_AHRS_PRIMARY_KP * error.x +
                         s_dual.primary_integral.x;
        const float gy = gyro.y + DUAL_AHRS_PRIMARY_KP * error.y +
                         s_dual.primary_integral.y;
        const float gz = gyro.z + DUAL_AHRS_PRIMARY_KP * error.z +
                         s_dual.primary_integral.z;
        half_dt = 0.5f * dt;
        q.w += (-q.x * gx - q.y * gy - q.z * gz) * half_dt;
        q.x += (previous_q.w * gx + previous_q.y * gz - previous_q.z * gy) *
               half_dt;
        q.y += (previous_q.w * gy - previous_q.x * gz + previous_q.z * gx) *
               half_dt;
        q.z += (previous_q.w * gz + previous_q.x * gy - previous_q.y * gx) *
               half_dt;
    }
    s_dual.primary_q = quaternion_normalize(q);
    s_dual.primary = quaternion_to_attitude(s_dual.primary_q);
}

static dual_ahrs_attitude_t zero_reference_attitude(
    const dual_ahrs_attitude_t *raw, float *yaw_offset,
    uint8_t *yaw_offset_valid, uint8_t *zero_pending)
{
    dual_ahrs_attitude_t output = {0};

    if (raw == NULL || yaw_offset == NULL || yaw_offset_valid == NULL ||
        zero_pending == NULL) {
        return output;
    }

    output = *raw;
    if (raw->valid == 0U) {
        return output;
    }
    if (*yaw_offset_valid == 0U) {
        output.roll = 0.0f;
        output.pitch = 0.0f;
        output.yaw = 0.0f;
        return output;
    }

    /* Roll and pitch stay in the leveled quaternion frame. Only yaw has a
     * scalar reference, so no Euler roll/pitch subtraction is introduced. */
    output.yaw = dual_ahrs_wrap_pi(raw->yaw - *yaw_offset);
    if (*zero_pending != 0U) {
        output.roll = 0.0f;
        output.pitch = 0.0f;
        output.yaw = 0.0f;
        *zero_pending = 0U;
    }
    return output;
}

static void capture_ready_yaw_offsets(void)
{
    if (s_dual.primary.valid == 0U || s_dual.redundant.valid == 0U ||
        (s_dual.primary_yaw_offset_valid != 0U &&
         s_dual.redundant_yaw_offset_valid != 0U)) {
        return;
    }

    /* The estimator is intentionally frozen before IMU_READY, so the first
     * simultaneous valid post-READY pair is the earliest physical heading
     * available for both independent references. */
    s_dual.primary_yaw_offset = s_dual.primary.yaw;
    s_dual.redundant_yaw_offset = s_dual.redundant.yaw;
    s_dual.primary_yaw_offset_valid = 1U;
    s_dual.redundant_yaw_offset_valid = 1U;
}

void dual_ahrs_init(void)
{
    uint8_t index;
    (void)memset(&s_dual, 0, sizeof(s_dual));
    for (index = 0U; index < 6U; ++index) {
        s_dual.notch[index] = make_notch(DUAL_AHRS_BIQUAD_NOTCH_HZ,
                                         DUAL_AHRS_BIQUAD_NOTCH_Q);
        s_dual.lpf[index] = make_lpf(DUAL_AHRS_BIQUAD_LPF_HZ);
    }
    imu_leveling_init(&s_dual.leveling_bmi);
    imu_leveling_init(&s_dual.leveling_lsm);
    s_dual.local_gravity_mps2 = IMU_LEVELING_G_DEFAULT_MPS2;
    reset_runtime_state();
    s_dual.state = DUAL_AHRS_STATE_WAIT_CAL;
    s_dual.output.state = s_dual.state;
}

void dual_ahrs_set_leveling(const imu_leveling_state_t *bmi,
                            const imu_leveling_state_t *lsm)
{
    if (bmi != NULL) {
        s_dual.leveling_bmi = *bmi;
    } else {
        imu_leveling_init(&s_dual.leveling_bmi);
    }
    if (lsm != NULL) {
        s_dual.leveling_lsm = *lsm;
    } else {
        imu_leveling_init(&s_dual.leveling_lsm);
    }
}

void dual_ahrs_set_local_gravity(float gravity_mps2)
{
    s_dual.local_gravity_mps2 =
        isfinite(gravity_mps2) && gravity_mps2 >= IMU_LEVELING_G_MIN &&
                gravity_mps2 <= IMU_LEVELING_G_MAX
            ? gravity_mps2
            : IMU_LEVELING_G_DEFAULT_MPS2;
}

void dual_ahrs_set_bias(const dual_ahrs_bias_t *bias)
{
    if (bias == NULL || vector_is_finite(bias->bmi_accel) == 0U ||
        vector_is_finite(bias->bmi_gyro) == 0U ||
        vector_is_finite(bias->lsm_accel) == 0U) {
        s_dual.bias_valid = 0U;
        reset_runtime_state();
        s_dual.state = DUAL_AHRS_STATE_WAIT_CAL;
        s_dual.output.state = s_dual.state;
        return;
    }

    s_dual.bias = *bias;
    s_dual.bias_valid = 1U;
    reset_runtime_state();
    s_dual.state = DUAL_AHRS_STATE_READY;
    s_dual.output.state = s_dual.state;
}

void dual_ahrs_update(const dual_ahrs_input_t *input)
{
    float primary_age = 0.0f;
    float lsm_age = 0.0f;
    uint64_t reference_timestamp_us = 0U;
    uint8_t primary_valid;
    uint8_t redundant_valid;
    dual_ahrs_vector3_t delta;
    dual_ahrs_vector3_t filtered_accel;
    dual_ahrs_vector3_t filtered_gyro;

    if (input == NULL) {
        return;
    }
    if (s_dual.state == DUAL_AHRS_STATE_WAIT_CAL ||
        s_dual.bias_valid == 0U) {
        /* The calibration gate is deliberately before every filter, history,
         * timestamp, and gyro integration operation. */
        return;
    }

    dual_ahrs_input_t calibrated_input = *input;
    calibrated_input.bmi_accel.x -= s_dual.bias.bmi_accel.x;
    calibrated_input.bmi_accel.y -= s_dual.bias.bmi_accel.y;
    calibrated_input.bmi_accel.z -= s_dual.bias.bmi_accel.z;
    calibrated_input.gyro.x -= s_dual.bias.bmi_gyro.x;
    calibrated_input.gyro.y -= s_dual.bias.bmi_gyro.y;
    calibrated_input.gyro.z -= s_dual.bias.bmi_gyro.z;
    /* Normalize the BMI323 Z gyro into the vehicle yaw convention after bias
     * removal, so the calibrated bias remains in the sensor frame. */
    calibrated_input.gyro.z = -calibrated_input.gyro.z;
    calibrated_input.lsm_accel.x -= s_dual.bias.lsm_accel.x;
    calibrated_input.lsm_accel.y -= s_dual.bias.lsm_accel.y;
    calibrated_input.lsm_accel.z -= s_dual.bias.lsm_accel.z;
    /* The frozen boot-time matrices are applied after sensor-frame bias
     * removal and before every DualAHRS filter/estimator operation. */
    calibrated_input.bmi_accel =
        level_vector(&s_dual.leveling_bmi, calibrated_input.bmi_accel);
    calibrated_input.gyro =
        level_vector(&s_dual.leveling_bmi, calibrated_input.gyro);
    calibrated_input.lsm_accel =
        level_vector(&s_dual.leveling_lsm, calibrated_input.lsm_accel);
    calibrated_input.mag =
        level_vector(&s_dual.leveling_lsm, calibrated_input.mag);
    input = &calibrated_input;
    if (input->bmi_accel_valid == 0U || input->bmi_gyro_valid == 0U ||
        input->bmi_timestamp_us == 0U) {
        s_dual.primary.valid = 0U;
    }
    if (input->lsm_accel_valid == 0U || input->lsm_mag_valid == 0U ||
        input->lsm_timestamp_us == 0U) {
        s_dual.redundant.valid = 0U;
    }
    /* R_level has already aligned gravity with +Z. Establish both filters
     * directly from that first static frame so their zero references cannot
     * be captured while the Mahony/IIR state is still converging from zero. */
    initialize_static_attitudes(input);
    update_redundant(input);
    if (input->lsm_mag_valid != 0U && input->lsm_timestamp_us != 0U &&
        input->lsm_timestamp_us > s_dual.last_mag_reference_timestamp_us) {
        const float norm = vector_norm(input->mag);
        if (s_dual.mag_reference_norm <= 1.0e-6f && isfinite(norm)) {
            s_dual.mag_reference_norm = norm;
        } else if (isfinite(norm)) {
            s_dual.mag_reference_norm =
                0.995f * s_dual.mag_reference_norm + 0.005f * norm;
        }
        s_dual.last_mag_reference_timestamp_us = input->lsm_timestamp_us;
    }
    if (input->bmi_accel_valid != 0U && input->bmi_gyro_valid != 0U &&
        input->bmi_timestamp_us != 0U) {
        /* Filter every acquired sample before dropping every second sample.
         * Running the biquads only on the decimated stream would move their
         * effective cutoff and would not provide the required anti-aliasing. */
        filtered_accel = filter_vector(input->bmi_accel, s_dual.notch,
                                       s_dual.lpf);
        filtered_gyro = filter_vector(input->gyro, &s_dual.notch[3],
                                      &s_dual.lpf[3]);
        if (s_dual.decimation_phase == 0U) {
            update_primary(input, filtered_accel, filtered_gyro);
        }
        s_dual.decimation_phase ^= 1U;
    }
    capture_ready_yaw_offsets();
    s_dual.output.primary = zero_reference_attitude(
        &s_dual.primary, &s_dual.primary_yaw_offset,
        &s_dual.primary_yaw_offset_valid, &s_dual.primary_zero_pending);
    s_dual.output.redundant = zero_reference_attitude(
        &s_dual.redundant, &s_dual.redundant_yaw_offset,
        &s_dual.redundant_yaw_offset_valid, &s_dual.redundant_zero_pending);
    primary_valid = s_dual.output.primary.valid;
    redundant_valid = s_dual.output.redundant.valid;
    s_dual.output.gravity_confidence = gravity_confidence(
        &input->bmi_accel, (float)input->radar_pwm_percent);
    s_dual.output.magnetic_confidence = mag_confidence(&input->mag);
    if (primary_valid != 0U && redundant_valid != 0U) {
        delta.x = delta_roll(s_dual.output.primary.roll, s_dual.redundant.roll);
        delta.y = delta_pitch(s_dual.output.primary.pitch,
                              s_dual.redundant.pitch);
        delta.z = delta_yaw(s_dual.output.primary.yaw, s_dual.redundant.yaw);
        s_dual.output.delta_rad = delta;
    } else {
        s_dual.output.delta_rad = (dual_ahrs_vector3_t){0.0f, 0.0f, 0.0f};
    }
    if (input->bmi_timestamp_us > reference_timestamp_us) {
        reference_timestamp_us = input->bmi_timestamp_us;
    }
    if (input->lsm_timestamp_us > reference_timestamp_us) {
        reference_timestamp_us = input->lsm_timestamp_us;
    }
    if (reference_timestamp_us >= s_dual.last_bmi_timestamp_us &&
        s_dual.last_bmi_timestamp_us != 0U) {
        primary_age = (float)(reference_timestamp_us -
                              s_dual.last_bmi_timestamp_us);
    } else if (s_dual.last_bmi_timestamp_us != 0U) {
        primary_age = DUAL_AHRS_PRIMARY_STALE_US;
    }
    if (reference_timestamp_us >= s_dual.last_lsm_timestamp_us &&
        s_dual.last_lsm_timestamp_us != 0U) {
        lsm_age = (float)(reference_timestamp_us -
                          s_dual.last_lsm_timestamp_us);
    } else if (s_dual.last_lsm_timestamp_us != 0U) {
        lsm_age = (float)DUAL_AHRS_LSM_STALE_US;
    }
    if (primary_valid != 0U && redundant_valid != 0U &&
        primary_age < DUAL_AHRS_PRIMARY_STALE_US &&
        lsm_age < (float)DUAL_AHRS_LSM_STALE_US) {
        s_dual.state = DUAL_AHRS_STATE_TRACKING;
    } else if (primary_valid != 0U || redundant_valid != 0U) {
        s_dual.state = DUAL_AHRS_STATE_DEGRADED;
    } else if (s_dual.state == DUAL_AHRS_STATE_READY) {
        /* Bias hand-off has completed, but no valid sample has arrived yet. */
        s_dual.state = DUAL_AHRS_STATE_READY;
    } else if (input->bmi_accel_valid != 0U || input->lsm_accel_valid != 0U) {
        s_dual.state = DUAL_AHRS_STATE_WARMUP;
    } else {
        s_dual.state = DUAL_AHRS_STATE_FAULT;
    }
    s_dual.output.state = s_dual.state;
    s_dual.output.schema = DUAL_AHRS_SCHEMA;
    s_dual.output.timestamp_ms = (uint32_t)(input->bmi_timestamp_us != 0U
                                                ? input->bmi_timestamp_us / 1000U
                                                : input->lsm_timestamp_us / 1000U);
    s_dual.output.sample_sequence = ++s_dual.sample_sequence;
    s_dual.output.flags = (uint8_t)((primary_valid != 0U ? 0x01U : 0U) |
                                    (redundant_valid != 0U ? 0x02U : 0U) |
                                    (input->lsm_mag_valid != 0U ? 0x04U : 0U) |
                                    (s_dual.output.gravity_confidence < 0.5f
                                         ? 0x10U
                                         : 0U) |
                                    (s_dual.output.magnetic_confidence < 0.25f
                                         ? 0x20U
                                         : 0U) |
                                    ((primary_age >= DUAL_AHRS_PRIMARY_STALE_US ||
                                      lsm_age >= (float)DUAL_AHRS_LSM_STALE_US)
                                         ? 0x40U
                                         : 0U) |
                                    (s_dual.state == DUAL_AHRS_STATE_FAULT
                                         ? 0x80U
                                         : 0U));
}

void dual_ahrs_get_output(dual_ahrs_output_t *output)
{
    if (output != NULL) {
        *output = s_dual.output;
    }
}

static void put_u16_le(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)(value >> 8U);
}

static void put_u32_le(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)((value >> 8U) & 0xFFU);
    destination[2] = (uint8_t)((value >> 16U) & 0xFFU);
    destination[3] = (uint8_t)(value >> 24U);
}

static void put_float_le(uint8_t *destination, float value)
{
    uint32_t bits;
    (void)memcpy(&bits, &value, sizeof(bits));
    put_u32_le(destination, bits);
}

static void put_attitude(uint8_t *destination, const dual_ahrs_attitude_t *attitude)
{
    put_float_le(&destination[0], attitude->roll);
    put_float_le(&destination[4], attitude->pitch);
    put_float_le(&destination[8], attitude->yaw);
    put_float_le(&destination[12], attitude->quaternion.w);
    put_float_le(&destination[16], attitude->quaternion.x);
    put_float_le(&destination[20], attitude->quaternion.y);
    put_float_le(&destination[24], attitude->quaternion.z);
}

int dual_ahrs_pack_payload(uint8_t *payload, size_t capacity)
{
    if (payload == NULL || capacity < DUAL_AHRS_PAYLOAD_LENGTH) {
        return -1;
    }
    payload[0] = DUAL_AHRS_SCHEMA;
    payload[1] = s_dual.output.flags;
    put_u16_le(&payload[2], 0U);
    put_u32_le(&payload[4], s_dual.output.timestamp_ms);
    put_u32_le(&payload[8], s_dual.output.sample_sequence);
    put_attitude(&payload[12], &s_dual.output.primary);
    put_attitude(&payload[40], &s_dual.output.redundant);
    put_float_le(&payload[68], s_dual.output.delta_rad.x);
    put_float_le(&payload[72], s_dual.output.delta_rad.y);
    put_float_le(&payload[76], s_dual.output.delta_rad.z);
    return (int)DUAL_AHRS_PAYLOAD_LENGTH;
}
