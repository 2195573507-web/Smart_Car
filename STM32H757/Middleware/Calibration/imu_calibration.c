#include "imu_calibration.h"

#include <float.h>
#include <math.h>
#include <string.h>

#include "imu_time.h"

/* All calibration accumulators store SI units. BMI323 is configured for the
 * +/-4 g range, where one raw LSB is 9.80665/8192 m/s^2. The active manager
 * already performs this conversion; the bounded check below only protects
 * this API when a raw BMI323 sample is supplied by an alternate caller. */
#define IMU_CALIBRATION_GRAVITY_MPS2 (9.80665f)
#define IMU_CALIBRATION_BMI323_LSB_PER_G (8192.0f)
#define IMU_CALIBRATION_RAW_ACCEL_THRESHOLD_MPS2 (100.0f)
#define IMU_CALIBRATION_GYRO_MOTION_THRESHOLD_RADPS (0.15f)
#define IMU_CALIBRATION_ACCEL_MOTION_DELTA_MPS2 (1.0f)

#if defined(IMU_MANAGER_USE_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

typedef struct
{
    double sum_x;
    double sum_y;
    double sum_z;
    double sum_square_x;
    double sum_square_y;
    double sum_square_z;
    uint32_t sample_count;
} imu_axis_accumulator_t;

typedef struct
{
    imu_axis_accumulator_t lsm_accel;
    imu_axis_accumulator_t bmi_accel;
    imu_axis_accumulator_t bmi_gyro;
    imu_calibration_bias_t bias;
    imu_calibration_result_t result;
    imu_calibrated_data_t last_calibrated;
    uint64_t window_start_timestamp_us;
    uint16_t bmi_configured_rate_hz;
    uint8_t window_active;
    uint8_t complete;
    uint8_t lsm_observed;
    uint8_t bmi_accel_observed;
    uint8_t bmi_gyro_observed;
    uint8_t static_motion_detected;
    imu_calibration_quality_t quality;
    imu_calibration_static_statistics_t static_statistics;
} imu_calibration_state_t;

static imu_calibration_state_t s_calibration;
static volatile uint8_t s_bmi_capture_active;

#if defined(IMU_MANAGER_USE_FREERTOS)
static SemaphoreHandle_t s_mutex;
#endif

static void lock_calibration(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex != NULL) {
        (void)xSemaphoreTake(s_mutex, portMAX_DELAY);
    }
#endif
}

static void unlock_calibration(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex != NULL) {
        (void)xSemaphoreGive(s_mutex);
    }
#endif
}

static uint8_t try_lock_calibration(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex == NULL) {
        return 0U;
    }
    return xSemaphoreTake(s_mutex, (TickType_t)0) == pdTRUE ? 1U : 0U;
#else
    return 1U;
#endif
}

static uint8_t finite_xyz(float x, float y, float z)
{
    return isfinite(x) && isfinite(y) && isfinite(z) ? 1U : 0U;
}

static void bmi323_accel_input_to_mps2(float accel[3])
{
    const float scale = IMU_CALIBRATION_GRAVITY_MPS2 /
                        IMU_CALIBRATION_BMI323_LSB_PER_G;

    if (accel == NULL || finite_xyz(accel[0], accel[1], accel[2]) == 0U) {
        return;
    }
    /* Detect units from the complete vector so a near-zero raw axis is never
     * left in LSB while the gravity-bearing axes are converted to m/s^2. */
    if (fmaxf(fabsf(accel[0]), fmaxf(fabsf(accel[1]), fabsf(accel[2]))) >
        IMU_CALIBRATION_RAW_ACCEL_THRESHOLD_MPS2) {
        accel[0] *= scale;
        accel[1] *= scale;
        accel[2] *= scale;
    }
}

static uint8_t sum_add_is_safe(double sum, double value)
{
    return isfinite(sum) && isfinite(value) && isfinite(sum + value) ? 1U : 0U;
}

static uint8_t sum_square_add_is_safe(double sum_square, double value)
{
    const double square = value * value;

    if (!isfinite(sum_square) || sum_square < 0.0 || !isfinite(square)) {
        return 0U;
    }
    return square <= (DBL_MAX - sum_square) ? 1U : 0U;
}

static uint8_t accumulate_xyz(imu_axis_accumulator_t *accumulator,
                              float x, float y, float z)
{
    const double value_x = (double)x;
    const double value_y = (double)y;
    const double value_z = (double)z;

    if (accumulator == NULL || accumulator->sample_count == UINT32_MAX ||
        finite_xyz(x, y, z) == 0U ||
        sum_add_is_safe(accumulator->sum_x, value_x) == 0U ||
        sum_add_is_safe(accumulator->sum_y, value_y) == 0U ||
        sum_add_is_safe(accumulator->sum_z, value_z) == 0U ||
        sum_square_add_is_safe(accumulator->sum_square_x, value_x) == 0U ||
        sum_square_add_is_safe(accumulator->sum_square_y, value_y) == 0U ||
        sum_square_add_is_safe(accumulator->sum_square_z, value_z) == 0U) {
        return 0U;
    }
    accumulator->sum_x += value_x;
    accumulator->sum_y += value_y;
    accumulator->sum_z += value_z;
    accumulator->sum_square_x += value_x * value_x;
    accumulator->sum_square_y += value_y * value_y;
    accumulator->sum_square_z += value_z * value_z;
    ++accumulator->sample_count;
    return 1U;
}

static imu_bias_xyz_t mean_xyz(const imu_axis_accumulator_t *accumulator)
{
    imu_bias_xyz_t result = {0};
    if (accumulator == NULL || accumulator->sample_count == 0U) {
        return result;
    }
    const double count = (double)accumulator->sample_count;
    result.x = (float)(accumulator->sum_x / count);
    result.y = (float)(accumulator->sum_y / count);
    result.z = (float)(accumulator->sum_z / count);
    return result;
}

static uint32_t expected_sample_count(uint16_t configured_rate_hz,
                                      uint64_t duration_us)
{
    const uint64_t expected =
        ((uint64_t)configured_rate_hz * duration_us) / UINT64_C(1000000);

    return expected > UINT32_MAX ? UINT32_MAX : (uint32_t)expected;
}

static uint32_t minimum_sample_count(uint32_t expected)
{
    const uint64_t numerator =
        (uint64_t)expected *
        (UINT64_C(100) - IMU_CALIBRATION_SAMPLE_TOLERANCE_PERCENT);

    return (uint32_t)((numerator + UINT64_C(99)) / UINT64_C(100));
}

static uint16_t actual_rate_hz(uint32_t sample_count, uint64_t duration_us);

static void set_gyro_bias_quality(void)
{
    const uint16_t rate_hz = s_calibration.bmi_configured_rate_hz;
    const uint64_t duration_us = rate_hz == 0U
                                     ? 0U
                                     : ((uint64_t)IMU_CAL_GYRO_BIAS_SAMPLE_COUNT *
                                        UINT64_C(1000000)) /
                                           rate_hz;
    const uint32_t actual = s_calibration.bmi_gyro.sample_count;

    s_calibration.quality.bmi_gyro = (imu_sample_quality_t){
        .configured_rate_hz = rate_hz,
        .actual_rate_hz = actual_rate_hz(actual, duration_us),
        .expected_sample_count = IMU_CAL_GYRO_BIAS_SAMPLE_COUNT,
        .minimum_sample_count = IMU_CAL_GYRO_BIAS_SAMPLE_COUNT,
        .actual_sample_count = actual,
        .quality_ok = actual == IMU_CAL_GYRO_BIAS_SAMPLE_COUNT ? 1U : 0U,
    };
}

static uint16_t actual_rate_hz(uint32_t sample_count, uint64_t duration_us)
{
    const uint64_t rate = duration_us == 0U
                              ? 0U
                              : (((uint64_t)sample_count * UINT64_C(1000000)) +
                                 (duration_us / UINT64_C(2))) /
                                    duration_us;

    return rate > UINT16_MAX ? UINT16_MAX : (uint16_t)rate;
}

static void set_quality(imu_sample_quality_t *quality,
                        uint16_t configured_rate_hz,
                        uint32_t actual_sample_count)
{
    const uint32_t expected = expected_sample_count(
        configured_rate_hz, IMU_CALIBRATION_WINDOW_US);
    const uint32_t minimum = minimum_sample_count(expected);

    if (quality == NULL) {
        return;
    }
    *quality = (imu_sample_quality_t){
        .configured_rate_hz = configured_rate_hz,
        .actual_rate_hz = actual_rate_hz(actual_sample_count,
                                         IMU_CALIBRATION_WINDOW_US),
        .expected_sample_count = expected,
        .minimum_sample_count = minimum,
        .actual_sample_count = actual_sample_count,
        .quality_ok = (expected != 0U && actual_sample_count >= minimum) ? 1U : 0U,
    };
}

static void store_quality(void)
{
    set_quality(&s_calibration.quality.lsm_accel,
                (uint16_t)IMU_CALIBRATION_SAMPLE_RATE_HZ,
                s_calibration.lsm_accel.sample_count);
    set_quality(&s_calibration.quality.bmi_accel,
                s_calibration.bmi_configured_rate_hz,
                s_calibration.bmi_accel.sample_count);
    set_gyro_bias_quality();
}

static uint8_t window_accepts_sample(uint64_t timestamp_us)
{
    return s_calibration.window_active != 0U &&
                   (timestamp_us - s_calibration.window_start_timestamp_us) <
                       IMU_CALIBRATION_WINDOW_US
               ? 1U
               : 0U;
}

static uint8_t all_streams_meet_quality(void)
{
    return s_calibration.quality.lsm_accel.quality_ok != 0U &&
                   s_calibration.quality.bmi_accel.quality_ok != 0U &&
                   s_calibration.quality.bmi_gyro.quality_ok != 0U
               ? 1U
               : 0U;
}

static void store_sample_counts(void)
{
    s_calibration.result.sample_counts.lsm_accel =
        s_calibration.lsm_accel.sample_count;
    s_calibration.result.sample_counts.bmi_accel =
        s_calibration.bmi_accel.sample_count;
    s_calibration.result.sample_counts.bmi_gyro =
        s_calibration.bmi_gyro.sample_count;
}

static float quality_ratio(const imu_sample_quality_t *quality)
{
    if (quality == NULL || quality->expected_sample_count == 0U) {
        return 0.0f;
    }
    return (float)quality->actual_sample_count /
           (float)quality->expected_sample_count;
}

static void store_accel_statistics(imu_calibration_static_sensor_t *statistics,
                                   const imu_axis_accumulator_t *accumulator,
                                   float valid_ratio)
{
    double variance_x;
    double variance_y;
    double variance_z;
    const double count = accumulator == NULL ? 0.0 :
                         (double)accumulator->sample_count;

    if (statistics == NULL) {
        return;
    }
    *statistics = (imu_calibration_static_sensor_t){
        .valid_ratio = valid_ratio,
    };
    if (accumulator == NULL || accumulator->sample_count == 0U) {
        return;
    }

    statistics->accel_mean[0] = (float)(accumulator->sum_x / count);
    statistics->accel_mean[1] = (float)(accumulator->sum_y / count);
    statistics->accel_mean[2] = (float)(accumulator->sum_z / count);
    variance_x = (accumulator->sum_square_x / count) -
                 ((double)statistics->accel_mean[0] *
                  (double)statistics->accel_mean[0]);
    variance_y = (accumulator->sum_square_y / count) -
                 ((double)statistics->accel_mean[1] *
                  (double)statistics->accel_mean[1]);
    variance_z = (accumulator->sum_square_z / count) -
                 ((double)statistics->accel_mean[2] *
                  (double)statistics->accel_mean[2]);
    variance_x = variance_x > 0.0 ? variance_x : 0.0;
    variance_y = variance_y > 0.0 ? variance_y : 0.0;
    variance_z = variance_z > 0.0 ? variance_z : 0.0;
    statistics->accel_std_mps2 =
        sqrtf((float)((variance_x + variance_y + variance_z) / 3.0));
}

static float gyro_rms(const imu_axis_accumulator_t *accumulator)
{
    const double count = accumulator == NULL ? 0.0 :
                         (double)accumulator->sample_count;
    double variance_x;
    double variance_y;
    double variance_z;

    if (accumulator == NULL || accumulator->sample_count == 0U) {
        return 0.0f;
    }
    variance_x = (accumulator->sum_square_x / count) -
                 ((accumulator->sum_x / count) *
                  (accumulator->sum_x / count));
    variance_y = (accumulator->sum_square_y / count) -
                 ((accumulator->sum_y / count) *
                  (accumulator->sum_y / count));
    variance_z = (accumulator->sum_square_z / count) -
                 ((accumulator->sum_z / count) *
                  (accumulator->sum_z / count));
    variance_x = variance_x > 0.0 ? variance_x : 0.0;
    variance_y = variance_y > 0.0 ? variance_y : 0.0;
    variance_z = variance_z > 0.0 ? variance_z : 0.0;
    return sqrtf((float)((variance_x + variance_y + variance_z) / 3.0));
}

static void store_static_statistics(void)
{
    const float lsm_ratio = quality_ratio(&s_calibration.quality.lsm_accel);
    const float bmi_accel_ratio =
        quality_ratio(&s_calibration.quality.bmi_accel);
    const float bmi_gyro_ratio = quality_ratio(&s_calibration.quality.bmi_gyro);
    const float bmi_ratio = bmi_accel_ratio < bmi_gyro_ratio
                                ? bmi_accel_ratio
                                : bmi_gyro_ratio;

    store_accel_statistics(&s_calibration.static_statistics.lsm,
                           &s_calibration.lsm_accel, lsm_ratio);
    store_accel_statistics(&s_calibration.static_statistics.bmi,
                           &s_calibration.bmi_accel, bmi_ratio);
    s_calibration.static_statistics.bmi.gyro_rms_radps =
        gyro_rms(&s_calibration.bmi_gyro);

    if (s_calibration.static_statistics.lsm.accel_std_mps2 >
            LSM_ACCEL_STD_MAX ||
        s_calibration.static_statistics.bmi.accel_std_mps2 >
            BMI_ACCEL_STD_MAX ||
        s_calibration.static_statistics.bmi.gyro_rms_radps >
            IMU_CALIBRATION_GYRO_MOTION_THRESHOLD_RADPS) {
        s_calibration.static_motion_detected = 1U;
    }
}

void imu_calibration_init(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
    }
#endif
    s_bmi_capture_active = 0U;
    lock_calibration();
    (void)memset(&s_calibration, 0, sizeof(s_calibration));
    unlock_calibration();
}

void imu_calibration_start(void)
{
    s_bmi_capture_active = 0U;
    lock_calibration();
    (void)memset(&s_calibration, 0, sizeof(s_calibration));
    unlock_calibration();
}

void imu_calibration_begin_window(uint64_t start_timestamp_us,
                                  uint16_t bmi_configured_rate_hz)
{
    lock_calibration();
    s_calibration.window_start_timestamp_us = start_timestamp_us;
    s_calibration.bmi_configured_rate_hz = bmi_configured_rate_hz;
    s_calibration.window_active = 1U;
    s_calibration.complete = 0U;
    s_bmi_capture_active = 1U;
    unlock_calibration();
}

uint8_t imu_calibration_bmi_capture_active(void)
{
    return s_bmi_capture_active;
}

uint8_t imu_calibration_window_expired(uint64_t now_timestamp_us)
{
    uint8_t expired;

    lock_calibration();
    expired = s_calibration.window_active != 0U &&
                      (now_timestamp_us -
                       s_calibration.window_start_timestamp_us) >=
                          IMU_CALIBRATION_WINDOW_US
                  ? 1U
                  : 0U;
    unlock_calibration();
    return expired;
}

uint8_t imu_calibration_static_motion_detected(void)
{
    uint8_t detected;

    lock_calibration();
    detected = s_calibration.static_motion_detected;
    unlock_calibration();
    return detected;
}

uint8_t imu_calibration_finish_window(uint64_t now_timestamp_us)
{
    uint8_t complete;

    lock_calibration();
    if (s_calibration.window_active != 0U &&
        (now_timestamp_us - s_calibration.window_start_timestamp_us) >=
            IMU_CALIBRATION_WINDOW_US) {
        s_calibration.window_active = 0U;
        s_bmi_capture_active = 0U;
        store_sample_counts();
        store_quality();
        store_static_statistics();
        if (all_streams_meet_quality() != 0U) {
            /* A one-pose static window cannot identify sensor bias separately
             * from mechanical incline. R_level owns the gravity direction;
             * leaving these at zero prevents gravity from being subtracted
             * once as a pseudo-bias and again by the rotation. */
            s_calibration.result.lsm_accel_bias = (imu_bias_xyz_t){0};
            s_calibration.result.bmi_accel_bias = (imu_bias_xyz_t){0};
            s_calibration.result.bmi_gyro_bias =
                mean_xyz(&s_calibration.bmi_gyro);
            s_calibration.bias = (imu_calibration_bias_t){0};
            s_calibration.complete = 1U;
        }
    }
    complete = s_calibration.complete;
    unlock_calibration();
    return complete;
}

void imu_calibration_update(const imu_raw_data_t *raw_data)
{
    if (raw_data == NULL) {
        return;
    }

    lock_calibration();
    const uint8_t lsm_accel_valid = raw_data->lsm_accel_valid != 0U ||
                                     raw_data->online != 0U;
    const float lsm_ax = raw_data->lsm_accel_valid != 0U ? raw_data->lsm_ax : raw_data->ax;
    const float lsm_ay = raw_data->lsm_accel_valid != 0U ? raw_data->lsm_ay : raw_data->ay;
    const float lsm_az = raw_data->lsm_accel_valid != 0U ? raw_data->lsm_az : raw_data->az;
    const uint64_t lsm_timestamp_us = raw_data->lsm_accel_valid != 0U
                                          ? raw_data->lsm_timestamp_us
                                          : raw_data->timestamp_us;

    if (s_calibration.window_active != 0U && lsm_accel_valid != 0U) {
        const float accel_norm = sqrtf((lsm_ax * lsm_ax) +
                                       (lsm_ay * lsm_ay) +
                                       (lsm_az * lsm_az));
        if (isfinite(accel_norm) &&
            fabsf(accel_norm - IMU_CALIBRATION_GRAVITY_MPS2) >
                IMU_CALIBRATION_ACCEL_MOTION_DELTA_MPS2) {
            s_calibration.static_motion_detected = 1U;
        }
    }

    if (lsm_accel_valid != 0U &&
        window_accepts_sample(lsm_timestamp_us) != 0U) {
        if (accumulate_xyz(&s_calibration.lsm_accel, lsm_ax, lsm_ay, lsm_az) !=
            0U) {
            s_calibration.lsm_observed = 1U;
        }
    }
    unlock_calibration();
}

void imu_calibration_update_bmi323(float accel_x, float accel_y, float accel_z,
                                   float gyro_x, float gyro_y, float gyro_z,
                                   uint64_t timestamp_us)
{
    float accel_mps2[3] = {accel_x, accel_y, accel_z};

    /* The ODR task must never wait behind the 100 Hz manager. If another
     * reader owns the accumulator briefly, the reported actual count reflects
     * the skipped observation. */
    if (s_bmi_capture_active == 0U || try_lock_calibration() == 0U) {
        return;
    }
    bmi323_accel_input_to_mps2(accel_mps2);
    if (s_calibration.window_active != 0U) {
        const float accel_norm = sqrtf((accel_mps2[0] * accel_mps2[0]) +
                                       (accel_mps2[1] * accel_mps2[1]) +
                                       (accel_mps2[2] * accel_mps2[2]));
        const float gyro_norm = sqrtf((gyro_x * gyro_x) + (gyro_y * gyro_y) +
                                      (gyro_z * gyro_z));
        if ((isfinite(accel_norm) &&
             fabsf(accel_norm - IMU_CALIBRATION_GRAVITY_MPS2) >
                 IMU_CALIBRATION_ACCEL_MOTION_DELTA_MPS2) ||
            (isfinite(gyro_norm) &&
             gyro_norm > IMU_CALIBRATION_GYRO_MOTION_THRESHOLD_RADPS)) {
            s_calibration.static_motion_detected = 1U;
        }
    }
    if (window_accepts_sample(timestamp_us) != 0U) {
        if (accumulate_xyz(&s_calibration.bmi_accel,
                           accel_mps2[0], accel_mps2[1],
                           accel_mps2[2]) != 0U) {
            s_calibration.bmi_accel_observed = 1U;
        }
        if (s_calibration.bmi_gyro.sample_count <
                IMU_CAL_GYRO_BIAS_SAMPLE_COUNT &&
            accumulate_xyz(&s_calibration.bmi_gyro,
                           gyro_x, gyro_y, gyro_z) != 0U) {
            s_calibration.bmi_gyro_observed = 1U;
        }
    }
    unlock_calibration();
}

uint8_t imu_calibration_is_complete(void)
{
    uint8_t complete;
    lock_calibration();
    complete = s_calibration.complete;
    unlock_calibration();
    return complete;
}

uint32_t imu_calibration_get_sample_count(void)
{
    uint32_t count;
    lock_calibration();
    count = s_calibration.lsm_accel.sample_count;
    if (s_calibration.bmi_accel.sample_count < count) {
        count = s_calibration.bmi_accel.sample_count;
    }
    if (s_calibration.bmi_gyro.sample_count < count) {
        count = s_calibration.bmi_gyro.sample_count;
    }
    unlock_calibration();
    return count;
}

uint32_t imu_calibration_get_sample_total(void)
{
    return IMU_CALIBRATION_LSM303_NOMINAL_SAMPLES;
}

uint8_t imu_calibration_get_progress(void)
{
    uint8_t progress;

    lock_calibration();
    if (s_calibration.complete != 0U) {
        progress = 100U;
    } else if (s_calibration.window_active != 0U) {
        const uint64_t elapsed_us = imu_time_now_us() -
                                    s_calibration.window_start_timestamp_us;
        const uint64_t percent =
            (elapsed_us * UINT64_C(100)) / IMU_CALIBRATION_WINDOW_US;
        progress = (uint8_t)(percent > 100U ? 100U : percent);
    } else {
        progress = 0U;
    }
    unlock_calibration();
    return progress;
}

imu_calibration_bias_t imu_calibration_get_bias(void)
{
    imu_calibration_bias_t bias;
    lock_calibration();
    bias = s_calibration.bias;
    unlock_calibration();
    return bias;
}

imu_calibration_result_t imu_calibration_get_result(void)
{
    imu_calibration_result_t result;
    lock_calibration();
    result = s_calibration.result;
    unlock_calibration();
    return result;
}

imu_calibration_sample_counts_t imu_calibration_get_sample_counts(void)
{
    imu_calibration_sample_counts_t counts;

    lock_calibration();
    counts.lsm_accel = s_calibration.lsm_accel.sample_count;
    counts.bmi_accel = s_calibration.bmi_accel.sample_count;
    counts.bmi_gyro = s_calibration.bmi_gyro.sample_count;
    unlock_calibration();
    return counts;
}

imu_calibration_quality_t imu_calibration_get_quality(void)
{
    imu_calibration_quality_t quality;

    lock_calibration();
    quality = s_calibration.quality;
    unlock_calibration();
    return quality;
}

imu_calibration_static_statistics_t imu_calibration_get_static_statistics(void)
{
    imu_calibration_static_statistics_t statistics;

    lock_calibration();
    statistics = s_calibration.static_statistics;
    unlock_calibration();
    return statistics;
}

uint8_t imu_calibration_is_lsm_complete(void)
{
    uint8_t complete;
    lock_calibration();
    complete = (s_calibration.complete != 0U &&
                s_calibration.quality.lsm_accel.quality_ok != 0U) ? 1U : 0U;
    unlock_calibration();
    return complete;
}

uint8_t imu_calibration_is_bmi_complete(void)
{
    uint8_t complete;
    lock_calibration();
    complete = (s_calibration.complete != 0U &&
                s_calibration.quality.bmi_accel.quality_ok != 0U &&
                s_calibration.quality.bmi_gyro.quality_ok != 0U) ? 1U : 0U;
    unlock_calibration();
    return complete;
}

imu_calibrated_data_t imu_calibration_apply(const imu_raw_data_t *raw_data)
{
    imu_calibrated_data_t calibrated = {0};
    imu_calibration_bias_t bias;

    if (raw_data == NULL) {
        return calibrated;
    }
    bias = imu_calibration_get_bias();
    calibrated = *raw_data;
    calibrated.ax -= bias.ax;
    calibrated.ay -= bias.ay;
    calibrated.az -= bias.az;
    lock_calibration();
    calibrated.lsm_ax = calibrated.ax - s_calibration.result.lsm_accel_bias.x + bias.ax;
    calibrated.lsm_ay = calibrated.ay - s_calibration.result.lsm_accel_bias.y + bias.ay;
    calibrated.lsm_az = calibrated.az - s_calibration.result.lsm_accel_bias.z + bias.az;
    calibrated.bmi_ax -= s_calibration.result.bmi_accel_bias.x;
    calibrated.bmi_ay -= s_calibration.result.bmi_accel_bias.y;
    calibrated.bmi_az -= s_calibration.result.bmi_accel_bias.z;
    calibrated.bmi_gx -= s_calibration.result.bmi_gyro_bias.x;
    calibrated.bmi_gy -= s_calibration.result.bmi_gyro_bias.y;
    calibrated.bmi_gz -= s_calibration.result.bmi_gyro_bias.z;
    s_calibration.last_calibrated = calibrated;
    unlock_calibration();
    return calibrated;
}

imu_calibrated_data_t imu_calibration_get_data(void)
{
    imu_calibrated_data_t calibrated;
    lock_calibration();
    calibrated = s_calibration.last_calibrated;
    unlock_calibration();
    return calibrated;
}
