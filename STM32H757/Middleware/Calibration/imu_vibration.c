#include "imu_vibration.h"

#include <math.h>
#include <string.h>

#include "imu_time.h"

#if defined(IMU_MANAGER_USE_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

static const uint8_t s_pwm_levels[IMU_VIBRATION_PROFILE_COUNT] =
    {20U, 40U, 60U, 80U, 100U};

typedef struct
{
    double sum[3];
    double sum_square[3];
    uint32_t sample_count;
    uint64_t last_timestamp_us;
} vibration_accumulator_t;

typedef struct
{
    imu_vibration_window_t window;
    imu_vibration_dataset_t lsm_dataset;
    imu_vibration_dataset_t bmi_dataset;
    imu_calibration_result_t calibration;
    imu_vibration_dataset_sink_t lsm_sink;
    imu_vibration_dataset_sink_t bmi_sink;
    void *lsm_sink_context;
    void *bmi_sink_context;
    vibration_accumulator_t lsm_accel;
    vibration_accumulator_t bmi_accel;
    vibration_accumulator_t bmi_gyro;
    lsm_vibration_profile_t lsm_profiles[IMU_VIBRATION_PROFILE_COUNT];
    bmi_vibration_profile_t bmi_profiles[IMU_VIBRATION_PROFILE_COUNT];
    lsm_vibration_profile_t lsm_current;
    bmi_vibration_profile_t bmi_current;
    uint8_t profile_index;
    uint32_t lsm_sequence;
    uint32_t bmi_sequence;
} imu_vibration_state_t;

static imu_vibration_state_t s_vibration;

#if defined(IMU_MANAGER_USE_FREERTOS)
static SemaphoreHandle_t s_mutex;
#endif

static void lock_vibration(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex != NULL) {
        (void)xSemaphoreTake(s_mutex, portMAX_DELAY);
    }
#endif
}

static void unlock_vibration(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex != NULL) {
        (void)xSemaphoreGive(s_mutex);
    }
#endif
}

static uint8_t finite_xyz(float x, float y, float z)
{
    return isfinite(x) && isfinite(y) && isfinite(z) ? 1U : 0U;
}

static uint8_t window_accepts_timestamp(uint64_t timestamp_us)
{
    return s_vibration.window.active != 0U &&
                   timestamp_us >=
                       s_vibration.window.common_start_timestamp_us &&
                   timestamp_us <
                       s_vibration.window.common_end_timestamp_us
               ? 1U
               : 0U;
}

static uint8_t accumulate(vibration_accumulator_t *accumulator,
                          float x, float y, float z, uint64_t timestamp_us)
{
    const double values[3] = {(double)x, (double)y, (double)z};

    if (accumulator == NULL || accumulator->sample_count == UINT32_MAX ||
        finite_xyz(x, y, z) == 0U) {
        return 0U;
    }
    for (uint8_t axis = 0U; axis < 3U; ++axis) {
        const double next_sum = accumulator->sum[axis] + values[axis];
        const double next_square = accumulator->sum_square[axis] +
                                   (values[axis] * values[axis]);

        if (!isfinite(next_sum) || !isfinite(next_square)) {
            return 0U;
        }
    }
    for (uint8_t axis = 0U; axis < 3U; ++axis) {
        accumulator->sum[axis] += values[axis];
        accumulator->sum_square[axis] += values[axis] * values[axis];
    }
    ++accumulator->sample_count;
    accumulator->last_timestamp_us = timestamp_us;
    return 1U;
}

static void calculate_rms(const vibration_accumulator_t *accumulator,
                          float rms[3])
{
    if (accumulator == NULL || rms == NULL ||
        accumulator->sample_count == 0U) {
        return;
    }
    for (uint8_t axis = 0U; axis < 3U; ++axis) {
        const double count = (double)accumulator->sample_count;
        const double mean = accumulator->sum[axis] / count;
        const double variance = (accumulator->sum_square[axis] / count) -
                                (mean * mean);
        rms[axis] = sqrtf((float)(variance > 0.0 ? variance : 0.0));
    }
}

static float total_rms(const float rms[3])
{
    return sqrtf((rms[0] * rms[0] + rms[1] * rms[1] + rms[2] * rms[2]) /
                 3.0f);
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
    /* The standard 10 s LSM303 window at 100 Hz therefore requires
     * 1000 expected samples and at least 900 valid samples. */
    const uint64_t numerator =
        (uint64_t)expected * IMU_VIBRATION_SAMPLE_QUALITY_FLOOR_PERCENT;

    return (uint32_t)((numerator + UINT64_C(99)) / UINT64_C(100));
}

static uint16_t actual_rate_hz(uint32_t valid_count, uint64_t duration_us)
{
    const uint64_t rate = duration_us == 0U
                              ? 0U
                              : (((uint64_t)valid_count * UINT64_C(1000000)) +
                                 (duration_us / UINT64_C(2))) /
                                    duration_us;

    return rate > UINT16_MAX ? UINT16_MAX : (uint16_t)rate;
}

static uint8_t sample_rate_supported(uint16_t sample_rate)
{
    return sample_rate == UINT16_C(100) || sample_rate == UINT16_C(200) ||
                   sample_rate == UINT16_C(400) || sample_rate == UINT16_C(800)
               ? 1U
               : 0U;
}

static void finalize_dataset_quality(imu_vibration_dataset_t *dataset)
{
    uint32_t expected;
    uint32_t minimum;

    if (dataset == NULL) {
        return;
    }
    expected = expected_sample_count(dataset->configured_rate_hz,
                                     s_vibration.window.duration_us);
    minimum = minimum_sample_count(expected);
    dataset->actual_rate_hz = actual_rate_hz(dataset->valid_count,
                                              s_vibration.window.duration_us);
    dataset->quality_ok = (expected != 0U &&
                           dataset->valid_count >= minimum) ? 1U : 0U;
    dataset->complete = dataset->quality_ok;
}

static void initialize_dataset(imu_vibration_dataset_t *dataset,
                               uint8_t sensor_id,
                               uint16_t configured_rate_hz,
                               uint8_t radar_pwm)
{
    if (dataset == NULL) {
        return;
    }
    *dataset = (imu_vibration_dataset_t){
        .common_start_timestamp_us =
            s_vibration.window.common_start_timestamp_us,
        .common_end_timestamp_us =
            s_vibration.window.common_end_timestamp_us,
        .window_sequence = s_vibration.window.window_sequence,
        .sample_rate = configured_rate_hz,
        .configured_rate_hz = configured_rate_hz,
        .sensor_id = sensor_id,
        .motion_label = IMU_MOTION_LABEL_VIBRATION,
        .radar_pwm = radar_pwm,
        .active = 1U
    };
}

static void finalize_window_locked(void)
{
    float lsm_rms[3] = {0};
    float bmi_accel_rms[3] = {0};
    float bmi_gyro_rms[3] = {0};

    if (s_vibration.window.active == 0U) {
        return;
    }
    s_vibration.window.active = 0U;
    s_vibration.window.complete = 1U;
    s_vibration.lsm_dataset.active = 0U;
    s_vibration.bmi_dataset.active = 0U;
    /* Window closure is time-based. Per-sensor completion additionally
     * requires valid samples at or above the configured 90% floor. */
    finalize_dataset_quality(&s_vibration.lsm_dataset);
    finalize_dataset_quality(&s_vibration.bmi_dataset);

    calculate_rms(&s_vibration.lsm_accel, lsm_rms);
    calculate_rms(&s_vibration.bmi_accel, bmi_accel_rms);
    calculate_rms(&s_vibration.bmi_gyro, bmi_gyro_rms);

    s_vibration.lsm_current.sample_count = s_vibration.lsm_accel.sample_count;
    s_vibration.lsm_current.timestamp = (uint32_t)(
        s_vibration.window.common_end_timestamp_us / UINT64_C(1000));
    s_vibration.lsm_current.common_start_timestamp =
        s_vibration.window.common_start_timestamp_us;
    s_vibration.lsm_current.common_end_timestamp =
        s_vibration.window.common_end_timestamp_us;
    s_vibration.lsm_current.sample_rate = IMU_VIBRATION_LSM_SAMPLE_RATE_HZ;
    s_vibration.lsm_current.invalid_sample_count =
        s_vibration.lsm_dataset.invalid_count;
    s_vibration.lsm_current.captured =
        s_vibration.lsm_dataset.captured_count;
    s_vibration.lsm_current.invalid = s_vibration.lsm_dataset.invalid_count;
    s_vibration.lsm_current.configured_rate_hz =
        s_vibration.lsm_dataset.configured_rate_hz;
    s_vibration.lsm_current.actual_rate_hz =
        s_vibration.lsm_dataset.actual_rate_hz;
    s_vibration.lsm_current.quality_ok = s_vibration.lsm_dataset.quality_ok;
    s_vibration.lsm_current.rms_x = lsm_rms[0];
    s_vibration.lsm_current.rms_y = lsm_rms[1];
    s_vibration.lsm_current.rms_z = lsm_rms[2];
    s_vibration.lsm_current.total_rms = total_rms(lsm_rms);

    s_vibration.bmi_current.sample_count = s_vibration.bmi_dataset.valid_count;
    s_vibration.bmi_current.timestamp = (uint32_t)(
        s_vibration.window.common_end_timestamp_us / UINT64_C(1000));
    s_vibration.bmi_current.common_start_timestamp =
        s_vibration.window.common_start_timestamp_us;
    s_vibration.bmi_current.common_end_timestamp =
        s_vibration.window.common_end_timestamp_us;
    s_vibration.bmi_current.sample_rate = s_vibration.bmi_dataset.sample_rate;
    s_vibration.bmi_current.invalid_sample_count =
        s_vibration.bmi_dataset.invalid_count;
    s_vibration.bmi_current.captured =
        s_vibration.bmi_dataset.captured_count;
    s_vibration.bmi_current.invalid = s_vibration.bmi_dataset.invalid_count;
    s_vibration.bmi_current.configured_rate_hz =
        s_vibration.bmi_dataset.configured_rate_hz;
    s_vibration.bmi_current.actual_rate_hz =
        s_vibration.bmi_dataset.actual_rate_hz;
    s_vibration.bmi_current.quality_ok = s_vibration.bmi_dataset.quality_ok;
    s_vibration.bmi_current.accel_rms_x = bmi_accel_rms[0];
    s_vibration.bmi_current.accel_rms_y = bmi_accel_rms[1];
    s_vibration.bmi_current.accel_rms_z = bmi_accel_rms[2];
    s_vibration.bmi_current.accel_total_rms = total_rms(bmi_accel_rms);
    s_vibration.bmi_current.gyro_rms_x = bmi_gyro_rms[0];
    s_vibration.bmi_current.gyro_rms_y = bmi_gyro_rms[1];
    s_vibration.bmi_current.gyro_rms_z = bmi_gyro_rms[2];
    s_vibration.bmi_current.gyro_total_rms = total_rms(bmi_gyro_rms);

    if (s_vibration.profile_index < IMU_VIBRATION_PROFILE_COUNT) {
        s_vibration.lsm_profiles[s_vibration.profile_index] =
            s_vibration.lsm_current;
        s_vibration.bmi_profiles[s_vibration.profile_index] =
            s_vibration.bmi_current;
    }
}

static void record_delivery(uint8_t sensor_id, uint32_t window_sequence,
                            imu_vibration_dataset_sink_t sink,
                            void *context,
                            const imu_vibration_sample_t *sample)
{
    imu_vibration_dataset_t *dataset;
    uint8_t accepted;

    if (sink == NULL || sample == NULL) {
        return;
    }
    accepted = sink(sample, context);
    lock_vibration();
    dataset = sensor_id == IMU_VIBRATION_SENSOR_LSM303
                  ? &s_vibration.lsm_dataset
                  : &s_vibration.bmi_dataset;
    if (dataset->window_sequence != window_sequence) {
        unlock_vibration();
        return;
    }
    if (accepted != 0U) {
        if (dataset->delivered_count != UINT32_MAX) {
            ++dataset->delivered_count;
        }
    } else if (dataset->delivery_failure_count != UINT32_MAX) {
        ++dataset->delivery_failure_count;
    }
    unlock_vibration();
}

void imu_vibration_init(void)
{
    imu_vibration_dataset_sink_t lsm_sink;
    imu_vibration_dataset_sink_t bmi_sink;
    void *lsm_sink_context;
    void *bmi_sink_context;

#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
    }
#endif
    lock_vibration();
    lsm_sink = s_vibration.lsm_sink;
    bmi_sink = s_vibration.bmi_sink;
    lsm_sink_context = s_vibration.lsm_sink_context;
    bmi_sink_context = s_vibration.bmi_sink_context;
    (void)memset(&s_vibration, 0, sizeof(s_vibration));
    s_vibration.lsm_sink = lsm_sink;
    s_vibration.bmi_sink = bmi_sink;
    s_vibration.lsm_sink_context = lsm_sink_context;
    s_vibration.bmi_sink_context = bmi_sink_context;
    unlock_vibration();
}

void imu_vibration_start(uint8_t radar_pwm)
{
    (void)imu_vibration_start_window(
        radar_pwm, imu_time_now_us(), IMU_VIBRATION_WINDOW_DURATION_US,
        IMU_VIBRATION_BMI_DEFAULT_SAMPLE_RATE_HZ);
}

uint8_t imu_vibration_start_window(uint8_t radar_pwm,
                                   uint64_t common_start_timestamp_us,
                                   uint64_t duration_us,
                                   uint16_t bmi_configured_rate_hz)
{
    const imu_calibration_result_t calibration = imu_calibration_get_result();

    if (duration_us == 0U ||
        common_start_timestamp_us > UINT64_MAX - duration_us ||
        sample_rate_supported(bmi_configured_rate_hz) == 0U) {
        return 0U;
    }
    lock_vibration();
    if (s_vibration.window.active != 0U) {
        unlock_vibration();
        return 0U;
    }
    s_vibration.window.common_start_timestamp_us = common_start_timestamp_us;
    s_vibration.window.common_end_timestamp_us =
        common_start_timestamp_us + duration_us;
    s_vibration.window.duration_us = duration_us;
    s_vibration.window.window_sequence += 1U;
    s_vibration.window.active = 1U;
    s_vibration.window.complete = 0U;
    s_vibration.calibration = calibration;
    s_vibration.lsm_accel = (vibration_accumulator_t){0};
    s_vibration.bmi_accel = (vibration_accumulator_t){0};
    s_vibration.bmi_gyro = (vibration_accumulator_t){0};
    s_vibration.lsm_current = (lsm_vibration_profile_t){
        .radar_pwm = radar_pwm,
        .common_start_timestamp = common_start_timestamp_us,
        .common_end_timestamp = common_start_timestamp_us + duration_us,
        .sample_rate = IMU_VIBRATION_LSM_SAMPLE_RATE_HZ,
        .configured_rate_hz = IMU_VIBRATION_LSM_SAMPLE_RATE_HZ,
    };
    s_vibration.bmi_current = (bmi_vibration_profile_t){
        .radar_pwm = radar_pwm,
        .common_start_timestamp = common_start_timestamp_us,
        .common_end_timestamp = common_start_timestamp_us + duration_us,
        .sample_rate = bmi_configured_rate_hz,
        .configured_rate_hz = bmi_configured_rate_hz,
    };
    s_vibration.lsm_sequence = 0U;
    s_vibration.bmi_sequence = 0U;
    initialize_dataset(&s_vibration.lsm_dataset, IMU_VIBRATION_SENSOR_LSM303,
                       IMU_VIBRATION_LSM_SAMPLE_RATE_HZ, radar_pwm);
    initialize_dataset(&s_vibration.bmi_dataset, IMU_VIBRATION_SENSOR_BMI323,
                       bmi_configured_rate_hz, radar_pwm);
    unlock_vibration();
    return 1U;
}

void imu_vibration_set_lsm_dataset_sink(imu_vibration_dataset_sink_t sink,
                                        void *context)
{
    lock_vibration();
    s_vibration.lsm_sink = sink;
    s_vibration.lsm_sink_context = context;
    unlock_vibration();
}

void imu_vibration_set_bmi_dataset_sink(imu_vibration_dataset_sink_t sink,
                                        void *context)
{
    lock_vibration();
    s_vibration.bmi_sink = sink;
    s_vibration.bmi_sink_context = context;
    unlock_vibration();
}

void imu_vibration_select_profile(uint8_t index)
{
    if (index >= IMU_VIBRATION_PROFILE_COUNT) {
        return;
    }
    lock_vibration();
    s_vibration.profile_index = index;
    unlock_vibration();
}

uint8_t imu_vibration_get_pwm_level(uint8_t index)
{
    return index < IMU_VIBRATION_PROFILE_COUNT ? s_pwm_levels[index] : 0U;
}

void imu_vibration_capture_lsm(float ax, float ay, float az,
                               uint64_t timestamp_us, uint8_t valid)
{
    imu_vibration_sample_t sample = {0};
    imu_vibration_dataset_sink_t sink = NULL;
    void *context = NULL;
    uint32_t window_sequence;
    uint8_t sample_valid;

    lock_vibration();
    if (window_accepts_timestamp(timestamp_us) == 0U) {
        unlock_vibration();
        return;
    }
    if (s_vibration.lsm_dataset.captured_count != UINT32_MAX) {
        ++s_vibration.lsm_dataset.captured_count;
    }
    sample.timestamp_us = timestamp_us;
    sample.sensor_id = IMU_VIBRATION_SENSOR_LSM303;
    sample.sample_rate = IMU_VIBRATION_LSM_SAMPLE_RATE_HZ;
    sample.sequence = s_vibration.lsm_sequence++;
    sample.motion_label = IMU_MOTION_LABEL_VIBRATION;
    sample.accel[0] = ax - s_vibration.calibration.lsm_accel_bias.x;
    sample.accel[1] = ay - s_vibration.calibration.lsm_accel_bias.y;
    sample.accel[2] = az - s_vibration.calibration.lsm_accel_bias.z;
    sample_valid = valid != 0U &&
                   accumulate(&s_vibration.lsm_accel, sample.accel[0],
                              sample.accel[1], sample.accel[2], timestamp_us) != 0U
                       ? 1U
                       : 0U;
    if (sample_valid == 0U) {
        if (s_vibration.lsm_dataset.invalid_count != UINT32_MAX) {
            ++s_vibration.lsm_dataset.invalid_count;
        }
        sample.accel[0] = 0.0f;
        sample.accel[1] = 0.0f;
        sample.accel[2] = 0.0f;
    } else if (s_vibration.lsm_dataset.valid_count != UINT32_MAX) {
        ++s_vibration.lsm_dataset.valid_count;
    }
    sample.valid = sample_valid;
    sink = s_vibration.lsm_sink;
    context = s_vibration.lsm_sink_context;
    window_sequence = s_vibration.lsm_dataset.window_sequence;
    if (sink == NULL &&
        s_vibration.lsm_dataset.delivery_failure_count != UINT32_MAX) {
        ++s_vibration.lsm_dataset.delivery_failure_count;
    }
    unlock_vibration();
    record_delivery(IMU_VIBRATION_SENSOR_LSM303, window_sequence, sink,
                    context, &sample);
}

void imu_vibration_capture_bmi(float ax, float ay, float az,
                               float gx, float gy, float gz,
                               uint64_t timestamp_us, uint16_t sample_rate,
                               uint8_t valid)
{
    imu_vibration_sample_t sample = {0};
    imu_vibration_dataset_sink_t sink = NULL;
    void *context = NULL;
    uint32_t window_sequence;
    uint8_t sample_valid;

    /* The callback runs after the lock is released. The critical section only
     * snapshots counters and scalar values, so an accepted 800 Hz sample is
     * never silently dropped due to lock contention. */
    lock_vibration();
    if (window_accepts_timestamp(timestamp_us) == 0U) {
        unlock_vibration();
        return;
    }
    if (s_vibration.bmi_dataset.captured_count != UINT32_MAX) {
        ++s_vibration.bmi_dataset.captured_count;
    }
    sample.timestamp_us = timestamp_us;
    sample.sensor_id = IMU_VIBRATION_SENSOR_BMI323;
    sample.sample_rate = sample_rate;
    sample.sequence = s_vibration.bmi_sequence++;
    sample.motion_label = IMU_MOTION_LABEL_VIBRATION;
    sample.accel[0] = ax - s_vibration.calibration.bmi_accel_bias.x;
    sample.accel[1] = ay - s_vibration.calibration.bmi_accel_bias.y;
    sample.accel[2] = az - s_vibration.calibration.bmi_accel_bias.z;
    sample.gyro[0] = gx - s_vibration.calibration.bmi_gyro_bias.x;
    sample.gyro[1] = gy - s_vibration.calibration.bmi_gyro_bias.y;
    sample.gyro[2] = gz - s_vibration.calibration.bmi_gyro_bias.z;
    sample_valid = valid != 0U &&
                   sample_rate == s_vibration.bmi_dataset.configured_rate_hz &&
                   accumulate(&s_vibration.bmi_accel, sample.accel[0],
                              sample.accel[1], sample.accel[2], timestamp_us) != 0U &&
                   accumulate(&s_vibration.bmi_gyro, sample.gyro[0],
                              sample.gyro[1], sample.gyro[2], timestamp_us) != 0U
                       ? 1U
                       : 0U;
    if (sample_valid == 0U) {
        if (s_vibration.bmi_dataset.invalid_count != UINT32_MAX) {
            ++s_vibration.bmi_dataset.invalid_count;
        }
        sample.accel[0] = 0.0f;
        sample.accel[1] = 0.0f;
        sample.accel[2] = 0.0f;
        sample.gyro[0] = 0.0f;
        sample.gyro[1] = 0.0f;
        sample.gyro[2] = 0.0f;
    } else if (s_vibration.bmi_dataset.valid_count != UINT32_MAX) {
        ++s_vibration.bmi_dataset.valid_count;
    }
    sample.valid = sample_valid;
    sink = s_vibration.bmi_sink;
    context = s_vibration.bmi_sink_context;
    window_sequence = s_vibration.bmi_dataset.window_sequence;
    if (sink == NULL &&
        s_vibration.bmi_dataset.delivery_failure_count != UINT32_MAX) {
        ++s_vibration.bmi_dataset.delivery_failure_count;
    }
    unlock_vibration();
    record_delivery(IMU_VIBRATION_SENSOR_BMI323, window_sequence, sink,
                    context, &sample);
}

void imu_vibration_poll(uint64_t timestamp_us)
{
    lock_vibration();
    if (s_vibration.window.active != 0U &&
        timestamp_us >= s_vibration.window.common_end_timestamp_us) {
        finalize_window_locked();
    }
    unlock_vibration();
}

void imu_vibration_update(const imu_calibrated_data_t *sample)
{
    const imu_calibration_result_t calibration = imu_calibration_get_result();
    const uint8_t lsm_valid = sample != NULL &&
                              (sample->lsm_accel_valid != 0U ||
                               sample->online != 0U);

    if (lsm_valid != 0U) {
        const float ax = sample->lsm_accel_valid != 0U ? sample->lsm_ax : sample->ax;
        const float ay = sample->lsm_accel_valid != 0U ? sample->lsm_ay : sample->ay;
        const float az = sample->lsm_accel_valid != 0U ? sample->lsm_az : sample->az;
        const uint64_t timestamp_us = sample->lsm_accel_valid != 0U
                                          ? sample->lsm_timestamp_us
                                          : sample->timestamp_us;
        imu_vibration_capture_lsm(ax + calibration.lsm_accel_bias.x,
                                  ay + calibration.lsm_accel_bias.y,
                                  az + calibration.lsm_accel_bias.z,
                                  timestamp_us, 1U);
    }
#if !defined(IMU_MANAGER_USE_FREERTOS)
    if (sample != NULL && sample->bmi_accel_valid != 0U &&
        sample->bmi_gyro_valid != 0U) {
        imu_vibration_capture_bmi(
                                  sample->bmi_ax +
                                      calibration.bmi_accel_bias.x,
                                  sample->bmi_ay +
                                      calibration.bmi_accel_bias.y,
                                  sample->bmi_az +
                                      calibration.bmi_accel_bias.z,
                                  sample->bmi_gx +
                                      calibration.bmi_gyro_bias.x,
                                  sample->bmi_gy +
                                      calibration.bmi_gyro_bias.y,
                                  sample->bmi_gz +
                                      calibration.bmi_gyro_bias.z,
                                  sample->bmi_timestamp_us,
                                  IMU_VIBRATION_BMI_DEFAULT_SAMPLE_RATE_HZ, 1U);
    }
#endif
}

uint8_t imu_vibration_is_lsm_complete(void)
{
    uint8_t complete;

    lock_vibration();
    complete = s_vibration.lsm_dataset.complete;
    unlock_vibration();
    return complete;
}

uint8_t imu_vibration_is_bmi_complete(void)
{
    uint8_t complete;

    lock_vibration();
    complete = s_vibration.bmi_dataset.complete;
    unlock_vibration();
    return complete;
}

uint8_t imu_vibration_is_complete(void)
{
    return (imu_vibration_is_lsm_complete() != 0U &&
            imu_vibration_is_bmi_complete() != 0U) ? 1U : 0U;
}

uint32_t imu_vibration_get_lsm_sample_count(void)
{
    uint32_t count;

    lock_vibration();
    count = s_vibration.lsm_dataset.valid_count;
    unlock_vibration();
    return count;
}

uint32_t imu_vibration_get_bmi_sample_count(void)
{
    uint32_t count;

    lock_vibration();
    count = s_vibration.bmi_dataset.valid_count;
    unlock_vibration();
    return count;
}

uint32_t imu_vibration_get_sample_count(void)
{
    /* The legacy boot status remains an LSM profile progress indicator. The
     * raw LSM and BMI datasets have separate counters and are never merged. */
    return imu_vibration_get_lsm_sample_count();
}

lsm_vibration_profile_t imu_vibration_get_lsm_result(void)
{
    lsm_vibration_profile_t result;

    lock_vibration();
    result = s_vibration.lsm_current;
    unlock_vibration();
    return result;
}

bmi_vibration_profile_t imu_vibration_get_bmi_result(void)
{
    bmi_vibration_profile_t result;

    lock_vibration();
    result = s_vibration.bmi_current;
    unlock_vibration();
    return result;
}

uint8_t imu_vibration_get_lsm_profile(uint8_t index,
                                      lsm_vibration_profile_t *profile)
{
    if (profile == NULL || index >= IMU_VIBRATION_PROFILE_COUNT) {
        return 0U;
    }
    lock_vibration();
    *profile = s_vibration.lsm_profiles[index];
    unlock_vibration();
    return 1U;
}

uint8_t imu_vibration_get_bmi_profile(uint8_t index,
                                      bmi_vibration_profile_t *profile)
{
    if (profile == NULL || index >= IMU_VIBRATION_PROFILE_COUNT) {
        return 0U;
    }
    lock_vibration();
    *profile = s_vibration.bmi_profiles[index];
    unlock_vibration();
    return 1U;
}

imu_vibration_window_t imu_vibration_get_window(void)
{
    imu_vibration_window_t window;

    lock_vibration();
    window = s_vibration.window;
    unlock_vibration();
    return window;
}

imu_vibration_dataset_t imu_vibration_get_lsm_dataset(void)
{
    imu_vibration_dataset_t dataset;

    lock_vibration();
    dataset = s_vibration.lsm_dataset;
    unlock_vibration();
    return dataset;
}

imu_vibration_dataset_t imu_vibration_get_bmi_dataset(void)
{
    imu_vibration_dataset_t dataset;

    lock_vibration();
    dataset = s_vibration.bmi_dataset;
    unlock_vibration();
    return dataset;
}
