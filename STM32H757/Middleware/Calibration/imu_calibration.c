#include "imu_calibration.h"

#include <math.h>
#include <string.h>

#if defined(IMU_MANAGER_USE_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

typedef struct
{
    double sum_x;
    double sum_y;
    double sum_z;
    double sum_square;
    imu_calibration_bias_t bias;
    imu_calibrated_data_t last_calibrated;
    uint32_t sample_count;
    uint32_t last_sample_timestamp;
    uint8_t timestamp_valid;
    uint8_t complete;
} imu_calibration_state_t;

static imu_calibration_state_t s_calibration;

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

void imu_calibration_init(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
    }
#endif
    lock_calibration();
    (void)memset(&s_calibration, 0, sizeof(s_calibration));
    unlock_calibration();
}

void imu_calibration_start(void)
{
    lock_calibration();
    (void)memset(&s_calibration, 0, sizeof(s_calibration));
    unlock_calibration();
}

void imu_calibration_update(const imu_raw_data_t *raw_data)
{
    if (raw_data == NULL || raw_data->online == 0U || s_calibration.complete != 0U ||
        !isfinite(raw_data->ax) || !isfinite(raw_data->ay) ||
        !isfinite(raw_data->az)) {
        return;
    }

    lock_calibration();
    if (s_calibration.complete == 0U &&
        (s_calibration.timestamp_valid == 0U ||
         raw_data->timestamp != s_calibration.last_sample_timestamp) &&
        s_calibration.sample_count < IMU_CALIBRATION_ACCEL_SAMPLES) {
        s_calibration.sum_x += raw_data->ax;
        s_calibration.sum_y += raw_data->ay;
        s_calibration.sum_z += raw_data->az;
        s_calibration.sum_square += (double)raw_data->ax * raw_data->ax +
                                    (double)raw_data->ay * raw_data->ay +
                                    (double)raw_data->az * raw_data->az;
        s_calibration.last_sample_timestamp = raw_data->timestamp;
        s_calibration.timestamp_valid = 1U;
        ++s_calibration.sample_count;
        if (s_calibration.sample_count >= IMU_CALIBRATION_ACCEL_SAMPLES) {
            const double count = (double)s_calibration.sample_count;
            s_calibration.bias.ax = (float)(s_calibration.sum_x / count);
            s_calibration.bias.ay = (float)(s_calibration.sum_y / count);
            s_calibration.bias.az =
                (float)(s_calibration.sum_z / count) - IMU_CALIBRATION_GRAVITY_MPS2;
            s_calibration.complete = 1U;
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
    count = s_calibration.sample_count;
    unlock_calibration();
    return count;
}

uint32_t imu_calibration_get_sample_total(void)
{
    return IMU_CALIBRATION_ACCEL_SAMPLES;
}

uint8_t imu_calibration_get_progress(void)
{
    const uint32_t count = imu_calibration_get_sample_count();
    const uint32_t progress =
        (count * 100U) / IMU_CALIBRATION_ACCEL_SAMPLES;
    return (uint8_t)(progress > 100U ? 100U : progress);
}

imu_calibration_bias_t imu_calibration_get_bias(void)
{
    imu_calibration_bias_t bias;
    lock_calibration();
    bias = s_calibration.bias;
    unlock_calibration();
    return bias;
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
