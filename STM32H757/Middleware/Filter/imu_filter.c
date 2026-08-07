#include "imu_filter.h"

#include <stddef.h>
#include <string.h>

#if defined(IMU_MANAGER_USE_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

static imu_filter_output_t imu_filter_output;
static uint8_t imu_filter_initialized;
static uint8_t imu_filter_window_count;
static uint8_t imu_filter_window_index;
static float imu_filter_alpha = IMU_FILTER_ALPHA;
static float imu_filter_accel_window[3][IMU_FILTER_MEDIAN_WINDOW];
static float imu_filter_mag_window[3][IMU_FILTER_MEDIAN_WINDOW];

#if defined(IMU_MANAGER_USE_FREERTOS)
static SemaphoreHandle_t imu_filter_mutex;
#endif

static void imu_filter_lock(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (imu_filter_mutex != NULL) {
        (void)xSemaphoreTake(imu_filter_mutex, portMAX_DELAY);
    }
#endif
}

static void imu_filter_unlock(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (imu_filter_mutex != NULL) {
        (void)xSemaphoreGive(imu_filter_mutex);
    }
#endif
}

static float imu_filter_median(const float *values, uint8_t count)
{
    float sorted[IMU_FILTER_MEDIAN_WINDOW];
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < count; ++i) {
        sorted[i] = values[i];
    }
    for (i = 1U; i < count; ++i) {
        const float value = sorted[i];
        j = i;
        while (j > 0U && sorted[j - 1U] > value) {
            sorted[j] = sorted[j - 1U];
            --j;
        }
        sorted[j] = value;
    }
    return sorted[count / 2U];
}

void imu_filter_init(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (imu_filter_mutex == NULL) {
        imu_filter_mutex = xSemaphoreCreateMutex();
    }
#endif
    imu_filter_lock();
    imu_filter_output = (imu_filter_output_t){0};
    imu_filter_initialized = 0U;
    imu_filter_window_count = 0U;
    imu_filter_window_index = 0U;
    imu_filter_alpha = IMU_FILTER_ALPHA;
    (void)memset(imu_filter_accel_window, 0, sizeof(imu_filter_accel_window));
    (void)memset(imu_filter_mag_window, 0, sizeof(imu_filter_mag_window));
    imu_filter_unlock();
}

void filter_set_vibration_profile(const imu_vibration_profile_t *profiles,
                                  uint8_t count)
{
    float max_rms = 0.0f;

    if (profiles == NULL || count == 0U) {
        return;
    }
    for (uint8_t index = 0U; index < count; ++index) {
        if (profiles[index].total_rms > max_rms) {
            max_rms = profiles[index].total_rms;
        }
    }

    /* Keep the existing IIR/median pipeline; vibration only selects alpha. */
    imu_filter_lock();
    imu_filter_alpha = max_rms > 0.50f ? 0.985f
                      : max_rms > 0.10f ? 0.975f
                                       : IMU_FILTER_ALPHA;
    imu_filter_unlock();
}

void imu_filter_update(const imu_calibrated_data_t *calibrated_data)
{
    float alpha;

    if (calibrated_data == NULL) {
        return;
    }

    imu_filter_lock();
    alpha = imu_filter_alpha;
    imu_filter_unlock();
    if (alpha <= 0.0f || alpha >= 1.0f) {
        alpha = IMU_FILTER_ALPHA;
    }

    imu_filter_lock();
    if (imu_filter_initialized == 0U) {
        uint8_t index;

        /* Prime all five slots so the first median is deterministic. */
        for (index = 0U; index < IMU_FILTER_MEDIAN_WINDOW; ++index) {
            imu_filter_accel_window[0][index] = calibrated_data->ax;
            imu_filter_accel_window[1][index] = calibrated_data->ay;
            imu_filter_accel_window[2][index] = calibrated_data->az;
            imu_filter_mag_window[0][index] = calibrated_data->mx;
            imu_filter_mag_window[1][index] = calibrated_data->my;
            imu_filter_mag_window[2][index] = calibrated_data->mz;
        }
        imu_filter_window_count = IMU_FILTER_MEDIAN_WINDOW;
        imu_filter_window_index = 0U;
        imu_filter_output.ax = calibrated_data->ax;
        imu_filter_output.ay = calibrated_data->ay;
        imu_filter_output.az = calibrated_data->az;
        imu_filter_output.mx = calibrated_data->mx;
        imu_filter_output.my = calibrated_data->my;
        imu_filter_output.mz = calibrated_data->mz;
        imu_filter_initialized = 1U;
    } else {
        const uint8_t index = imu_filter_window_index;
        float median_ax;
        float median_ay;
        float median_az;
        float median_mx;
        float median_my;
        float median_mz;

        imu_filter_accel_window[0][index] = calibrated_data->ax;
        imu_filter_accel_window[1][index] = calibrated_data->ay;
        imu_filter_accel_window[2][index] = calibrated_data->az;
        imu_filter_mag_window[0][index] = calibrated_data->mx;
        imu_filter_mag_window[1][index] = calibrated_data->my;
        imu_filter_mag_window[2][index] = calibrated_data->mz;
        imu_filter_window_index =
            (uint8_t)((index + 1U) % IMU_FILTER_MEDIAN_WINDOW);
        if (imu_filter_window_count < IMU_FILTER_MEDIAN_WINDOW) {
            ++imu_filter_window_count;
        }
        median_ax = imu_filter_median(imu_filter_accel_window[0],
                                      imu_filter_window_count);
        median_ay = imu_filter_median(imu_filter_accel_window[1],
                                      imu_filter_window_count);
        median_az = imu_filter_median(imu_filter_accel_window[2],
                                      imu_filter_window_count);
        median_mx = imu_filter_median(imu_filter_mag_window[0],
                                      imu_filter_window_count);
        median_my = imu_filter_median(imu_filter_mag_window[1],
                                      imu_filter_window_count);
        median_mz = imu_filter_median(imu_filter_mag_window[2],
                                      imu_filter_window_count);
        imu_filter_output.ax = (alpha * imu_filter_output.ax) +
                               ((1.0f - alpha) * median_ax);
        imu_filter_output.ay = (alpha * imu_filter_output.ay) +
                               ((1.0f - alpha) * median_ay);
        imu_filter_output.az = (alpha * imu_filter_output.az) +
                               ((1.0f - alpha) * median_az);
        imu_filter_output.mx = (alpha * imu_filter_output.mx) +
                               ((1.0f - alpha) * median_mx);
        imu_filter_output.my = (alpha * imu_filter_output.my) +
                               ((1.0f - alpha) * median_my);
        imu_filter_output.mz = (alpha * imu_filter_output.mz) +
                               ((1.0f - alpha) * median_mz);
    }
    imu_filter_output.timestamp = calibrated_data->timestamp;
    imu_filter_output.online = calibrated_data->online;
    imu_filter_unlock();
}

imu_filtered_data_t imu_filter_get_output(void)
{
    imu_filtered_data_t output;

    imu_filter_lock();
    output = imu_filter_output;
    imu_filter_unlock();
    return output;
}

uint8_t imu_filter_is_ready(void)
{
    uint8_t ready;

    imu_filter_lock();
    ready = imu_filter_initialized;
    imu_filter_unlock();
    return ready;
}
