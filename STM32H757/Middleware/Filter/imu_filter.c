#include "imu_filter.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "log_service.h"

#define IMU_FILTER_VIBRATION_MEDIUM_RMS_MPS2 0.980665f
#define IMU_FILTER_VIBRATION_HIGH_RMS_MPS2   1.961330f

typedef enum
{
    IMU_FILTER_VIBRATION_INVALID = 0U,
    IMU_FILTER_VIBRATION_LOW,
    IMU_FILTER_VIBRATION_MEDIUM,
    IMU_FILTER_VIBRATION_HIGH
} imu_filter_vibration_level_t;

#if defined(IMU_MANAGER_USE_FREERTOS)
#include "FreeRTOS.h"
#include "semphr.h"
#endif

static imu_filter_output_t imu_filter_output;
static uint8_t imu_filter_initialized;
static uint8_t imu_filter_window_count;
static uint8_t imu_filter_window_index;
static float imu_filter_alpha = IMU_FILTER_ALPHA;
static uint8_t imu_filter_vibration_log_emitted;
static lsm_vibration_profile_t imu_filter_profiles[IMU_VIBRATION_PROFILE_COUNT];
static uint8_t imu_filter_profile_count;
static uint8_t imu_filter_radar_pwm;
static uint8_t imu_filter_runtime_selection_valid;
static uint8_t imu_filter_runtime_profile_valid;
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

static const char *imu_filter_vibration_level_name(
    imu_filter_vibration_level_t level)
{
    switch (level) {
    case IMU_FILTER_VIBRATION_LOW:
        return "LOW";
    case IMU_FILTER_VIBRATION_MEDIUM:
        return "MEDIUM";
    case IMU_FILTER_VIBRATION_HIGH:
        return "HIGH";
    case IMU_FILTER_VIBRATION_INVALID:
    default:
        return "INVALID";
    }
}

static float imu_filter_alpha_for_rms(float rms,
                                      imu_filter_vibration_level_t *level,
                                      const char **alpha_text)
{
    if (level != NULL) {
        *level = IMU_FILTER_VIBRATION_INVALID;
    }
    if (alpha_text != NULL) {
        *alpha_text = "0.950";
    }
    if (!isfinite(rms) || rms < 0.0f) {
        return IMU_FILTER_ALPHA;
    }
    if (rms < IMU_FILTER_VIBRATION_MEDIUM_RMS_MPS2) {
        if (level != NULL) {
            *level = IMU_FILTER_VIBRATION_LOW;
        }
        return IMU_FILTER_ALPHA;
    }
    if (rms < IMU_FILTER_VIBRATION_HIGH_RMS_MPS2) {
        if (level != NULL) {
            *level = IMU_FILTER_VIBRATION_MEDIUM;
        }
        if (alpha_text != NULL) {
            *alpha_text = "0.975";
        }
        return 0.975f;
    }
    if (level != NULL) {
        *level = IMU_FILTER_VIBRATION_HIGH;
    }
    if (alpha_text != NULL) {
        *alpha_text = "0.985";
    }
    return 0.985f;
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
    imu_filter_vibration_log_emitted = 0U;
    (void)memset(imu_filter_profiles, 0, sizeof(imu_filter_profiles));
    imu_filter_profile_count = 0U;
    imu_filter_radar_pwm = 0U;
    imu_filter_runtime_selection_valid = 0U;
    imu_filter_runtime_profile_valid = 0U;
    (void)memset(imu_filter_accel_window, 0, sizeof(imu_filter_accel_window));
    (void)memset(imu_filter_mag_window, 0, sizeof(imu_filter_mag_window));
    imu_filter_unlock();
}

void filter_set_vibration_profile(const lsm_vibration_profile_t *profiles,
                                  uint8_t count)
{
    float max_rms = 0.0f;
    float selected_alpha = IMU_FILTER_ALPHA;
    const char *alpha_text = "0.950";
    imu_filter_vibration_level_t vibration_level =
        IMU_FILTER_VIBRATION_INVALID;
    uint8_t valid_rms = 0U;
    uint8_t emit_log = 0U;
    char log_line[96];

    if (profiles != NULL && count != 0U) {
        for (uint8_t index = 0U; index < count; ++index) {
            const float total_rms = profiles[index].total_rms;

            if (isfinite(total_rms) && total_rms >= 0.0f &&
                (valid_rms == 0U || total_rms > max_rms)) {
                max_rms = total_rms;
                valid_rms = 1U;
            }
        }
    }

    if (valid_rms != 0U) {
        selected_alpha = imu_filter_alpha_for_rms(max_rms, &vibration_level,
                                                   &alpha_text);
    }

    /* Keep the existing IIR/median pipeline; vibration only selects alpha. */
    imu_filter_lock();
    imu_filter_profile_count = count > IMU_VIBRATION_PROFILE_COUNT
                                   ? IMU_VIBRATION_PROFILE_COUNT
                                   : count;
    if (profiles != NULL && imu_filter_profile_count != 0U) {
        (void)memcpy(imu_filter_profiles, profiles,
                     (size_t)imu_filter_profile_count *
                         sizeof(imu_filter_profiles[0]));
    } else {
        (void)memset(imu_filter_profiles, 0, sizeof(imu_filter_profiles));
    }
    imu_filter_alpha = selected_alpha;
    if (imu_filter_vibration_log_emitted == 0U) {
        imu_filter_vibration_log_emitted = 1U;
        emit_log = 1U;
    }
    imu_filter_unlock();

    if (emit_log != 0U) {
        if (valid_rms != 0U) {
            const uint32_t rms_milli =
                (uint32_t)((max_rms * 1000.0f) + 0.5f);
            (void)snprintf(log_line, sizeof(log_line),
                           "IMU_FILTER vibration_level=%s alpha=%s rms=%lu.%03lu m/s2",
                           imu_filter_vibration_level_name(vibration_level),
                           alpha_text,
                           (unsigned long)(rms_milli / 1000U),
                           (unsigned long)(rms_milli % 1000U));
        } else {
            (void)snprintf(log_line, sizeof(log_line),
                           "IMU_FILTER vibration_level=INVALID alpha=%s rms=invalid",
                           alpha_text);
        }
        LOG_INFO(log_line);
    }

    /* PWM 0 is the runtime-safe default. Re-selecting it here also means a
     * profile load cannot leave the filter using the previous max-RMS alpha. */
    imu_filter_set_radar_pwm(0U);
}

void imu_filter_set_radar_pwm(uint8_t radar_pwm_percent)
{
    float selected_alpha = IMU_FILTER_ALPHA;
    float selected_rms = 0.0f;
    uint8_t profile_valid = 0U;
    uint8_t selection_changed = 0U;
    uint8_t previous_pwm;
    uint8_t previous_profile_valid;
    uint8_t previous_selection_valid;
    char log_line[96];

    imu_filter_lock();
    previous_pwm = imu_filter_radar_pwm;
    previous_profile_valid = imu_filter_runtime_profile_valid;
    previous_selection_valid = imu_filter_runtime_selection_valid;
    if (radar_pwm_percent != 0U && radar_pwm_percent <= 100U) {
        for (uint8_t index = 0U; index < imu_filter_profile_count; ++index) {
            const lsm_vibration_profile_t *profile = &imu_filter_profiles[index];
            if (profile->radar_pwm == radar_pwm_percent &&
                isfinite(profile->total_rms) && profile->total_rms >= 0.0f) {
                selected_rms = profile->total_rms;
                selected_alpha = imu_filter_alpha_for_rms(selected_rms, NULL,
                                                           NULL);
                profile_valid = 1U;
                break;
            }
        }
    }
    imu_filter_radar_pwm = radar_pwm_percent;
    imu_filter_alpha = selected_alpha;
    imu_filter_runtime_profile_valid = profile_valid;
    if (previous_selection_valid == 0U || previous_pwm != radar_pwm_percent ||
        previous_profile_valid != profile_valid) {
        imu_filter_runtime_selection_valid = 1U;
        selection_changed = 1U;
    }
    imu_filter_unlock();

    if (selection_changed == 0U) {
        return;
    }
    if (profile_valid != 0U) {
        const uint32_t rms_milli =
            (uint32_t)((selected_rms * 100.0f) + 0.5f);
        const char *alpha_text = selected_alpha >= 0.984f ? "0.985"
                                  : selected_alpha >= 0.974f ? "0.975"
                                                              : "0.950";
        (void)snprintf(log_line, sizeof(log_line),
                       "RADAR_PWM_FILTER pwm=%u rms=%lu.%02lu alpha=%s",
                       (unsigned)radar_pwm_percent,
                       (unsigned long)(rms_milli / 100U),
                       (unsigned long)(rms_milli % 100U), alpha_text);
    } else {
        (void)snprintf(log_line, sizeof(log_line),
                       "RADAR_PWM_FILTER pwm=%u rms=invalid alpha=0.950",
                       (unsigned)radar_pwm_percent);
    }
    LOG_INFO(log_line);
}

uint8_t imu_filter_get_radar_pwm(void)
{
    return imu_filter_radar_pwm;
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
