#include "imu_vibration.h"

#include <math.h>
#include <string.h>

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
    imu_vibration_profile_t profiles[IMU_VIBRATION_PROFILE_COUNT];
    imu_vibration_profile_t current;
    uint8_t profile_index;
    uint8_t complete;
    uint32_t sample_count;
    uint32_t last_sample_timestamp;
    uint8_t timestamp_valid;
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

void imu_vibration_init(void)
{
#if defined(IMU_MANAGER_USE_FREERTOS)
    if (s_mutex == NULL) {
        s_mutex = xSemaphoreCreateMutex();
    }
#endif
    lock_vibration();
    (void)memset(&s_vibration, 0, sizeof(s_vibration));
    unlock_vibration();
}

void imu_vibration_start(uint8_t radar_pwm)
{
    lock_vibration();
    (void)memset(s_vibration.sum, 0, sizeof(s_vibration.sum));
    (void)memset(s_vibration.sum_square, 0, sizeof(s_vibration.sum_square));
    s_vibration.current = (imu_vibration_profile_t){.radar_pwm = radar_pwm};
    s_vibration.sample_count = 0U;
    s_vibration.last_sample_timestamp = 0U;
    s_vibration.timestamp_valid = 0U;
    s_vibration.complete = 0U;
    unlock_vibration();
}

void imu_vibration_update(const imu_calibrated_data_t *sample)
{
    if (sample == NULL || sample->online == 0U ||
        !isfinite(sample->ax) || !isfinite(sample->ay) ||
        !isfinite(sample->az)) {
        return;
    }

    lock_vibration();
    if (s_vibration.complete == 0U &&
        (s_vibration.timestamp_valid == 0U ||
         sample->timestamp != s_vibration.last_sample_timestamp) &&
        s_vibration.sample_count < IMU_VIBRATION_SAMPLES) {
        const float values[3] = {sample->ax, sample->ay, sample->az};
        for (uint8_t axis = 0U; axis < 3U; ++axis) {
            s_vibration.sum[axis] += values[axis];
            s_vibration.sum_square[axis] +=
                (double)values[axis] * values[axis];
        }
        s_vibration.last_sample_timestamp = sample->timestamp;
        s_vibration.timestamp_valid = 1U;
        ++s_vibration.sample_count;
        if (s_vibration.sample_count >= IMU_VIBRATION_SAMPLES) {
            const double count = (double)s_vibration.sample_count;
            float rms[3];
            for (uint8_t axis = 0U; axis < 3U; ++axis) {
                const double mean = s_vibration.sum[axis] / count;
                const double variance =
                    (s_vibration.sum_square[axis] / count) - (mean * mean);
                rms[axis] = sqrtf((float)(variance > 0.0 ? variance : 0.0));
            }
            s_vibration.current.rms_x = rms[0];
            s_vibration.current.rms_y = rms[1];
            s_vibration.current.rms_z = rms[2];
            s_vibration.current.total_rms =
                sqrtf((rms[0] * rms[0] + rms[1] * rms[1] +
                       rms[2] * rms[2]) /
                      3.0f);
            if (s_vibration.profile_index < IMU_VIBRATION_PROFILE_COUNT) {
                s_vibration.profiles[s_vibration.profile_index] =
                    s_vibration.current;
            }
            s_vibration.complete = 1U;
        }
    }
    unlock_vibration();
}

uint8_t imu_vibration_is_complete(void)
{
    uint8_t complete;
    lock_vibration();
    complete = s_vibration.complete;
    unlock_vibration();
    return complete;
}

uint32_t imu_vibration_get_sample_count(void)
{
    uint32_t count;
    lock_vibration();
    count = s_vibration.sample_count;
    unlock_vibration();
    return count;
}

imu_vibration_profile_t imu_vibration_get_result(void)
{
    imu_vibration_profile_t result;
    lock_vibration();
    result = s_vibration.current;
    unlock_vibration();
    return result;
}

uint8_t imu_vibration_get_profile(uint8_t index,
                                  imu_vibration_profile_t *profile)
{
    if (profile == NULL || index >= IMU_VIBRATION_PROFILE_COUNT) {
        return 0U;
    }
    lock_vibration();
    *profile = s_vibration.profiles[index];
    unlock_vibration();
    return 1U;
}

/* Called by the boot manager before each new level. */
void imu_vibration_select_profile(uint8_t index)
{
    if (index < IMU_VIBRATION_PROFILE_COUNT) {
        lock_vibration();
        s_vibration.profile_index = index;
        unlock_vibration();
    }
}

uint8_t imu_vibration_get_pwm_level(uint8_t index)
{
    return index < IMU_VIBRATION_PROFILE_COUNT ? s_pwm_levels[index] : 0U;
}
