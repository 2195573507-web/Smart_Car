#ifndef IMU_VIBRATION_H
#define IMU_VIBRATION_H

#include <stdint.h>

#include "imu_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_VIBRATION_SAMPLES UINT32_C(1000)
#define IMU_VIBRATION_PROFILE_COUNT UINT8_C(5)

typedef struct
{
    uint8_t radar_pwm;
    float rms_x;
    float rms_y;
    float rms_z;
    float total_rms;
} imu_vibration_profile_t;

void imu_vibration_init(void);
void imu_vibration_start(uint8_t radar_pwm);
void imu_vibration_select_profile(uint8_t index);
uint8_t imu_vibration_get_pwm_level(uint8_t index);
void imu_vibration_update(const imu_calibrated_data_t *sample);
uint8_t imu_vibration_is_complete(void);
uint32_t imu_vibration_get_sample_count(void);
imu_vibration_profile_t imu_vibration_get_result(void);
uint8_t imu_vibration_get_profile(uint8_t index,
                                  imu_vibration_profile_t *profile);

#ifdef __cplusplus
}
#endif

#endif /* IMU_VIBRATION_H */
