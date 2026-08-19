#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include <stdint.h>

#include "imu_calibration.h"
#include "imu_vibration.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_FILTER_MEDIAN_WINDOW 5U
#define IMU_FILTER_ALPHA 0.95f

typedef struct
{
    float ax;
    float ay;
    float az;
    float mx;
    float my;
    float mz;
    uint32_t timestamp;
    uint8_t online;
} imu_filtered_data_t;

/* Compatibility name for existing runtime consumers. */
typedef imu_filtered_data_t imu_filter_output_t;

void imu_filter_init(void);
void filter_set_vibration_profile(const lsm_vibration_profile_t *profiles,
                                  uint8_t count);
/* Select the cached calibration profile for the current radar PWM. Values
 * without an exact calibrated profile fall back to IMU_FILTER_ALPHA. */
void imu_filter_set_radar_pwm(uint8_t radar_pwm_percent);
uint8_t imu_filter_get_radar_pwm(void);
void imu_filter_update(const imu_calibrated_data_t *calibrated_data);
imu_filtered_data_t imu_filter_get_output(void);
uint8_t imu_filter_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_FILTER_H */
