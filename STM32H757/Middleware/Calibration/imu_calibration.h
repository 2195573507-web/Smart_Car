#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_CALIBRATION_ACCEL_SAMPLES UINT32_C(5000)
#define IMU_CALIBRATION_SAMPLE_RATE_HZ UINT32_C(100)
#define IMU_CALIBRATION_GRAVITY_MPS2 9.80665f

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
} imu_raw_data_t;

typedef imu_raw_data_t imu_calibrated_data_t;

typedef struct
{
    float ax;
    float ay;
    float az;
    float mx;
    float my;
    float mz;
} imu_calibration_bias_t;

void imu_calibration_init(void);
void imu_calibration_start(void);
void imu_calibration_update(const imu_raw_data_t *raw_data);
uint8_t imu_calibration_is_complete(void);
uint32_t imu_calibration_get_sample_count(void);
uint32_t imu_calibration_get_sample_total(void);
uint8_t imu_calibration_get_progress(void);
imu_calibration_bias_t imu_calibration_get_bias(void);
imu_calibrated_data_t imu_calibration_apply(const imu_raw_data_t *raw_data);
imu_calibrated_data_t imu_calibration_get_data(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_CALIBRATION_H */
