#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_CAL_STATIC_WINDOW_MS UINT32_C(6000)
/* Retained for callers that use the older calibration-window name. */
#define IMU_CALIBRATION_WINDOW_MS IMU_CAL_STATIC_WINDOW_MS
#define IMU_CALIBRATION_WINDOW_US \
    ((uint64_t)IMU_CAL_STATIC_WINDOW_MS * UINT64_C(1000))
#define IMU_CALIBRATION_SAMPLE_RATE_HZ UINT32_C(100)
#define IMU_CALIBRATION_SAMPLE_TOLERANCE_PERCENT UINT32_C(10)
/* BMI323 runs at 200 Hz in the normal image. A six-second static window
 * leaves rate/scheduling headroom while the gyro-bias accumulator itself is
 * capped at exactly this many accepted samples. */
#define IMU_CAL_GYRO_BIAS_SAMPLE_COUNT UINT32_C(1000)
#define BMI_ACCEL_STD_MAX (0.10f)
#define LSM_ACCEL_STD_MAX (0.50f)
#define IMU_CALIBRATION_LSM303_NOMINAL_SAMPLES \
    ((IMU_CALIBRATION_SAMPLE_RATE_HZ * IMU_CAL_STATIC_WINDOW_MS) / \
     UINT32_C(1000))

typedef struct
{
    /* Legacy LSM303 view retained for the existing calibration/filter path. */
    float ax;
    float ay;
    float az;
    float mx;
    float my;
    float mz;
    uint32_t timestamp;
    uint64_t timestamp_us;
    uint8_t online;

    /* Unified snapshot fields. Values are never inferred from sensor order. */
    float lsm_ax;
    float lsm_ay;
    float lsm_az;
    float lsm_mx;
    float lsm_my;
    float lsm_mz;
    float bmi_ax;
    float bmi_ay;
    float bmi_az;
    float bmi_gx;
    float bmi_gy;
    float bmi_gz;
    uint32_t lsm_timestamp;
    uint32_t bmi_timestamp;
    uint64_t lsm_timestamp_us;
    uint64_t bmi_timestamp_us;
    uint8_t lsm_accel_valid;
    uint8_t lsm_mag_valid;
    uint8_t bmi_accel_valid;
    uint8_t bmi_gyro_valid;
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

typedef struct
{
    float x;
    float y;
    float z;
} imu_bias_xyz_t;

typedef struct
{
    uint32_t lsm_accel;
    uint32_t bmi_accel;
    uint32_t bmi_gyro;
} imu_calibration_sample_counts_t;

typedef struct
{
    uint16_t configured_rate_hz;
    uint16_t actual_rate_hz;
    uint32_t expected_sample_count;
    uint32_t minimum_sample_count;
    uint32_t actual_sample_count;
    uint8_t quality_ok;
} imu_sample_quality_t;

typedef struct
{
    imu_sample_quality_t lsm_accel;
    imu_sample_quality_t bmi_accel;
    imu_sample_quality_t bmi_gyro;
} imu_calibration_quality_t;

/* Frozen at static-window closure for consumers that need the raw stationary
 * observation without treating the gravity vector as an accelerometer bias. */
typedef struct
{
    float accel_mean[3];
    float accel_std_mps2;
    float gyro_rms_radps;
    float valid_ratio;
} imu_calibration_static_sensor_t;

typedef struct
{
    imu_calibration_static_sensor_t lsm;
    imu_calibration_static_sensor_t bmi;
} imu_calibration_static_statistics_t;

typedef struct
{
    imu_bias_xyz_t lsm_accel_bias;
    imu_bias_xyz_t bmi_accel_bias;
    imu_bias_xyz_t bmi_gyro_bias;
    imu_calibration_sample_counts_t sample_counts;
} imu_calibration_result_t;

void imu_calibration_init(void);
void imu_calibration_start(void);
void imu_calibration_begin_window(uint64_t start_timestamp_us,
                                  uint16_t bmi_configured_rate_hz);
uint8_t imu_calibration_bmi_capture_active(void);
uint8_t imu_calibration_window_expired(uint64_t now_timestamp_us);
uint8_t imu_calibration_finish_window(uint64_t now_timestamp_us);
/* True when a dynamic sample was observed during the most recent static
 * window. The flag is cleared by imu_calibration_start(). */
uint8_t imu_calibration_static_motion_detected(void);
void imu_calibration_update(const imu_raw_data_t *raw_data);
void imu_calibration_update_bmi323(float accel_x, float accel_y, float accel_z,
                                   float gyro_x, float gyro_y, float gyro_z,
                                   uint64_t timestamp_us);
uint8_t imu_calibration_is_complete(void);
uint32_t imu_calibration_get_sample_count(void);
uint32_t imu_calibration_get_sample_total(void);
uint8_t imu_calibration_get_progress(void);
imu_calibration_bias_t imu_calibration_get_bias(void);
imu_calibration_result_t imu_calibration_get_result(void);
imu_calibration_sample_counts_t imu_calibration_get_sample_counts(void);
imu_calibration_quality_t imu_calibration_get_quality(void);
imu_calibration_static_statistics_t imu_calibration_get_static_statistics(void);
uint8_t imu_calibration_is_lsm_complete(void);
uint8_t imu_calibration_is_bmi_complete(void);
imu_calibrated_data_t imu_calibration_apply(const imu_raw_data_t *raw_data);
imu_calibrated_data_t imu_calibration_get_data(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_CALIBRATION_H */
