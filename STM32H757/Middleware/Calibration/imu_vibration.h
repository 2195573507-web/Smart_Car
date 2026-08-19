#ifndef IMU_VIBRATION_H
#define IMU_VIBRATION_H

#include <stdint.h>

#include "imu_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

/* A vibration run is admitted against one shared, monotonic microsecond
 * interval.  The sensor tasks may have normal scheduling jitter, but neither
 * source is accepted before the common start or after the common end. */
#define IMU_VIBRATION_STEP_WINDOW_MS UINT32_C(10000)
/* Retained for callers that use the older generic window-duration name. */
#define IMU_VIBRATION_WINDOW_DURATION_MS IMU_VIBRATION_STEP_WINDOW_MS
#define IMU_VIBRATION_WINDOW_DURATION_US \
    ((uint64_t)IMU_VIBRATION_STEP_WINDOW_MS * UINT64_C(1000))
#define IMU_VIBRATION_LSM_SAMPLE_RATE_HZ UINT16_C(100)
#define IMU_VIBRATION_BMI_DEFAULT_SAMPLE_RATE_HZ UINT16_C(100)
#define IMU_VIBRATION_SAMPLE_QUALITY_FLOOR_PERCENT UINT32_C(90)
/* Preserve the existing tolerance macro for source compatibility. */
#define IMU_VIBRATION_SAMPLE_TOLERANCE_PERCENT \
    (UINT32_C(100) - IMU_VIBRATION_SAMPLE_QUALITY_FLOOR_PERCENT)
#define IMU_VIBRATION_SAMPLES \
    ((IMU_VIBRATION_LSM_SAMPLE_RATE_HZ * IMU_VIBRATION_STEP_WINDOW_MS) / \
     UINT32_C(1000))
#define IMU_VIBRATION_PROFILE_COUNT UINT8_C(5)

typedef enum
{
    IMU_VIBRATION_SENSOR_LSM303 = 0x01U,
    IMU_VIBRATION_SENSOR_BMI323 = 0x02U
} imu_vibration_sensor_id_t;

typedef enum
{
    IMU_MOTION_LABEL_UNKNOWN = 0U,
    IMU_MOTION_LABEL_STATIC = 1U,
    IMU_MOTION_LABEL_VIBRATION = 2U
} imu_motion_label_t;

/* Timestamp is the MCU monotonic microsecond clock.  LSM303 samples carry
 * an explicit zero gyro vector because that sensor has no gyro channel. */
typedef struct
{
    uint64_t timestamp_us;
    uint8_t sensor_id;
    uint16_t sample_rate;
    uint32_t sequence;
    float accel[3];
    float gyro[3];
    uint8_t valid;
    uint8_t motion_label;
} imu_vibration_sample_t;

/* A sink must complete in bounded, non-blocking time.  It receives records
 * from exactly one source and returns nonzero only after accepting the record. */
typedef uint8_t (*imu_vibration_dataset_sink_t)(
    const imu_vibration_sample_t *sample, void *context);

typedef struct
{
    uint64_t common_start_timestamp_us;
    uint64_t common_end_timestamp_us;
    uint64_t duration_us;
    uint32_t window_sequence;
    uint8_t active;
    uint8_t complete;
} imu_vibration_window_t;

typedef struct
{
    uint64_t common_start_timestamp_us;
    uint64_t common_end_timestamp_us;
    uint32_t window_sequence;
    uint32_t captured_count;
    uint32_t valid_count;
    uint32_t invalid_count;
    uint32_t delivered_count;
    uint32_t delivery_failure_count;
    uint16_t sample_rate;
    uint16_t configured_rate_hz;
    uint16_t actual_rate_hz;
    uint8_t sensor_id;
    uint8_t motion_label;
    uint8_t radar_pwm;
    uint8_t active;
    uint8_t complete;
    uint8_t quality_ok;
} imu_vibration_dataset_t;

typedef struct
{
    uint8_t radar_pwm;
    uint32_t sample_count;
    uint32_t timestamp;
    uint64_t common_start_timestamp;
    uint64_t common_end_timestamp;
    uint16_t sample_rate;
    uint32_t invalid_sample_count;
    uint32_t captured;
    uint32_t invalid;
    uint16_t configured_rate_hz;
    uint16_t actual_rate_hz;
    uint8_t quality_ok;
    float rms_x;
    float rms_y;
    float rms_z;
    float total_rms;
} lsm_vibration_profile_t;

typedef struct
{
    uint8_t radar_pwm;
    uint32_t sample_count;
    uint32_t timestamp;
    uint64_t common_start_timestamp;
    uint64_t common_end_timestamp;
    uint16_t sample_rate;
    uint32_t invalid_sample_count;
    uint32_t captured;
    uint32_t invalid;
    uint16_t configured_rate_hz;
    uint16_t actual_rate_hz;
    uint8_t quality_ok;
    float accel_rms_x;
    float accel_rms_y;
    float accel_rms_z;
    float accel_total_rms;
    float gyro_rms_x;
    float gyro_rms_y;
    float gyro_rms_z;
    float gyro_total_rms;
} bmi_vibration_profile_t;

void imu_vibration_init(void);
void imu_vibration_start(uint8_t radar_pwm);
uint8_t imu_vibration_start_window(uint8_t radar_pwm,
                                   uint64_t common_start_timestamp_us,
                                   uint64_t duration_us,
                                   uint16_t bmi_configured_rate_hz);
void imu_vibration_set_lsm_dataset_sink(imu_vibration_dataset_sink_t sink,
                                        void *context);
void imu_vibration_set_bmi_dataset_sink(imu_vibration_dataset_sink_t sink,
                                        void *context);
void imu_vibration_select_profile(uint8_t index);
uint8_t imu_vibration_get_pwm_level(uint8_t index);
void imu_vibration_capture_lsm(float ax, float ay, float az,
                               uint64_t timestamp_us, uint8_t valid);
void imu_vibration_capture_bmi(float ax, float ay, float az,
                               float gx, float gy, float gz,
                               uint64_t timestamp_us, uint16_t sample_rate,
                               uint8_t valid);
void imu_vibration_poll(uint64_t timestamp_us);
void imu_vibration_update(const imu_calibrated_data_t *sample);
uint8_t imu_vibration_is_complete(void);
uint8_t imu_vibration_is_lsm_complete(void);
uint8_t imu_vibration_is_bmi_complete(void);
uint32_t imu_vibration_get_sample_count(void);
lsm_vibration_profile_t imu_vibration_get_lsm_result(void);
bmi_vibration_profile_t imu_vibration_get_bmi_result(void);
uint32_t imu_vibration_get_lsm_sample_count(void);
uint32_t imu_vibration_get_bmi_sample_count(void);
uint8_t imu_vibration_get_lsm_profile(uint8_t index,
                                      lsm_vibration_profile_t *profile);
uint8_t imu_vibration_get_bmi_profile(uint8_t index,
                                      bmi_vibration_profile_t *profile);
imu_vibration_window_t imu_vibration_get_window(void);
imu_vibration_dataset_t imu_vibration_get_lsm_dataset(void);
imu_vibration_dataset_t imu_vibration_get_bmi_dataset(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_VIBRATION_H */
