#ifndef IMU_LEVELING_H
#define IMU_LEVELING_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_LEVELING_TILT_MAX_DEG (30.0f)
#define IMU_LEVELING_G_MIN (9.0f)
#define IMU_LEVELING_G_MAX (10.5f)
#define IMU_LEVELING_G_DEFAULT_MPS2 \
    ((IMU_LEVELING_G_MIN + IMU_LEVELING_G_MAX) * 0.5f)

/* Static-window admission limits. The gyro gate applies to BMI323; callers
 * without a gyro provide 0.0f and retain the accelerometer-quality gates. */
#define IMU_LEVELING_VALID_RATIO_MIN (0.90f)
#define IMU_LEVELING_GYRO_RMS_MAX_RADPS (0.15f)
#define IMU_LEVELING_ACCEL_STD_MAX_MPS2 (0.15f)

typedef enum
{
    IMU_LEVELING_FALLBACK_NONE = 0,
    IMU_LEVELING_FALLBACK_NOT_COMPUTED,
    IMU_LEVELING_FALLBACK_SAMPLE_INSUFFICIENT,
    IMU_LEVELING_FALLBACK_GYRO_NOISE,
    IMU_LEVELING_FALLBACK_ACCEL_VARIANCE,
    IMU_LEVELING_FALLBACK_GRAVITY_OUT_OF_RANGE,
    IMU_LEVELING_FALLBACK_TILT_EXCEEDED,
    IMU_LEVELING_FALLBACK_INVERTED_START,
    IMU_LEVELING_FALLBACK_NONFINITE,
    IMU_LEVELING_FALLBACK_MATRIX_INVALID
} imu_leveling_fallback_reason_t;

typedef struct
{
    float r_level[3][3];
    float g_local_mps2;
    float tilt_deg;
    float accel_mean[3];
    float gyro_rms_radps;
    float accel_std_mps2;
    float valid_ratio;
    bool valid;
    imu_leveling_fallback_reason_t fallback_reason;
} imu_leveling_state_t;

void imu_leveling_init(imu_leveling_state_t *state);
bool imu_leveling_compute(imu_leveling_state_t *state,
                          const float accel_mean[3],
                          float gyro_rms,
                          float accel_std,
                          float valid_ratio);
/* Explicit sensor policy entry. The legacy entry retains the default limit
 * for any caller that does not own a sensor-specific noise budget. */
bool imu_leveling_compute_with_accel_std_limit(imu_leveling_state_t *state,
                                               const float accel_mean[3],
                                               float gyro_rms,
                                               float accel_std,
                                               float valid_ratio,
                                               float accel_std_max);
void imu_leveling_rotate_vector(const imu_leveling_state_t *state,
                                const float v_in[3],
                                float v_out[3]);

#ifdef __cplusplus
}
#endif

#endif /* IMU_LEVELING_H */
