#ifndef DUAL_AHRS_H
#define DUAL_AHRS_H

#include <stddef.h>
#include <stdint.h>

#include "imu_leveling.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DUAL_AHRS_SCHEMA UINT8_C(2)
#define DUAL_AHRS_PAYLOAD_LENGTH UINT16_C(80)
#define DUAL_AHRS_PI 3.14159265358979323846f
#define DUAL_AHRS_TWO_PI (2.0f * DUAL_AHRS_PI)

typedef struct
{
    float x;
    float y;
    float z;
} dual_ahrs_vector3_t;

typedef struct
{
    dual_ahrs_vector3_t bmi_accel;
    dual_ahrs_vector3_t bmi_gyro;
    dual_ahrs_vector3_t lsm_accel;
} dual_ahrs_bias_t;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} dual_ahrs_quaternion_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    dual_ahrs_quaternion_t quaternion;
    uint8_t valid;
} dual_ahrs_attitude_t;

typedef struct
{
    dual_ahrs_vector3_t bmi_accel;
    dual_ahrs_vector3_t gyro;
    dual_ahrs_vector3_t lsm_accel;
    dual_ahrs_vector3_t mag;
    uint64_t bmi_timestamp_us;
    uint64_t lsm_timestamp_us;
    uint8_t bmi_accel_valid;
    uint8_t bmi_gyro_valid;
    uint8_t lsm_accel_valid;
    uint8_t lsm_mag_valid;
} dual_ahrs_input_t;

typedef enum
{
    DUAL_AHRS_STATE_WAIT_CAL = 0U,
    DUAL_AHRS_STATE_RESET = DUAL_AHRS_STATE_WAIT_CAL,
    DUAL_AHRS_STATE_WARMUP = 1U,
    DUAL_AHRS_STATE_TRACKING = 2U,
    DUAL_AHRS_STATE_RUNNING = DUAL_AHRS_STATE_TRACKING,
    DUAL_AHRS_STATE_DEGRADED = 3U,
    DUAL_AHRS_STATE_FAULT = 4U,
    DUAL_AHRS_STATE_READY = 5U
} dual_ahrs_state_t;

typedef struct
{
    uint8_t schema;
    uint8_t flags;
    uint32_t timestamp_ms;
    uint32_t sample_sequence;
    dual_ahrs_attitude_t primary;
    dual_ahrs_attitude_t redundant;
    dual_ahrs_vector3_t delta_rad;
    float gravity_confidence;
    float magnetic_confidence;
    dual_ahrs_state_t state;
} dual_ahrs_output_t;

/* Keep these helpers public for deterministic host/replay tests. */
float gravity_confidence(const dual_ahrs_vector3_t *accel);
float mag_confidence(const dual_ahrs_vector3_t *mag);
float dual_ahrs_wrap_pi(float angle);
float delta_roll(float primary, float redundant);
float delta_pitch(float primary, float redundant);
float delta_yaw(float primary, float redundant);

void dual_ahrs_init(void);
/* Pass NULL to hold the estimator at WAIT_CAL and clear runtime history. */
void dual_ahrs_set_bias(const dual_ahrs_bias_t *bias);
void dual_ahrs_set_leveling(const imu_leveling_state_t *bmi,
                            const imu_leveling_state_t *lsm);
void dual_ahrs_set_local_gravity(float gravity_mps2);
void dual_ahrs_update(const dual_ahrs_input_t *input);
void dual_ahrs_get_output(dual_ahrs_output_t *output);

/* Returns the latest primary yaw and transformed/filtered body Z rate. */
uint8_t dual_ahrs_get_heading_state(float *yaw_rad, float *gyro_z_rad_s);

/* Serializes the schema=2 SCBP-CAN DualAHRS payload. */
int dual_ahrs_pack_payload(uint8_t *payload, size_t capacity);

#ifdef __cplusplus
}
#endif

#endif /* DUAL_AHRS_H */
