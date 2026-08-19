#ifndef IMU_BOOT_MANAGER_H
#define IMU_BOOT_MANAGER_H

#include <stdint.h>

#include "bsp_status.h"
#include "imu_calibration.h"
#include "imu_vibration.h"

#ifdef __cplusplus
extern "C" {
#endif

/* DUAL_IMU_BOOT is the only lifecycle authority.  Legacy boot states below
 * remain a read-only compatibility view for existing filter/log consumers. */
typedef enum
{
    IMU_PHASE_IDLE = 0,
    IMU_PHASE_INIT,
    IMU_PHASE_SELF_TEST,
    IMU_PHASE_STATIC_CALIBRATION,
    IMU_PHASE_VIBRATION_CAPTURE,
    IMU_PHASE_VERIFY,
    IMU_PHASE_READY,
    IMU_PHASE_FAILED,
    IMU_PHASE_COUNT
} imu_phase_t;

typedef enum
{
    IMU_ERROR_NONE = 0,
    IMU_ERROR_LSM_INIT,
    IMU_ERROR_BMI_INIT,
    IMU_ERROR_INIT_TIMEOUT,
    IMU_ERROR_LSM_SELF_TEST,
    IMU_ERROR_BMI_SELF_TEST,
    IMU_ERROR_RADAR_SYNC_TIMEOUT,
    IMU_ERROR_STATIC_WINDOW,
    IMU_ERROR_VIBRATION_WINDOW,
    IMU_ERROR_CAL_EVENT_TIMEOUT,
    IMU_ERROR_CAL_EVENT_REJECTED,
    IMU_ERROR_VERIFY_TIMEOUT,
    IMU_ERROR_TASK_CREATE
} imu_error_t;

typedef struct
{
    uint32_t start_timestamp;
    uint32_t end_timestamp;
} imu_phase_timing_t;

typedef struct
{
    imu_phase_t phase;

    uint8_t lsm_progress;
    uint8_t bmi_progress;

    uint8_t overall_progress;

    uint32_t phase_start_time;

    imu_error_t error;

    uint32_t phase_end_time;
    uint8_t flags;
    uint8_t vibration_index;
    uint8_t radar_pwm;
    imu_phase_timing_t phase_timing[IMU_PHASE_COUNT];
} dual_imu_manager_t;

#define DUAL_IMU_STATUS_FLAG_LSM_PHASE_COMPLETE UINT8_C(0x01)
#define DUAL_IMU_STATUS_FLAG_BMI_PHASE_COMPLETE UINT8_C(0x02)
#define DUAL_IMU_STATUS_FLAG_PHASE_ACTIVE       UINT8_C(0x04)
#define DUAL_IMU_STATUS_FLAG_EVENT_WAITING      UINT8_C(0x08)

typedef enum
{
    IMU_BOOT_INIT = 0,
    /* SCBP-V3 BOOT_READY payload remains the frozen value 1. */
    WAIT_SYNC,
    /* Source compatibility for callers that used the old local name. */
    WAIT_RADAR_ZERO = WAIT_SYNC,
    STATIC_CAL_WAIT,
    STATIC_CAL_SAMPLE,
    STATIC_CAL_DONE,
    WAIT_RADAR_LEVEL,
    VIBRATION_SAMPLE,
    VIBRATION_LEVEL_DONE,
    VIBRATION_ALL_DONE,
    FILTER_READY,
    IMU_READY,
    IMU_ERROR,
    /* Transient local state; appended to preserve existing state values. */
    SYNCED,
    /* Calibration transport recovery; waits for a fresh zero-PWM sync. */
    CAL_SYNC_RECOVERY
} imu_boot_state_t;

typedef void (*imu_boot_transport_callback_t)(uint8_t type,
                                              const uint8_t *payload,
                                              uint16_t length);

typedef struct
{
    imu_boot_state_t state;
    imu_phase_t phase;
    uint8_t progress;
    uint8_t lsm_progress;
    uint8_t bmi_progress;
    uint8_t radar_pwm;
    uint32_t sample_count;
    uint32_t sample_total;
    float rms;
    uint8_t error;
    const char *error_reason;
} imu_boot_status_t;

void imu_boot_manager_init(void);
/* Call only during initial boot or an explicit user-requested recalibration. */
void imu_boot_manager_reset(void);
void imu_boot_manager_set_transport(imu_boot_transport_callback_t callback);
void imu_boot_manager_step(void);
void imu_boot_manager_update(const imu_raw_data_t *raw_data);
void imu_boot_manager_on_radar_pwm_ready(uint8_t speed);
void imu_boot_manager_on_cal_event_ack(uint8_t event_id, uint8_t result);

imu_boot_state_t imu_boot_manager_get_state(void);
void imu_boot_manager_get_status(imu_boot_status_t *status);
void imu_boot_manager_get_dual_status(dual_imu_manager_t *status);
uint8_t imu_boot_manager_get_phase_timing(imu_phase_t phase,
                                          imu_phase_timing_t *timing);
uint8_t imu_boot_manager_is_ready(void);
uint8_t imu_boot_manager_is_error(void);
uint8_t imu_boot_manager_get_progress(void);
uint8_t imu_boot_manager_get_radar_pwm(void);
uint32_t imu_boot_manager_get_sample_count(void);
uint32_t imu_boot_manager_get_sample_total(void);
float imu_boot_manager_get_rms(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_BOOT_MANAGER_H */
