#ifndef IMU_BOOT_MANAGER_H
#define IMU_BOOT_MANAGER_H

#include <stdint.h>

#include "bsp_status.h"
#include "imu_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

/* DUAL_IMU_BOOT is the only lifecycle authority.  Legacy boot states below
 * remain a read-only compatibility view for existing filter/log consumers. */
typedef enum
{
    IMU_PHASE_IDLE = 0,
    IMU_PHASE_INIT = 1,
    IMU_PHASE_SELF_TEST = 2,
    IMU_PHASE_STATIC_CALIBRATION = 3,
    /* Values 4 and 5 remain reserved for persisted diagnostic readers. */
    IMU_PHASE_READY = 6,
    IMU_PHASE_FAILED = 7,
    IMU_PHASE_COUNT = 8
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
    /* Preserve the historical error byte carried by the status payload. */
    IMU_ERROR_TASK_CREATE = 12
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
    imu_phase_timing_t phase_timing[IMU_PHASE_COUNT];
} dual_imu_manager_t;

#define DUAL_IMU_STATUS_FLAG_LSM_PHASE_COMPLETE UINT8_C(0x01)
#define DUAL_IMU_STATUS_FLAG_BMI_PHASE_COMPLETE UINT8_C(0x02)
#define DUAL_IMU_STATUS_FLAG_PHASE_ACTIVE       UINT8_C(0x04)

typedef enum
{
    IMU_BOOT_INIT = 0,
    /* BOOT_READY payload uses this stable lifecycle value. */
    WAIT_SYNC,
    /* Source compatibility for callers that used the old local name. */
    WAIT_RADAR_ZERO = WAIT_SYNC,
    STATIC_CAL_WAIT,
    STATIC_CAL_SAMPLE,
    STATIC_CAL_DONE,
    /* Values 5..9 remain reserved for compatibility with older receivers. */
    IMU_READY = 10,
    IMU_ERROR = 11,
    /* Transient local state; appended to preserve existing state values. */
    SYNCED,
    /* Calibration transport recovery; waits for a fresh zero-PWM sync. */
    CAL_SYNC_RECOVERY
} imu_boot_state_t;

typedef void (*imu_boot_transport_callback_t)(uint16_t message_id,
                                              uint8_t flags,
                                              const uint8_t *payload,
                                              uint8_t length);

typedef struct
{
    imu_boot_state_t state;
    imu_phase_t phase;
    uint8_t progress;
    uint8_t lsm_progress;
    uint8_t bmi_progress;
    uint32_t sample_count;
    uint32_t sample_total;
    uint8_t error;
    const char *error_reason;
} imu_boot_status_t;

void imu_boot_manager_init(void);
/* Call only during initial boot or an explicit user-requested recalibration. */
void imu_boot_manager_reset(void);
void imu_boot_manager_set_transport(imu_boot_transport_callback_t callback);
void imu_boot_manager_step(void);
void imu_boot_manager_update(const imu_raw_data_t *raw_data);
uint8_t imu_boot_manager_on_radar_pwm_ready(uint8_t speed);

imu_boot_state_t imu_boot_manager_get_state(void);
void imu_boot_manager_get_status(imu_boot_status_t *status);
void imu_boot_manager_get_dual_status(dual_imu_manager_t *status);
uint8_t imu_boot_manager_get_phase_timing(imu_phase_t phase,
                                          imu_phase_timing_t *timing);
uint8_t imu_boot_manager_is_ready(void);
uint8_t imu_boot_manager_is_error(void);
uint8_t imu_boot_manager_get_progress(void);
uint32_t imu_boot_manager_get_sample_count(void);
uint32_t imu_boot_manager_get_sample_total(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_BOOT_MANAGER_H */
