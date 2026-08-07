#ifndef IMU_BOOT_MANAGER_H
#define IMU_BOOT_MANAGER_H

#include <stdint.h>

#include "bsp_status.h"
#include "imu_calibration.h"
#include "imu_vibration.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    IMU_BOOT_INIT = 0,
    WAIT_RADAR_ZERO,
    STATIC_CAL_WAIT,
    STATIC_CAL_SAMPLE,
    STATIC_CAL_DONE,
    WAIT_RADAR_LEVEL,
    VIBRATION_SAMPLE,
    VIBRATION_LEVEL_DONE,
    VIBRATION_ALL_DONE,
    FILTER_READY,
    IMU_READY,
    IMU_ERROR
} imu_boot_state_t;

typedef void (*imu_boot_transport_callback_t)(uint8_t type,
                                              const uint8_t *payload,
                                              uint16_t length);

typedef struct
{
    imu_boot_state_t state;
    uint8_t progress;
    uint8_t radar_pwm;
    uint32_t sample_count;
    uint32_t sample_total;
    float rms;
    uint8_t error;
    const char *error_reason;
} imu_boot_status_t;

void imu_boot_manager_init(bsp_status_t lsm303_status);
/* Call only during initial boot or an explicit user-requested recalibration. */
void imu_boot_manager_reset(void);
void imu_boot_manager_set_transport(imu_boot_transport_callback_t callback);
void imu_boot_manager_step(void);
void imu_boot_manager_update(const imu_raw_data_t *raw_data);
void imu_boot_manager_on_radar_pwm_ready(uint8_t speed);
void imu_boot_manager_on_cal_event_ack(uint8_t event_id, uint8_t result);

imu_boot_state_t imu_boot_manager_get_state(void);
void imu_boot_manager_get_status(imu_boot_status_t *status);
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
