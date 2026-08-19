#ifndef RADAR_CONTROL_H
#define RADAR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define RADAR_MIN_SPEED 0U
#define RADAR_MAX_SPEED 100U

typedef enum {
    /* BOOT: wait for the accepted STM startup query. */
    RADAR_CONTROL_WAIT_STM_QUERY = 0,
    /* WAIT_CAL: only the STM-owned calibration override is allowed. */
    RADAR_CONTROL_WAIT_IMU_CAL,
    /* Reserved legacy value; the current App-controlled path does not enter it. */
    RADAR_CONTROL_SOFT_START,
    RADAR_CONTROL_RUNNING,
    /* CAL_DONE is the completion gate immediately before RUNNING. */
    RADAR_CONTROL_CAL_DONE,
} radar_control_state_t;

void radar_control_init(void);

/* Task context only: this takes the control mutex and performs LEDC writes.
 * Returns true only when the request was accepted and LEDC was updated. */
bool radar_control_set_speed(uint8_t percent);

/* Task context only. Valid completion transitions WAIT_CAL -> CAL_DONE. */
void radar_control_set_imu_cal_done(bool done);

/* Task context only. Accepts the STM32 startup query and releases BOOT. */
void radar_control_handle_pwm_ready_query(void);

/* Calibration PWM is an explicit STM32-owned override. It never selects an
 * App running speed or the next calibration level on S3. */
/* Task context only. Applies both LEDC operations and reports success. */
bool radar_control_set_calibration_pwm(uint8_t percent);
void radar_control_release_calibration_lock(void);
bool radar_control_is_calibration_active(void);

bool radar_control_is_running(void);

radar_control_state_t radar_control_get_state(void);

uint8_t radar_control_get_speed(void);

/* Returns the PWM most recently applied by the calibration override. */
uint8_t radar_control_get_calibration_pwm(void);

#endif /* RADAR_CONTROL_H */
