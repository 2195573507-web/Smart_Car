#ifndef RADAR_CONTROL_H
#define RADAR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define RADAR_MIN_SPEED 0U
#define RADAR_MAX_SPEED 100U

typedef enum {
    RADAR_CONTROL_WAIT_STM_QUERY = 0,
    RADAR_CONTROL_WAIT_IMU_CAL,
    RADAR_CONTROL_SOFT_START,
    RADAR_CONTROL_RUNNING,
} radar_control_state_t;

void radar_control_init(void);

void radar_control_set_speed(uint8_t percent);

void radar_control_set_imu_cal_done(bool done);

/* Accepts the STM32 startup query and releases the IMU-calibration gate. */
void radar_control_handle_pwm_ready_query(void);

/* Calibration PWM is an explicit STM32-owned override. It never advances the
 * normal soft-start state machine or selects the next scan level on S3. */
/* Applies both LEDC operations and reports whether the update succeeded. */
bool radar_control_set_calibration_pwm(uint8_t percent);
void radar_control_release_calibration_lock(void);
bool radar_control_is_calibration_active(void);

bool radar_control_is_running(void);

radar_control_state_t radar_control_get_state(void);

uint8_t radar_control_get_speed(void);

/* Returns the PWM most recently applied by the calibration override. */
uint8_t radar_control_get_calibration_pwm(void);

#endif /* RADAR_CONTROL_H */
