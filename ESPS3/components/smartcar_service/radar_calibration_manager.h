#ifndef RADAR_CALIBRATION_MANAGER_H
#define RADAR_CALIBRATION_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    RADAR_CAL_IDLE = 0,
    RADAR_SET_PWM,
    RADAR_WAIT_ACK,
    RADAR_WAIT_EVENT,
    RADAR_NEXT_LEVEL,
    RADAR_CAL_DONE,
    RADAR_CAL_ERROR
} radar_calibration_state_t;

void radar_calibration_manager_init(void);
void radar_calibration_manager_step(void);
void radar_calibration_manager_on_frame(uint8_t type,
                                        const uint8_t *payload,
                                        uint16_t length);
radar_calibration_state_t radar_calibration_manager_get_state(void);
uint8_t radar_calibration_manager_get_pwm(void);
bool radar_calibration_manager_is_done(void);

#endif /* RADAR_CALIBRATION_MANAGER_H */
