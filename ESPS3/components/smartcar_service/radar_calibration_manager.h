#ifndef RADAR_CALIBRATION_MANAGER_H
#define RADAR_CALIBRATION_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#include "scbp_link.h"

typedef enum {
    RADAR_WAIT_SYNC = 0,
    RADAR_SET_PWM,
    RADAR_WAIT_ACK,
    RADAR_WAIT_STATIC_EVENT,
    RADAR_CAL_DONE,
    RADAR_CAL_ERROR
} radar_calibration_state_t;

typedef int (*radar_calibration_send_ready_t)(uint8_t speed_percent, void *context);

void radar_calibration_manager_init(void);
void radar_calibration_manager_set_transport(radar_calibration_send_ready_t send_ready,
                                             void *context);
void radar_calibration_manager_step(void);
bool radar_calibration_manager_on_boot_ready(const uint8_t *payload, uint8_t length);
bool radar_calibration_manager_on_cal_event(const uint8_t *payload, uint8_t length);
void radar_calibration_manager_on_ready_response(scbp_link_tx_result_t result,
                                                 uint8_t status_code);
radar_calibration_state_t radar_calibration_manager_get_state(void);
uint8_t radar_calibration_manager_get_pwm(void);
bool radar_calibration_manager_is_done(void);

#endif /* RADAR_CALIBRATION_MANAGER_H */
