#ifndef MOTOR_BOARD_TASK_H
#define MOTOR_BOARD_TASK_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void motor_board_task_start(void);
void motor_board_task(void *argument);
bool motor_board_set_target_wheel_speeds(const float speeds[4]);
bool motor_board_update_pid_params(float kp, float ki, float kd,
                                   float max_accel);
bool motor_board_force_stop(void);
void motor_board_get_actual_wheel_speeds(float speeds[4]);
bool motor_board_get_battery_voltage(float *voltage);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_BOARD_TASK_H */
