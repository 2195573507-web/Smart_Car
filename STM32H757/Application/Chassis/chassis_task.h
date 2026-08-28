#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void chassis_task_start(void);
void chassis_task(void *argument);
bool chassis_task_set_velocity(float v_mm_s, float w_rad_s);
void chassis_task_force_stop(void);
void chassis_task_set_heading_target(float v_mm_s, float target_yaw_deg);
/* Optional target yaw-rate feedforward input in rad/s; default is 0.0f. */
void chassis_task_set_heading_feedforward(float w_ff_rad_s);
bool chassis_task_heading_target_is_admissible(float v_mm_s,
                                               float target_yaw_deg);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_TASK_H */
