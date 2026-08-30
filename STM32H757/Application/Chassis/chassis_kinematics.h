#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_TRACK_WIDTH_MM 193.0f
#define CHASSIS_WHEEL_COUNT 4U
#define CHASSIS_WHEEL_SPEED_LIMIT_MM_S 1000.0f

typedef enum {
    CHASSIS_WHEEL_RR = 0U,
    CHASSIS_WHEEL_RF = 1U,
    CHASSIS_WHEEL_LR = 2U,
    CHASSIS_WHEEL_LF = 3U
} chassis_wheel_index_t;

/* Wheel order is RR, RF, LR, LF, matching the MotorBoard protocol. */
bool chassis_kinematics_compute(float linear_mm_s, float angular_rad_s,
                                float wheel_speed[CHASSIS_WHEEL_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_KINEMATICS_H */
