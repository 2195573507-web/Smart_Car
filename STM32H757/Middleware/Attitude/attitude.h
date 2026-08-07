#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} attitude_quaternion_t;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    attitude_quaternion_t quaternion;
} attitude_state_t;

typedef enum
{
    AHRS_WAIT_CAL = 0,
    AHRS_READY
} ahrs_state_t;

#define ATTITUDE_ZERO_SAMPLE_COUNT 500U

void attitude_init(void);
void attitude_zero_init(void);
uint8_t attitude_zero_is_ready(void);
void attitude_update(void);
attitude_state_t attitude_get_state(void);
ahrs_state_t attitude_get_status(void);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_H */
