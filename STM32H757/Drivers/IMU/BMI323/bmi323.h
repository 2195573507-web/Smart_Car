#ifndef BMI323_H
#define BMI323_H

#include <stdint.h>

#include "bsp_status.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef IMU_VECTOR3F_DEFINED
#define IMU_VECTOR3F_DEFINED
typedef struct
{
    float x;
    float y;
    float z;
} Vector3f;
#endif

#define BMI323_CHIP_ID_VALUE UINT16_C(0x0043)

/* Output units are m/s^2, rad/s, and degrees C respectively. */
bsp_status_t bmi323_init(void);
bsp_status_t bmi323_init_diag(void);
bsp_status_t bmi323_get_chip_id(uint8_t *chip_id);
bsp_status_t bmi323_read_acc(Vector3f *acc);
bsp_status_t bmi323_read_gyro(Vector3f *gyro);
bsp_status_t bmi323_read_temperature(float *temperature);
uint8_t bmi323_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* BMI323_H */
