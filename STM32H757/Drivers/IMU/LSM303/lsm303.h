#ifndef LSM303_H
#define LSM303_H

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

/* LSM303DLHC-compatible module addresses (7-bit I2C). */
#define LSM303_ACCEL_ADDRESS_DEFAULT UINT8_C(0x19)
#define LSM303_MAG_ADDRESS_DEFAULT   UINT8_C(0x1E)

/* Output units are m/s^2 and microtesla. */
bsp_status_t lsm303_init(void);
bsp_status_t lsm303_init_diag(void);
bsp_status_t lsm303_get_accel_id(uint8_t *id);
bsp_status_t lsm303_get_mag_id(uint8_t id[3]);
bsp_status_t lsm303_read_acc(Vector3f *acc);
bsp_status_t lsm303_read_mag(Vector3f *mag);
uint8_t lsm303_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* LSM303_H */
