#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <stdint.h>

#include "bsp_status.h"
#include "imu_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    uint32_t timestamp;
    uint8_t online;
} bmi323_data_t;

typedef struct
{
    float ax;
    float ay;
    float az;
} lsm_accel_data_t;

typedef struct
{
    float mx;
    float my;
    float mz;
} lsm_mag_data_t;

typedef struct
{
    uint32_t update_count;
    uint32_t read_calls;
    uint32_t last_update_ms;
    bsp_status_t last_status;
} imu_sensor_stats_t;

bsp_status_t imu_init(void);
bsp_status_t imu_recover(void);
bsp_status_t imu_update(void);

bsp_status_t imu_get_bmi323_data(bmi323_data_t *data);
bsp_status_t imu_manager_get_lsm_accel(lsm_accel_data_t *data);
bsp_status_t imu_manager_get_lsm_mag(lsm_mag_data_t *data);
bsp_status_t imu_get_bmi323_stats(imu_sensor_stats_t *stats);
bsp_status_t imu_get_lsm303_stats(imu_sensor_stats_t *stats);

/* One scheduler iteration, useful for bare-metal integration and tests. */
bsp_status_t imu_task_step(void);

/* FreeRTOS-compatible task entry; uses the BSP tick in the no-RTOS fallback. */
void imu_task(void *argument);

uint8_t imu_is_ready(void);
/* Hardware initialization state is independent from recent data health. */
uint8_t imu_manager_get_lsm303_init_success(void);
uint8_t imu_manager_get_lsm303_online(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_MANAGER_H */
