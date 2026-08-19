#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include <stdint.h>

#include "bsp_status.h"
#include "BMI323/bmi323.h"
#include "imu_calibration.h"
#include "imu_leveling.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    uint32_t timestamp;
    uint64_t timestamp_us;
    uint32_t sample_count;
    uint32_t invalid_count;
    uint8_t valid;
    uint8_t accel_valid;
    uint8_t gyro_valid;
} bmi323_data_t;

#define BMI_RING_BUFFER_SIZE UINT16_C(512)

typedef struct
{
    uint64_t timestamp_us;
    int16_t accel[3];
    int16_t gyro[3];
    uint8_t valid;
} bmi323_raw_sample_t;

typedef struct
{
    bmi323_raw_sample_t buffer[BMI_RING_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    uint32_t overflow_count;
    uint64_t last_capture_us;
} bmi323_ring_buffer_t;

typedef struct
{
    uint32_t sample_count;
    uint32_t overflow_count;
    uint32_t last_timestamp;
    uint64_t first_timestamp_us;
    uint64_t last_timestamp_us;
    uint32_t max_latency_us;
    uint32_t read_fail_count;
    uint32_t contention_drop_count;
    uint16_t pending_count;
    uint16_t configured_rate_hz;
    uint16_t measured_rate_hz;
} bmi323_capture_stat_t;

typedef struct
{
    float ax;
    float ay;
    float az;
    uint32_t timestamp;
    uint64_t timestamp_us;
} lsm_accel_data_t;

typedef struct
{
    float mx;
    float my;
    float mz;
    uint32_t timestamp;
    uint64_t timestamp_us;
} lsm_mag_data_t;

typedef struct
{
    uint32_t update_count;
    uint32_t read_calls;
    uint32_t last_update_ms;
    bsp_status_t last_status;
} imu_sensor_stats_t;

typedef struct
{
    uint8_t lsm_started;
    uint8_t bmi_started;
    uint8_t lsm_complete;
    uint8_t bmi_complete;
    uint8_t lsm_success;
    uint8_t bmi_success;
    uint32_t lsm_start_time;
    uint32_t bmi_start_time;
    uint32_t lsm_end_time;
    uint32_t bmi_end_time;
} imu_dual_init_status_t;

bsp_status_t imu_init(void);
bsp_status_t imu_recover(void);
bsp_status_t imu_update(void);

/* The boot manager starts these two independent-bus workers together during
 * DUAL_IMU_BOOT/INIT. Their completion is polled as one lifecycle barrier. */
bsp_status_t imu_manager_start_dual_initialization(void);
void imu_manager_get_dual_initialization_status(imu_dual_init_status_t *status);
uint8_t imu_manager_finalize_dual_initialization(void);

/* Leveling states are committed exactly once when the static window closes.
 * The runtime paths only read their frozen matrices. */
void imu_manager_reset_leveling(void);
void imu_manager_commit_leveling(void);
void imu_manager_get_leveling_states(imu_leveling_state_t *bmi,
                                     imu_leveling_state_t *lsm);

bsp_status_t imu_get_bmi323_data(bmi323_data_t *data);
bsp_status_t imu_manager_set_bmi323_sample_rate(bmi323_sample_rate_t sample_rate);
bmi323_sample_rate_t imu_manager_get_bmi323_sample_rate(void);
bsp_status_t imu_manager_get_bmi323_capture_stats(bmi323_capture_stat_t *stats);
bsp_status_t imu_manager_start_bmi323_task(void);
bsp_status_t imu_manager_get_lsm_accel(lsm_accel_data_t *data);
bsp_status_t imu_manager_get_lsm_mag(lsm_mag_data_t *data);
bsp_status_t imu_manager_get_snapshot(imu_raw_data_t *snapshot);
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
