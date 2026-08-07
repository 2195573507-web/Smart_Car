#ifndef IMU_RUNTIME_H
#define IMU_RUNTIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void imu_runtime_start(void);
void imu_debug_task(void *argument);
uint32_t imu_runtime_get_log_fail_count(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_RUNTIME_H */
