#ifndef IMU_TIME_H
#define IMU_TIME_H

#include <stdint.h>

#include "bsp_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* IMU acquisition timestamps are always monotonic microseconds from the DWT
 * clock. Milliseconds are a compatibility view derived from this same value. */
bsp_status_t imu_time_init(void);
uint64_t imu_time_now_us(void);
uint32_t imu_time_now_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* IMU_TIME_H */
