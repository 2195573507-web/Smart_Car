#ifndef ATTITUDE_STARTUP_COORDINATOR_H
#define ATTITUDE_STARTUP_COORDINATOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Published only after the existing IMU lifecycle and attitude zero gate have
 * both completed and the estimator has been observed running. */
extern volatile uint8_t g_attitude_is_ready;

void attitude_startup_coordinator_start(void);
void attitude_startup_coordinator_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_STARTUP_COORDINATOR_H */
