#ifndef S3_SERVICE_H
#define S3_SERVICE_H

#include <stdint.h>

void s3_service_init(void);
void s3_service_step(void);
void s3_service_task(void *argument);
void s3_service_start(void);

/* Transport callback used by the STM IMU boot manager. */
void s3_service_send_boot_frame(uint8_t type, const uint8_t *payload,
                                uint16_t length);

#endif /* S3_SERVICE_H */
