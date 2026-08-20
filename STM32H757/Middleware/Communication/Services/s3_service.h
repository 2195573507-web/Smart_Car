#ifndef S3_SERVICE_H
#define S3_SERVICE_H

#include <stdint.h>

#include "scbp_protocol_defs.h"

void s3_service_init(void);
void s3_service_step(void);
void s3_service_task(void *argument);
void s3_service_start(void);

int s3_service_send_message(uint8_t priority, uint16_t message_id, uint8_t flags,
                            const uint8_t *payload, uint8_t length);
void s3_service_send_boot_message(uint16_t message_id, uint8_t flags,
                                  const uint8_t *payload, uint8_t length);
void s3_service_send_imu_telemetry(const uint8_t *payload, uint8_t length);
void s3_service_send_dual_attitude(const uint8_t *payload, uint8_t length);
int s3_service_send_log(const uint8_t *payload, uint8_t length);

#endif /* S3_SERVICE_H */
