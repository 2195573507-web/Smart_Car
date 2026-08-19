#ifndef S3_SERVICE_H
#define S3_SERVICE_H

#include <stdint.h>

void s3_service_init(void);
void s3_service_step(void);
void s3_service_task(void *argument);
void s3_service_start(void);

/* Compatibility transport callback used by the STM IMU boot manager. The
 * implementation serializes the legacy event types as SCBP-V3 MSG_ID values. */
void s3_service_send_boot_frame(uint8_t type, const uint8_t *payload,
                                uint16_t length);

/* Binary IMU/radar diagnostics share the SCBP-V3 STM-S3 frame transport. */
void s3_service_send_telemetry_frame(uint8_t type, const uint8_t *payload,
                                     uint16_t length);

/* Telemetry-only schema=2 dual attitude producer. */
void s3_service_send_dual_attitude(const uint8_t *payload, uint16_t length);

#endif /* S3_SERVICE_H */
