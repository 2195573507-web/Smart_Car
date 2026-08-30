#ifndef SMARTCAR_SERVICE_H
#define SMARTCAR_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef bool (*smartcar_service_telemetry_sink_t)(
    uint16_t message_id,
    const uint8_t *encoded_frame,
    uint16_t encoded_length,
    uint32_t ingress_timestamp_ms,
    void *context);

/* Register before smartcar_service_init(); the callback must only copy or
 * enqueue the complete SRPv4 frame and must not perform blocking I/O. */
esp_err_t smartcar_service_set_telemetry_sink(
    smartcar_service_telemetry_sink_t sink,
    void *context);

esp_err_t smartcar_service_init(void);

#endif /* SMARTCAR_SERVICE_H */
