#ifndef ROS_MOTION_GATEWAY_H
#define ROS_MOTION_GATEWAY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "ros_motion_protocol.h"
#include "ros_motion_state.h"

#define ROS_MOTION_TCP_PORT UINT16_C(8766)
#define ROS_MOTION_WATCHDOG_PERIOD_MS UINT32_C(20)
#define ROS_MOTION_DEFAULT_LINEAR_LIMIT_MM_S INT32_C(100)
#define ROS_MOTION_DEFAULT_ANGULAR_LIMIT_MRAD_S INT32_C(300)

/* Must run before smartcar_service_init(); binds only bounded service callbacks. */
esp_err_t ros_motion_gateway_bind_service(void);

/* Starts Wi-Fi/TCP only after smartcar_service_init() has created its owner task. */
esp_err_t ros_motion_gateway_init(void);
bool ros_motion_gateway_is_running(void);

/* Service-owner callback: copies only validated SRP frames into the gateway slot. */
bool ros_motion_gateway_telemetry_sink(uint16_t message_id,
                                       const uint8_t *encoded_frame,
                                       uint16_t encoded_length,
                                       uint32_t ingress_timestamp_ms,
                                       void *context);

/* Invoked only by the command bridge service task after local safety revocation. */
void ros_motion_gateway_on_safety_stop(void *context);

#endif /* ROS_MOTION_GATEWAY_H */
