#ifndef ROS_MOTION_STATE_H
#define ROS_MOTION_STATE_H

#include <stdbool.h>
#include <stdint.h>

#define ROS_MOTION_DEFAULT_LEASE_MS UINT16_C(220)
#define ROS_MOTION_MIN_TTL_MS UINT16_C(20)
#define ROS_MOTION_MAX_TTL_MS UINT16_C(220)

typedef enum {
    ROS_MOTION_STATE_DISCONNECTED = 0,
    ROS_MOTION_STATE_HANDSHAKE = 1,
    ROS_MOTION_STATE_READY = 2,
    ROS_MOTION_STATE_LEASED = 3,
    ROS_MOTION_STATE_FAULT = 4
} ros_motion_state_t;

typedef struct {
    float linear_m_s;
    float angular_rad_s;
    uint32_t session_id;
    uint32_t lease_id;
    uint32_t sequence;
    uint16_t ttl_ms;
} ros_motion_command_t;

typedef struct {
    ros_motion_state_t state;
    uint32_t session_id;
    uint32_t lease_id;
    uint32_t last_sequence;
    uint64_t lease_deadline_ms;
    bool have_sequence;
    bool lease_active;
    bool latest_valid;
    ros_motion_command_t latest;
} ros_motion_state_machine_t;

void ros_motion_state_init(ros_motion_state_machine_t *machine);
void ros_motion_state_connected(ros_motion_state_machine_t *machine,
                                 uint32_t session_id);
bool ros_motion_state_hello_ack(ros_motion_state_machine_t *machine,
                                uint32_t session_id, uint32_t sequence);
bool ros_motion_state_request_lease(ros_motion_state_machine_t *machine,
                                    uint32_t session_id, uint32_t lease_id,
                                    uint32_t sequence, uint16_t ttl_ms,
                                    uint64_t now_ms);
bool ros_motion_state_accept_command(ros_motion_state_machine_t *machine,
                                     const ros_motion_command_t *command,
                                     uint64_t now_ms, int32_t linear_limit_mm_s,
                                     int32_t angular_limit_mrad_s);
bool ros_motion_state_accept_stop(ros_motion_state_machine_t *machine,
                                  uint32_t session_id, uint32_t lease_id,
                                  uint32_t sequence, uint64_t now_ms);
bool ros_motion_state_accept_heartbeat(ros_motion_state_machine_t *machine,
                                       uint32_t session_id, uint32_t lease_id,
                                       uint32_t sequence, uint64_t now_ms);
bool ros_motion_state_expired(ros_motion_state_machine_t *machine,
                              uint64_t now_ms);
void ros_motion_state_revoke(ros_motion_state_machine_t *machine);
bool ros_motion_state_take_latest(ros_motion_state_machine_t *machine,
                                   ros_motion_command_t *command);

#endif /* ROS_MOTION_STATE_H */
