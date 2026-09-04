#include "ros_motion_state.h"

#include <math.h>
#include <stddef.h>

static bool sequence_newer(uint32_t candidate, uint32_t previous)
{
    return candidate != previous && (int32_t)(candidate - previous) > 0;
}

static bool identity_ok(ros_motion_state_machine_t *machine,
                        uint32_t session_id, uint32_t lease_id,
                        uint64_t now_ms)
{
    if (!machine->lease_active || machine->session_id != session_id ||
        machine->lease_id != lease_id) {
        return false;
    }
    if (now_ms >= machine->lease_deadline_ms) {
        ros_motion_state_revoke(machine);
        return false;
    }
    return true;
}

static bool sequence_accept(ros_motion_state_machine_t *machine,
                            uint32_t sequence)
{
    if (machine->have_sequence &&
        !sequence_newer(sequence, machine->last_sequence)) {
        return false;
    }
    machine->last_sequence = sequence;
    machine->have_sequence = true;
    return true;
}

void ros_motion_state_init(ros_motion_state_machine_t *machine)
{
    if (machine != NULL) {
        *machine = (ros_motion_state_machine_t){
            .state = ROS_MOTION_STATE_DISCONNECTED
        };
    }
}

void ros_motion_state_connected(ros_motion_state_machine_t *machine,
                                uint32_t session_id)
{
    if (machine == NULL) {
        return;
    }
    *machine = (ros_motion_state_machine_t){
        .state = session_id == 0U ? ROS_MOTION_STATE_DISCONNECTED
                                  : ROS_MOTION_STATE_HANDSHAKE,
        .session_id = session_id
    };
}

bool ros_motion_state_hello_ack(ros_motion_state_machine_t *machine,
                                uint32_t session_id, uint32_t sequence)
{
    if (machine == NULL || machine->state != ROS_MOTION_STATE_HANDSHAKE ||
        machine->session_id != session_id || sequence == 0U ||
        !sequence_accept(machine, sequence)) {
        return false;
    }
    machine->state = ROS_MOTION_STATE_READY;
    return true;
}

bool ros_motion_state_request_lease(ros_motion_state_machine_t *machine,
                                    uint32_t session_id, uint32_t lease_id,
                                    uint32_t sequence, uint16_t ttl_ms,
                                    uint64_t now_ms)
{
    if (machine == NULL || machine->state != ROS_MOTION_STATE_READY ||
        machine->session_id != session_id || lease_id == 0U ||
        ttl_ms < ROS_MOTION_MIN_TTL_MS || ttl_ms > ROS_MOTION_MAX_TTL_MS ||
        !sequence_accept(machine, sequence)) {
        return false;
    }
    machine->lease_id = lease_id;
    machine->lease_deadline_ms = now_ms + ttl_ms;
    machine->lease_active = true;
    machine->state = ROS_MOTION_STATE_LEASED;
    return true;
}

bool ros_motion_state_accept_command(ros_motion_state_machine_t *machine,
                                     const ros_motion_command_t *command,
                                     uint64_t now_ms, int32_t linear_limit_mm_s,
                                     int32_t angular_limit_mrad_s)
{
    if (machine == NULL || command == NULL ||
        !identity_ok(machine, command->session_id, command->lease_id, now_ms) ||
        command->ttl_ms < ROS_MOTION_MIN_TTL_MS ||
        command->ttl_ms > ROS_MOTION_MAX_TTL_MS ||
        !isfinite(command->linear_m_s) || !isfinite(command->angular_rad_s) ||
        fabsf(command->linear_m_s) * 1000.0f > (float)linear_limit_mm_s ||
        fabsf(command->angular_rad_s) >
            (float)angular_limit_mrad_s / 1000.0f ||
        !sequence_accept(machine, command->sequence)) {
        return false;
    }
    machine->latest = *command;
    machine->latest_valid = true;
    machine->lease_deadline_ms = now_ms + command->ttl_ms;
    return true;
}

bool ros_motion_state_accept_stop(ros_motion_state_machine_t *machine,
                                  uint32_t session_id, uint32_t lease_id,
                                  uint32_t sequence, uint64_t now_ms)
{
    if (machine == NULL || !identity_ok(machine, session_id, lease_id, now_ms) ||
        !sequence_accept(machine, sequence)) {
        return false;
    }
    machine->latest_valid = false;
    machine->lease_deadline_ms = now_ms;
    ros_motion_state_revoke(machine);
    return true;
}

bool ros_motion_state_accept_heartbeat(ros_motion_state_machine_t *machine,
                                       uint32_t session_id, uint32_t lease_id,
                                       uint32_t sequence, uint64_t now_ms)
{
    if (machine == NULL || !identity_ok(machine, session_id, lease_id, now_ms) ||
        !sequence_accept(machine, sequence)) {
        return false;
    }
    machine->lease_deadline_ms = now_ms + ROS_MOTION_DEFAULT_LEASE_MS;
    return true;
}

bool ros_motion_state_expired(ros_motion_state_machine_t *machine,
                              uint64_t now_ms)
{
    if (machine == NULL || !machine->lease_active ||
        now_ms < machine->lease_deadline_ms) {
        return false;
    }
    ros_motion_state_revoke(machine);
    return true;
}

void ros_motion_state_revoke(ros_motion_state_machine_t *machine)
{
    if (machine == NULL) {
        return;
    }
    machine->lease_active = false;
    machine->lease_id = 0U;
    machine->lease_deadline_ms = 0U;
    machine->latest_valid = false;
    machine->state = machine->session_id == 0U
                         ? ROS_MOTION_STATE_DISCONNECTED
                         : ROS_MOTION_STATE_FAULT;
}

bool ros_motion_state_take_latest(ros_motion_state_machine_t *machine,
                                  ros_motion_command_t *command)
{
    if (machine == NULL || command == NULL || !machine->latest_valid) {
        return false;
    }
    *command = machine->latest;
    machine->latest_valid = false;
    return true;
}
