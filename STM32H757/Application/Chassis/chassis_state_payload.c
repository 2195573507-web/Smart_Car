#include "chassis_state_payload.h"

#include <math.h>
#include <string.h>

#include "srp_registry.h"
#include "srp_wire.h"

/* CHASSIS_STATE 纯 payload/序号门实现；创建人：待确认（当前维护人：Zhiqin）。 */

#define CHASSIS_STATE_RAD_TO_DEG 57.2957795130823208768f

bool chassis_state_sequence_is_new(uint32_t sequence,
                                   uint32_t last_sequence,
                                   bool have_last_sequence)
{
    return !have_last_sequence || sequence != last_sequence;
}

bool chassis_state_pack_payload(
    uint8_t *payload,
    size_t capacity,
    const chassis_odometry_state_t *state,
    uint8_t flags,
    uint32_t timestamp_ms)
{
    const float yaw_deg = state == NULL ? 0.0f :
        state->yaw_rad * CHASSIS_STATE_RAD_TO_DEG;

    if (payload == NULL || capacity < SRP_PAYLOAD_CHASSIS_STATE_SIZE ||
        state == NULL || !isfinite(state->x_mm) || !isfinite(state->y_mm) ||
        !isfinite(state->yaw_rad) || !isfinite(yaw_deg) ||
        !isfinite(state->total_distance_m) || state->total_distance_m < 0.0f) {
        return false;
    }

    (void)memset(payload, 0, SRP_PAYLOAD_CHASSIS_STATE_SIZE);
    payload[0] = SRP_CHASSIS_STATE_SCHEMA;
    payload[1] = flags & SRP_CHASSIS_STATE_FLAGS_MASK;
    srp_wire_write_u32_le(&payload[4], timestamp_ms);
    srp_wire_write_f32_le(&payload[8], state->x_mm);
    srp_wire_write_f32_le(&payload[12], state->y_mm);
    srp_wire_write_f32_le(&payload[16], yaw_deg);
    srp_wire_write_f32_le(&payload[20], state->total_distance_m);
    return true;
}
