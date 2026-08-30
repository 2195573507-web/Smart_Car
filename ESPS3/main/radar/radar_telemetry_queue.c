#include "radar_telemetry_queue.h"

#include <math.h>
#include <string.h>

#include "srp_codec.h"
#include "srp_wire.h"

enum {
    RADAR_TELEMETRY_OBSERVATION_ATTITUDE = 0,
    RADAR_TELEMETRY_OBSERVATION_IMU_LSM303 = 1,
    RADAR_TELEMETRY_OBSERVATION_IMU_BMI323 = 2,
    RADAR_TELEMETRY_OBSERVATION_COUNT = 3
};

static radar_telemetry_queue_stream_stats_t *stream_stats(
    radar_telemetry_queue_t *queue,
    radar_telemetry_queue_class_t stream)
{
    if (queue == NULL) {
        return NULL;
    }

    switch (stream) {
    case RADAR_TELEMETRY_QUEUE_CLASS_WHEEL:
        return &queue->stats.wheel;
    case RADAR_TELEMETRY_QUEUE_CLASS_ATTITUDE:
        return &queue->stats.attitude;
    case RADAR_TELEMETRY_QUEUE_CLASS_IMU_LSM303:
        return &queue->stats.imu_lsm303;
    case RADAR_TELEMETRY_QUEUE_CLASS_IMU_BMI323:
        return &queue->stats.imu_bmi323;
    default:
        return NULL;
    }
}

static void refresh_depth_stats(radar_telemetry_queue_t *queue)
{
    size_t observation_depth = 0U;

    if (queue == NULL) {
        return;
    }

    for (size_t index = 0U; index < RADAR_TELEMETRY_OBSERVATION_COUNT;
         ++index) {
        if ((index == RADAR_TELEMETRY_OBSERVATION_ATTITUDE &&
             queue->attitude_pending) ||
            (index == RADAR_TELEMETRY_OBSERVATION_IMU_LSM303 &&
             queue->imu_pending[0]) ||
            (index == RADAR_TELEMETRY_OBSERVATION_IMU_BMI323 &&
             queue->imu_pending[1])) {
            ++observation_depth;
        }
    }

    queue->stats.wheel.depth = queue->wheel_count;
    queue->stats.attitude.depth = queue->attitude_pending ? 1U : 0U;
    queue->stats.imu_lsm303.depth = queue->imu_pending[0] ? 1U : 0U;
    queue->stats.imu_bmi323.depth = queue->imu_pending[1] ? 1U : 0U;
    queue->stats.depth = queue->wheel_count + observation_depth;

    if (queue->stats.wheel.depth > queue->stats.wheel.high_watermark) {
        queue->stats.wheel.high_watermark = queue->stats.wheel.depth;
    }
    if (queue->stats.attitude.depth > queue->stats.attitude.high_watermark) {
        queue->stats.attitude.high_watermark = queue->stats.attitude.depth;
    }
    if (queue->stats.imu_lsm303.depth >
        queue->stats.imu_lsm303.high_watermark) {
        queue->stats.imu_lsm303.high_watermark =
            queue->stats.imu_lsm303.depth;
    }
    if (queue->stats.imu_bmi323.depth >
        queue->stats.imu_bmi323.high_watermark) {
        queue->stats.imu_bmi323.high_watermark =
            queue->stats.imu_bmi323.depth;
    }
    if (queue->stats.depth > queue->stats.high_watermark) {
        queue->stats.high_watermark = queue->stats.depth;
    }
}

static bool finite_f32_array(const uint8_t *data,
                             size_t offset,
                             size_t count)
{
    if (data == NULL) {
        return false;
    }

    for (size_t index = 0U; index < count; ++index) {
        if (!isfinite(srp_wire_read_f32_le(
                &data[offset + index * sizeof(float)]))) {
            return false;
        }
    }
    return true;
}

static bool validate_payload(const srp_frame_t *frame,
                             uint16_t message_id,
                             radar_telemetry_queue_class_t *stream)
{
    if (frame == NULL || frame->payload == NULL || stream == NULL) {
        return false;
    }

    switch (message_id) {
    case SRP_MSG_ID_WHEEL_SPEED_STATUS:
        if (frame->length != SRP_PAYLOAD_WHEEL_SPEED_STATUS_SIZE ||
            !finite_f32_array(frame->payload, 0U, 4U)) {
            return false;
        }
        *stream = RADAR_TELEMETRY_QUEUE_CLASS_WHEEL;
        return true;

    case SRP_MSG_ID_ATTITUDE:
        if (frame->length != SRP_PAYLOAD_DUAL_AHRS_SIZE ||
            frame->payload[0] != SRP_DUAL_AHRS_SCHEMA ||
            frame->payload[2] != 0U || frame->payload[3] != 0U ||
            !finite_f32_array(frame->payload, 12U, 17U)) {
            return false;
        }
        *stream = RADAR_TELEMETRY_QUEUE_CLASS_ATTITUDE;
        return true;

    case SRP_MSG_ID_IMU_TELEMETRY:
        if (frame->length != SRP_PAYLOAD_IMU_TELEMETRY_SIZE ||
            !finite_f32_array(frame->payload, 6U, 6U)) {
            return false;
        }
        if (frame->payload[0] == SRP_IMU_SENSOR_LSM303) {
            *stream = RADAR_TELEMETRY_QUEUE_CLASS_IMU_LSM303;
            return true;
        }
        if (frame->payload[0] == SRP_IMU_SENSOR_BMI323) {
            *stream = RADAR_TELEMETRY_QUEUE_CLASS_IMU_BMI323;
            return true;
        }
        return false;

    default:
        return false;
    }
}

static bool decode_and_validate(const uint8_t *encoded_frame,
                                size_t encoded_length,
                                uint16_t message_id,
                                srp_frame_t *decoded,
                                radar_telemetry_queue_class_t *stream)
{
    if (encoded_frame == NULL || decoded == NULL || stream == NULL ||
        encoded_length < (size_t)SRP_HEADER_SIZE + SRP_TRAILER_SIZE ||
        encoded_length > RADAR_TELEMETRY_QUEUE_MAX_FRAME_SIZE ||
        srp_decode(encoded_frame, encoded_length, decoded) != 0) {
        return false;
    }

    if (decoded->type != message_id || decoded->flags != SRP_FLAG_STREAM_DATA ||
        (message_id == SRP_MSG_ID_ATTITUDE
             ? (decoded->priority != SRP_PRIORITY_COMMAND &&
                decoded->priority != SRP_PRIORITY_TELEMETRY)
             : decoded->priority != SRP_PRIORITY_TELEMETRY)) {
        return false;
    }

    return validate_payload(decoded, message_id, stream);
}

static radar_telemetry_entry_t *observation_entry(
    radar_telemetry_queue_t *queue,
    unsigned int observation)
{
    if (queue == NULL || queue->storage.attitude_entry == NULL ||
        queue->storage.imu_entries == NULL) {
        return NULL;
    }

    if (observation == RADAR_TELEMETRY_OBSERVATION_ATTITUDE) {
        return queue->storage.attitude_entry;
    }
    if (observation == RADAR_TELEMETRY_OBSERVATION_IMU_LSM303) {
        return &queue->storage.imu_entries[0];
    }
    if (observation == RADAR_TELEMETRY_OBSERVATION_IMU_BMI323) {
        return &queue->storage.imu_entries[1];
    }
    return NULL;
}

static bool observation_pending(const radar_telemetry_queue_t *queue,
                                unsigned int observation)
{
    if (queue == NULL) {
        return false;
    }
    if (observation == RADAR_TELEMETRY_OBSERVATION_ATTITUDE) {
        return queue->attitude_pending;
    }
    if (observation == RADAR_TELEMETRY_OBSERVATION_IMU_LSM303) {
        return queue->imu_pending[0];
    }
    if (observation == RADAR_TELEMETRY_OBSERVATION_IMU_BMI323) {
        return queue->imu_pending[1];
    }
    return false;
}

static void clear_observation_pending(radar_telemetry_queue_t *queue,
                                       unsigned int observation)
{
    if (queue == NULL) {
        return;
    }
    if (observation == RADAR_TELEMETRY_OBSERVATION_ATTITUDE) {
        queue->attitude_pending = false;
    } else if (observation == RADAR_TELEMETRY_OBSERVATION_IMU_LSM303) {
        queue->imu_pending[0] = false;
    } else if (observation == RADAR_TELEMETRY_OBSERVATION_IMU_BMI323) {
        queue->imu_pending[1] = false;
    }
}

static bool pop_observation(radar_telemetry_queue_t *queue,
                            radar_telemetry_entry_t *entry)
{
    if (queue == NULL || entry == NULL) {
        return false;
    }

    for (unsigned int offset = 0U;
         offset < RADAR_TELEMETRY_OBSERVATION_COUNT;
         ++offset) {
        const unsigned int observation =
            (unsigned int)((queue->next_observation + offset) %
                           RADAR_TELEMETRY_OBSERVATION_COUNT);
        if (!observation_pending(queue, observation)) {
            continue;
        }

        radar_telemetry_entry_t *source = observation_entry(queue, observation);
        if (source == NULL) {
            return false;
        }
        *entry = *source;
        clear_observation_pending(queue, observation);
        queue->next_observation = (uint8_t)((observation + 1U) %
                                            RADAR_TELEMETRY_OBSERVATION_COUNT);
        queue->wheel_burst = 0U;
        refresh_depth_stats(queue);
        return true;
    }
    return false;
}

static bool pop_wheel(radar_telemetry_queue_t *queue,
                      radar_telemetry_entry_t *entry)
{
    if (queue == NULL || entry == NULL || queue->wheel_count == 0U ||
        queue->storage.wheel_entries == NULL ||
        queue->storage.wheel_capacity == 0U) {
        return false;
    }

    *entry = queue->storage.wheel_entries[queue->wheel_tail];
    queue->wheel_tail = (queue->wheel_tail + 1U) %
                        queue->storage.wheel_capacity;
    --queue->wheel_count;
    if (queue->wheel_burst < UINT8_MAX) {
        ++queue->wheel_burst;
    }
    refresh_depth_stats(queue);
    return true;
}

bool radar_telemetry_queue_init(
    radar_telemetry_queue_t *queue,
    const radar_telemetry_queue_storage_t *storage)
{
    if (queue == NULL || storage == NULL || storage->wheel_entries == NULL ||
        storage->wheel_capacity == 0U || storage->attitude_entry == NULL ||
        storage->imu_entries == NULL) {
        return false;
    }

    (void)memset(queue, 0, sizeof(*queue));
    queue->storage = *storage;
    queue->initialized = true;
    refresh_depth_stats(queue);
    return true;
}

bool radar_telemetry_queue_push(
    radar_telemetry_queue_t *queue,
    uint16_t message_id,
    const uint8_t *encoded_frame,
    size_t encoded_length,
    uint32_t ingress_timestamp_ms)
{
    srp_frame_t decoded;
    radar_telemetry_queue_class_t stream;

    if (queue == NULL || !queue->initialized ||
        !decode_and_validate(encoded_frame, encoded_length, message_id,
                             &decoded, &stream)) {
        if (queue != NULL && queue->initialized) {
            ++queue->stats.rejected;
        }
        return false;
    }

    radar_telemetry_queue_stream_stats_t *stream_counter =
        stream_stats(queue, stream);
    if (stream_counter == NULL) {
        ++queue->stats.rejected;
        return false;
    }

    if (stream == RADAR_TELEMETRY_QUEUE_CLASS_WHEEL) {
        if (queue->wheel_count >= queue->storage.wheel_capacity) {
            ++stream_counter->dropped;
            return false;
        }

        radar_telemetry_entry_t *destination =
            &queue->storage.wheel_entries[queue->wheel_head];
        (void)memmove(destination->data, encoded_frame, encoded_length);
        destination->length = (uint16_t)encoded_length;
        destination->message_id = message_id;
        destination->ingress_timestamp_ms = ingress_timestamp_ms;
        queue->wheel_head = (queue->wheel_head + 1U) %
                            queue->storage.wheel_capacity;
        ++queue->wheel_count;
        ++stream_counter->accepted;
        refresh_depth_stats(queue);
        return true;
    }

    const unsigned int observation =
        stream == RADAR_TELEMETRY_QUEUE_CLASS_ATTITUDE
            ? RADAR_TELEMETRY_OBSERVATION_ATTITUDE
            : stream == RADAR_TELEMETRY_QUEUE_CLASS_IMU_LSM303
                  ? RADAR_TELEMETRY_OBSERVATION_IMU_LSM303
                  : RADAR_TELEMETRY_OBSERVATION_IMU_BMI323;
    radar_telemetry_entry_t *destination = observation_entry(queue, observation);
    if (destination == NULL) {
        ++queue->stats.rejected;
        return false;
    }
    if (observation_pending(queue, observation)) {
        ++stream_counter->overwritten;
    }
    (void)memmove(destination->data, encoded_frame, encoded_length);
    destination->length = (uint16_t)encoded_length;
    destination->message_id = message_id;
    destination->ingress_timestamp_ms = ingress_timestamp_ms;
    if (observation == RADAR_TELEMETRY_OBSERVATION_ATTITUDE) {
        queue->attitude_pending = true;
    } else if (observation == RADAR_TELEMETRY_OBSERVATION_IMU_LSM303) {
        queue->imu_pending[0] = true;
    } else {
        queue->imu_pending[1] = true;
    }
    ++stream_counter->accepted;
    refresh_depth_stats(queue);
    return true;
}

bool radar_telemetry_queue_pop(radar_telemetry_queue_t *queue,
                               radar_telemetry_entry_t *entry)
{
    if (queue == NULL || entry == NULL || !queue->initialized) {
        return false;
    }

    const bool observations_waiting =
        queue->attitude_pending || queue->imu_pending[0] ||
        queue->imu_pending[1];
    if (queue->wheel_count != 0U &&
        (!observations_waiting ||
         queue->wheel_burst < RADAR_TELEMETRY_QUEUE_WHEEL_BURST_MAX)) {
        return pop_wheel(queue, entry);
    }
    if (observations_waiting && pop_observation(queue, entry)) {
        return true;
    }
    return pop_wheel(queue, entry);
}

bool radar_telemetry_queue_has_pending(
    const radar_telemetry_queue_t *queue)
{
    return queue != NULL && queue->initialized &&
           (queue->wheel_count != 0U || queue->attitude_pending ||
            queue->imu_pending[0] || queue->imu_pending[1]);
}

void radar_telemetry_queue_get_stats(
    const radar_telemetry_queue_t *queue,
    radar_telemetry_queue_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    (void)memset(stats, 0, sizeof(*stats));
    if (queue != NULL && queue->initialized) {
        *stats = queue->stats;
    }
}
