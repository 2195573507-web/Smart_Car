#include <stdio.h>
#include <string.h>

#include "radar_telemetry_queue.h"
#include "srp_crc.h"
#include "srp_codec.h"
#include "srp_wire.h"

#define TEST_ASSERT(condition) do { \
    if (!(condition)) { \
        (void)fprintf(stderr, "assertion failed: %s at %s:%d\n", \
                      #condition, __FILE__, __LINE__); \
        return 1; \
    } \
} while (0)

static int make_frame(uint16_t message_id,
                      uint8_t sequence,
                      uint8_t sensor_id,
                      float marker,
                      uint8_t *wire,
                      uint16_t *wire_length)
{
    uint8_t payload[SRP_MAX_PAYLOAD] = {0};
    uint16_t payload_length = 0U;

    if (wire == NULL || wire_length == NULL) {
        return -1;
    }

    switch (message_id) {
    case SRP_MSG_ID_WHEEL_SPEED_STATUS:
        payload_length = SRP_PAYLOAD_WHEEL_SPEED_STATUS_SIZE;
        for (size_t index = 0U; index < 4U; ++index) {
            srp_wire_write_f32_le(&payload[index * sizeof(float)],
                                   marker + (float)index);
        }
        break;

    case SRP_MSG_ID_ATTITUDE:
        payload_length = SRP_PAYLOAD_DUAL_AHRS_SIZE;
        payload[0] = SRP_DUAL_AHRS_SCHEMA;
        for (size_t index = 0U; index < 17U; ++index) {
            srp_wire_write_f32_le(&payload[12U + index * sizeof(float)],
                                   marker + (float)index);
        }
        break;

    case SRP_MSG_ID_IMU_TELEMETRY:
        payload_length = SRP_PAYLOAD_IMU_TELEMETRY_SIZE;
        payload[0] = sensor_id;
        payload[1] = SRP_IMU_TELEMETRY_FLAG_ONLINE;
        for (size_t index = 0U; index < 6U; ++index) {
            srp_wire_write_f32_le(&payload[6U + index * sizeof(float)],
                                   marker + (float)index);
        }
        break;

    default:
        payload_length = 1U;
        payload[0] = 0xA5U;
        break;
    }

    const srp_frame_t frame = {
        .priority = SRP_PRIORITY_TELEMETRY,
        .type = (uint8_t)message_id,
        .sequence = sequence,
        .flags = SRP_FLAG_STREAM_DATA,
        .length = payload_length,
        .payload = payload,
    };
    return srp_encode(&frame, wire, SRP_MAX_FRAME_SIZE,
                           wire_length);
}

static int init_queue(radar_telemetry_queue_t *queue,
                      radar_telemetry_entry_t *wheel_entries,
                      size_t wheel_capacity,
                      radar_telemetry_entry_t *attitude_entry,
                      radar_telemetry_entry_t *imu_entries)
{
    const radar_telemetry_queue_storage_t storage = {
        .wheel_entries = wheel_entries,
        .wheel_capacity = wheel_capacity,
        .attitude_entry = attitude_entry,
        .imu_entries = imu_entries,
    };
    return radar_telemetry_queue_init(queue, &storage) ? 0 : 1;
}

static int test_maximum_frame_contract(void)
{
    radar_telemetry_queue_t queue;
    radar_telemetry_entry_t wheel[1];
    radar_telemetry_entry_t attitude;
    radar_telemetry_entry_t imu[2];
    uint8_t wire[SRP_MAX_FRAME_SIZE];
    uint8_t payload[SRP_MAX_PAYLOAD] = {0};
    uint16_t wire_length = 0U;
    radar_telemetry_queue_stats_t stats;

    TEST_ASSERT(sizeof(wire) == SRP_MAX_FRAME_SIZE);
    TEST_ASSERT(sizeof(wheel[0].data) == SRP_MAX_FRAME_SIZE);
    TEST_ASSERT(init_queue(&queue, wheel, 1U, &attitude, imu) == 0);

    const srp_frame_t frame = {
        .priority = SRP_PRIORITY_LOG,
        .type = SRP_MSG_ID_LOG,
        .sequence = 0x55U,
        .flags = SRP_FLAG_STREAM_DATA,
        .length = SRP_MAX_PAYLOAD,
        .payload = payload,
    };
    TEST_ASSERT(srp_encode(&frame, wire, sizeof(wire), &wire_length) == 0);
    TEST_ASSERT(wire_length == SRP_HEADER_SIZE + SRP_MAX_PAYLOAD + SRP_TRAILER_SIZE);
    TEST_ASSERT(!radar_telemetry_queue_push(&queue, SRP_MSG_ID_LOG, wire,
                                            wire_length, 10U));
    radar_telemetry_queue_get_stats(&queue, &stats);
    TEST_ASSERT(stats.rejected == 1U);
    TEST_ASSERT(stats.depth == 0U);
    return 0;
}

static int test_wheel_fifo_order_and_full_rejection(void)
{
    radar_telemetry_queue_t queue;
    radar_telemetry_entry_t wheel[2];
    radar_telemetry_entry_t attitude;
    radar_telemetry_entry_t imu[2];
    radar_telemetry_entry_t output;
    uint8_t wire[SRP_MAX_FRAME_SIZE];
    uint16_t wire_length = 0U;
    radar_telemetry_queue_stats_t stats;

    TEST_ASSERT(init_queue(&queue, wheel, 2U, &attitude, imu) == 0);
    for (uint8_t sequence = 1U; sequence <= 2U; ++sequence) {
        TEST_ASSERT(make_frame(SRP_MSG_ID_WHEEL_SPEED_STATUS, sequence, 0U,
                               (float)sequence, wire, &wire_length) == 0);
        TEST_ASSERT(radar_telemetry_queue_push(
                        &queue, SRP_MSG_ID_WHEEL_SPEED_STATUS, wire,
                        wire_length, (uint32_t)sequence * 10U));
    }
    TEST_ASSERT(make_frame(SRP_MSG_ID_WHEEL_SPEED_STATUS, 3U, 0U, 3.0f,
                           wire, &wire_length) == 0);
    TEST_ASSERT(!radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_WHEEL_SPEED_STATUS, wire,
                    wire_length, 30U));
    radar_telemetry_queue_get_stats(&queue, &stats);
    TEST_ASSERT(stats.wheel.accepted == 2U);
    TEST_ASSERT(stats.wheel.dropped == 1U);
    TEST_ASSERT(stats.wheel.depth == 2U);
    TEST_ASSERT(stats.wheel.high_watermark == 2U);

    TEST_ASSERT(radar_telemetry_queue_pop(&queue, &output));
    TEST_ASSERT(output.message_id == SRP_MSG_ID_WHEEL_SPEED_STATUS);
    TEST_ASSERT(output.data[5] == 1U);
    TEST_ASSERT(output.ingress_timestamp_ms == 10U);
    TEST_ASSERT(radar_telemetry_queue_pop(&queue, &output));
    TEST_ASSERT(output.data[5] == 2U);
    TEST_ASSERT(!radar_telemetry_queue_has_pending(&queue));
    return 0;
}

static int test_attitude_latest_only(void)
{
    radar_telemetry_queue_t queue;
    radar_telemetry_entry_t wheel[1];
    radar_telemetry_entry_t attitude;
    radar_telemetry_entry_t imu[2];
    radar_telemetry_entry_t output;
    uint8_t wire[SRP_MAX_FRAME_SIZE];
    uint16_t wire_length = 0U;
    radar_telemetry_queue_stats_t stats;

    TEST_ASSERT(init_queue(&queue, wheel, 1U, &attitude, imu) == 0);
    TEST_ASSERT(make_frame(SRP_MSG_ID_ATTITUDE, 10U, 0U, 10.0f, wire,
                           &wire_length) == 0);
    TEST_ASSERT(radar_telemetry_queue_push(&queue, SRP_MSG_ID_ATTITUDE, wire,
                                           wire_length, 100U));
    TEST_ASSERT(make_frame(SRP_MSG_ID_ATTITUDE, 11U, 0U, 11.0f, wire,
                           &wire_length) == 0);
    TEST_ASSERT(radar_telemetry_queue_push(&queue, SRP_MSG_ID_ATTITUDE, wire,
                                           wire_length, 110U));
    radar_telemetry_queue_get_stats(&queue, &stats);
    TEST_ASSERT(stats.attitude.accepted == 2U);
    TEST_ASSERT(stats.attitude.overwritten == 1U);
    TEST_ASSERT(stats.attitude.depth == 1U);
    TEST_ASSERT(stats.attitude.high_watermark == 1U);
    TEST_ASSERT(radar_telemetry_queue_pop(&queue, &output));
    TEST_ASSERT(output.message_id == SRP_MSG_ID_ATTITUDE);
    TEST_ASSERT(output.data[5] == 11U);
    TEST_ASSERT(output.ingress_timestamp_ms == 110U);
    return 0;
}

static int test_independent_imu_latest_slots(void)
{
    radar_telemetry_queue_t queue;
    radar_telemetry_entry_t wheel[1];
    radar_telemetry_entry_t attitude;
    radar_telemetry_entry_t imu[2];
    radar_telemetry_entry_t output;
    uint8_t wire[SRP_MAX_FRAME_SIZE];
    uint16_t wire_length = 0U;
    radar_telemetry_queue_stats_t stats;

    TEST_ASSERT(init_queue(&queue, wheel, 1U, &attitude, imu) == 0);
    TEST_ASSERT(make_frame(SRP_MSG_ID_IMU_TELEMETRY, 20U,
                           SRP_IMU_SENSOR_LSM303, 20.0f, wire,
                           &wire_length) == 0);
    TEST_ASSERT(radar_telemetry_queue_push(&queue, SRP_MSG_ID_IMU_TELEMETRY,
                                           wire, wire_length, 200U));
    TEST_ASSERT(make_frame(SRP_MSG_ID_IMU_TELEMETRY, 21U,
                           SRP_IMU_SENSOR_BMI323, 21.0f, wire,
                           &wire_length) == 0);
    TEST_ASSERT(radar_telemetry_queue_push(&queue, SRP_MSG_ID_IMU_TELEMETRY,
                                           wire, wire_length, 210U));
    TEST_ASSERT(make_frame(SRP_MSG_ID_IMU_TELEMETRY, 22U,
                           SRP_IMU_SENSOR_LSM303, 22.0f, wire,
                           &wire_length) == 0);
    TEST_ASSERT(radar_telemetry_queue_push(&queue, SRP_MSG_ID_IMU_TELEMETRY,
                                           wire, wire_length, 220U));

    radar_telemetry_queue_get_stats(&queue, &stats);
    TEST_ASSERT(stats.imu_lsm303.accepted == 2U);
    TEST_ASSERT(stats.imu_lsm303.overwritten == 1U);
    TEST_ASSERT(stats.imu_bmi323.accepted == 1U);
    TEST_ASSERT(stats.imu_bmi323.overwritten == 0U);
    TEST_ASSERT(stats.depth == 2U);
    TEST_ASSERT(radar_telemetry_queue_pop(&queue, &output));
    TEST_ASSERT(output.data[5] == 22U);
    TEST_ASSERT(output.data[8] == SRP_IMU_SENSOR_LSM303);
    TEST_ASSERT(radar_telemetry_queue_pop(&queue, &output));
    TEST_ASSERT(output.data[5] == 21U);
    TEST_ASSERT(output.data[8] == SRP_IMU_SENSOR_BMI323);
    TEST_ASSERT(!radar_telemetry_queue_has_pending(&queue));
    return 0;
}

static int test_wheel_priority_is_bounded_fair(void)
{
    radar_telemetry_queue_t queue;
    radar_telemetry_entry_t wheel[8];
    radar_telemetry_entry_t attitude;
    radar_telemetry_entry_t imu[2];
    radar_telemetry_entry_t output;
    uint8_t wire[SRP_MAX_FRAME_SIZE];
    uint16_t wire_length = 0U;
    uint8_t sequence;

    TEST_ASSERT(init_queue(&queue, wheel, 8U, &attitude, imu) == 0);
    TEST_ASSERT(make_frame(SRP_MSG_ID_ATTITUDE, 100U, 0U, 100.0f, wire,
                           &wire_length) == 0);
    TEST_ASSERT(radar_telemetry_queue_push(&queue, SRP_MSG_ID_ATTITUDE, wire,
                                           wire_length, 1000U));
    for (sequence = 1U; sequence <= 8U; ++sequence) {
        TEST_ASSERT(make_frame(SRP_MSG_ID_WHEEL_SPEED_STATUS, sequence, 0U,
                               (float)sequence, wire, &wire_length) == 0);
        TEST_ASSERT(radar_telemetry_queue_push(
                        &queue, SRP_MSG_ID_WHEEL_SPEED_STATUS, wire,
                        wire_length, (uint32_t)sequence));
    }

    for (sequence = 1U; sequence <= 4U; ++sequence) {
        TEST_ASSERT(radar_telemetry_queue_pop(&queue, &output));
        TEST_ASSERT(output.message_id == SRP_MSG_ID_WHEEL_SPEED_STATUS);
        TEST_ASSERT(output.data[5] == sequence);
    }
    TEST_ASSERT(radar_telemetry_queue_pop(&queue, &output));
    TEST_ASSERT(output.message_id == SRP_MSG_ID_ATTITUDE);
    TEST_ASSERT(output.data[5] == 100U);
    return 0;
}

static int test_invalid_inputs_are_rejected(void)
{
    radar_telemetry_queue_t queue;
    radar_telemetry_entry_t wheel[2];
    radar_telemetry_entry_t attitude;
    radar_telemetry_entry_t imu[2];
    uint8_t wire[SRP_MAX_FRAME_SIZE];
    uint16_t wire_length = 0U;
    radar_telemetry_queue_stats_t stats;

    TEST_ASSERT(init_queue(&queue, wheel, 2U, &attitude, imu) == 0);
    TEST_ASSERT(!radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_WHEEL_SPEED_STATUS, NULL, 0U, 0U));
    TEST_ASSERT(make_frame(SRP_MSG_ID_WHEEL_SPEED_STATUS, 1U, 0U, 1.0f,
                           wire, &wire_length) == 0);
    TEST_ASSERT(!radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_ATTITUDE, wire, wire_length, 0U));
    wire[wire_length - 1U] ^= 0x01U;
    TEST_ASSERT(!radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_WHEEL_SPEED_STATUS, wire,
                    wire_length, 0U));
    TEST_ASSERT(make_frame(SRP_MSG_ID_IMU_TELEMETRY, 2U, 0x03U, 2.0f, wire,
                           &wire_length) == 0);
    TEST_ASSERT(!radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_IMU_TELEMETRY, wire, wire_length, 0U));
    TEST_ASSERT(make_frame(SRP_MSG_ID_WHEEL_SPEED_STATUS, 3U, 0U, 3.0f,
                           wire, &wire_length) == 0);
    TEST_ASSERT(!radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_WHEEL_SPEED_STATUS, wire,
                    wire_length - 1U, 0U));
    TEST_ASSERT(make_frame(SRP_MSG_ID_WHEEL_SPEED_STATUS, 4U, 0U, 4.0f,
                           wire, &wire_length) == 0);
    wire[4] = SRP_FLAG_ACK_REQUIRED; /* Telemetry must not request ACK. */
    {
        const uint16_t crc = srp_crc16_ccitt_false(
            &wire[2], 6U + SRP_PAYLOAD_WHEEL_SPEED_STATUS_SIZE);
        wire[wire_length - 4U] = (uint8_t)crc;
        wire[wire_length - 3U] = (uint8_t)(crc >> 8U);
    }
    TEST_ASSERT(!radar_telemetry_queue_push(
                    &queue, SRP_MSG_ID_WHEEL_SPEED_STATUS, wire,
                    wire_length, 0U));
    radar_telemetry_queue_get_stats(&queue, &stats);
    TEST_ASSERT(stats.rejected == 6U);
    TEST_ASSERT(stats.depth == 0U);
    TEST_ASSERT(!radar_telemetry_queue_has_pending(&queue));
    return 0;
}

int main(void)
{
    if (test_maximum_frame_contract() != 0 ||
        test_wheel_fifo_order_and_full_rejection() != 0 ||
        test_attitude_latest_only() != 0 ||
        test_independent_imu_latest_slots() != 0 ||
        test_wheel_priority_is_bounded_fair() != 0 ||
        test_invalid_inputs_are_rejected() != 0) {
        return 1;
    }
    return 0;
}
