#ifndef S3_RADAR_TELEMETRY_QUEUE_H
#define S3_RADAR_TELEMETRY_QUEUE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "srp_registry.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * This queue transports complete SRPv4 wire frames.  Keep the storage
 * limit tied to the shared protocol definition so a frame can never be
 * truncated while crossing the service/uplink boundary.
 */
#define RADAR_TELEMETRY_QUEUE_MAX_FRAME_SIZE SRP_MAX_FRAME_SIZE
#define RADAR_TELEMETRY_QUEUE_IMU_SLOT_COUNT 2U
#define RADAR_TELEMETRY_QUEUE_WHEEL_BURST_MAX 4U

typedef enum {
    RADAR_TELEMETRY_QUEUE_CLASS_WHEEL = 0,
    RADAR_TELEMETRY_QUEUE_CLASS_ATTITUDE,
    RADAR_TELEMETRY_QUEUE_CLASS_IMU_LSM303,
    RADAR_TELEMETRY_QUEUE_CLASS_IMU_BMI323,
    RADAR_TELEMETRY_QUEUE_CLASS_COUNT
} radar_telemetry_queue_class_t;

/* A queue entry owns its bytes; no parser-owned payload pointer escapes. */
typedef struct {
    uint8_t data[RADAR_TELEMETRY_QUEUE_MAX_FRAME_SIZE];
    uint16_t length;
    uint16_t message_id;
    uint32_t ingress_timestamp_ms;
} radar_telemetry_entry_t;

/* Per-stream counters and current resource usage. */
typedef struct {
    uint32_t accepted;
    uint32_t overwritten;
    uint32_t dropped;
    size_t depth;
    size_t high_watermark;
} radar_telemetry_queue_stream_stats_t;

typedef struct {
    radar_telemetry_queue_stream_stats_t wheel;
    radar_telemetry_queue_stream_stats_t attitude;
    radar_telemetry_queue_stream_stats_t imu_lsm303;
    radar_telemetry_queue_stream_stats_t imu_bmi323;
    uint32_t rejected;
    size_t depth;
    size_t high_watermark;
} radar_telemetry_queue_stats_t;

/* All entry storage is supplied and retained by the caller. */
typedef struct {
    radar_telemetry_entry_t *wheel_entries;
    size_t wheel_capacity;
    radar_telemetry_entry_t *attitude_entry;
    radar_telemetry_entry_t *imu_entries;
} radar_telemetry_queue_storage_t;

/*
 * Queue state is intentionally plain C and contains no RTOS primitives.
 * Callers must serialize push/pop/stats access when used from multiple
 * execution contexts.
 */
typedef struct {
    radar_telemetry_queue_storage_t storage;
    size_t wheel_head;
    size_t wheel_tail;
    size_t wheel_count;
    bool attitude_pending;
    bool imu_pending[RADAR_TELEMETRY_QUEUE_IMU_SLOT_COUNT];
    uint8_t next_observation;
    uint8_t wheel_burst;
    bool initialized;
    radar_telemetry_queue_stats_t stats;
} radar_telemetry_queue_t;

/*
 * Initialize a queue over caller-owned storage.  imu_entries must point to
 * an array of RADAR_TELEMETRY_QUEUE_IMU_SLOT_COUNT entries; index 0 is sensor
 * id 0x01 (LSM303) and index 1 is sensor id 0x02 (BMI323).
 */
bool radar_telemetry_queue_init(
    radar_telemetry_queue_t *queue,
    const radar_telemetry_queue_storage_t *storage);

/*
 * Validate and enqueue one complete encoded SRPv4 frame.  message_id must
 * match the frame type.  Invalid/unsupported frames
 * increment stats.rejected.  A full wheel FIFO rejects the new frame and
 * increments wheel.dropped; it never discards an older wheel sample.
 */
bool radar_telemetry_queue_push(
    radar_telemetry_queue_t *queue,
    uint16_t message_id,
    const uint8_t *encoded_frame,
    size_t encoded_length,
    uint32_t ingress_timestamp_ms);

/* Pop according to the bounded-fairness wheel-priority scheduler. */
bool radar_telemetry_queue_pop(radar_telemetry_queue_t *queue,
                               radar_telemetry_entry_t *entry);

bool radar_telemetry_queue_has_pending(
    const radar_telemetry_queue_t *queue);

void radar_telemetry_queue_get_stats(
    const radar_telemetry_queue_t *queue,
    radar_telemetry_queue_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* S3_RADAR_TELEMETRY_QUEUE_H */
