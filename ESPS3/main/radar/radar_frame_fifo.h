#ifndef S3_RADAR_FRAME_FIFO_H
#define S3_RADAR_FRAME_FIFO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "radar_parser.h"

/* 256 complete frames provide a bounded PSRAM backlog for short TCP stalls. */
#define RADAR_FRAME_FIFO_DEPTH 256U

typedef struct {
    uint8_t data[RADAR_PARSER_MAX_FRAME_SIZE];
    size_t length;
    uint32_t sequence;
    uint32_t timestamp_ms;
} radar_frame_fifo_entry_t;

typedef struct {
    radar_frame_fifo_entry_t *entries;
    size_t capacity;
    size_t head;
    size_t tail;
    size_t count;
    size_t high_watermark;
    uint32_t dropped_oldest_count;
} radar_frame_fifo_t;

typedef struct {
    size_t capacity;
    size_t count;
    size_t high_watermark;
    uint32_t dropped_oldest_count;
} radar_frame_fifo_stats_t;

/* The caller owns the entry storage for the lifetime of the FIFO. */
bool radar_frame_fifo_init(radar_frame_fifo_t *fifo,
                           radar_frame_fifo_entry_t *entries,
                           size_t capacity);

/* Push a complete validated frame. A full queue drops its oldest entry. */
bool radar_frame_fifo_push(radar_frame_fifo_t *fifo,
                           const uint8_t *data,
                           size_t length,
                           uint32_t sequence,
                           uint32_t timestamp_ms);

/* Pop the oldest frame. A short output buffer leaves the entry queued. */
bool radar_frame_fifo_pop(radar_frame_fifo_t *fifo,
                          uint8_t *data,
                          size_t capacity,
                          size_t *length,
                          uint32_t *sequence,
                          uint32_t *timestamp_ms);

void radar_frame_fifo_get_stats(const radar_frame_fifo_t *fifo,
                                radar_frame_fifo_stats_t *stats);

#endif
