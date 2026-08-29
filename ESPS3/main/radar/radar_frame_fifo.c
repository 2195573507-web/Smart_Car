#include "radar_frame_fifo.h"

#include <string.h>

bool radar_frame_fifo_init(radar_frame_fifo_t *fifo,
                           radar_frame_fifo_entry_t *entries,
                           size_t capacity)
{
    if (fifo == NULL || entries == NULL || capacity == 0U) {
        return false;
    }

    memset(fifo, 0, sizeof(*fifo));
    fifo->entries = entries;
    fifo->capacity = capacity;
    return true;
}

bool radar_frame_fifo_push(radar_frame_fifo_t *fifo,
                           const uint8_t *data,
                           size_t length,
                           uint32_t sequence,
                           uint32_t timestamp_ms)
{
    if (fifo == NULL || fifo->entries == NULL || fifo->capacity == 0U ||
        data == NULL || length == 0U ||
        length > RADAR_PARSER_MAX_FRAME_SIZE) {
        return false;
    }

    if (fifo->count == fifo->capacity) {
        fifo->tail = (fifo->tail + 1U) % fifo->capacity;
        --fifo->count;
        ++fifo->dropped_oldest_count;
    }

    radar_frame_fifo_entry_t *entry = &fifo->entries[fifo->head];
    memcpy(entry->data, data, length);
    entry->length = length;
    entry->sequence = sequence;
    entry->timestamp_ms = timestamp_ms;
    fifo->head = (fifo->head + 1U) % fifo->capacity;
    ++fifo->count;
    if (fifo->count > fifo->high_watermark) {
        fifo->high_watermark = fifo->count;
    }
    return true;
}

bool radar_frame_fifo_pop(radar_frame_fifo_t *fifo,
                          uint8_t *data,
                          size_t capacity,
                          size_t *length,
                          uint32_t *sequence,
                          uint32_t *timestamp_ms)
{
    if (fifo == NULL || fifo->entries == NULL || fifo->capacity == 0U ||
        data == NULL || length == NULL || capacity == 0U) {
        return false;
    }
    if (fifo->count == 0U) {
        *length = 0U;
        return false;
    }

    const radar_frame_fifo_entry_t *entry = &fifo->entries[fifo->tail];
    if (entry->length > capacity) {
        *length = entry->length;
        return false;
    }

    memcpy(data, entry->data, entry->length);
    *length = entry->length;
    if (sequence != NULL) {
        *sequence = entry->sequence;
    }
    if (timestamp_ms != NULL) {
        *timestamp_ms = entry->timestamp_ms;
    }
    fifo->tail = (fifo->tail + 1U) % fifo->capacity;
    --fifo->count;
    return true;
}

void radar_frame_fifo_get_stats(const radar_frame_fifo_t *fifo,
                                radar_frame_fifo_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    memset(stats, 0, sizeof(*stats));
    if (fifo != NULL) {
        stats->capacity = fifo->capacity;
        stats->count = fifo->count;
        stats->high_watermark = fifo->high_watermark;
        stats->dropped_oldest_count = fifo->dropped_oldest_count;
    }
}
