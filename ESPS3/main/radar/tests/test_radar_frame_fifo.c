#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "radar_frame_fifo.h"

static void test_fifo_preserves_order_and_metadata(void)
{
    radar_frame_fifo_t fifo;
    static radar_frame_fifo_entry_t entries[4];
    uint8_t input[3] = {0xAAU, 0x55U, 0x01U};
    uint8_t output[RADAR_PARSER_MAX_FRAME_SIZE] = {0};
    size_t length = 0U;
    uint32_t sequence = 0U;
    uint32_t timestamp_ms = 0U;
    radar_frame_fifo_stats_t stats;

    assert(radar_frame_fifo_init(&fifo, entries, 4U));
    assert(radar_frame_fifo_push(&fifo, input, sizeof(input), 10U, 100U));
    input[2] = 0x02U;
    assert(radar_frame_fifo_push(&fifo, input, sizeof(input), 11U, 200U));

    assert(radar_frame_fifo_pop(&fifo, output, sizeof(output), &length,
                                &sequence, &timestamp_ms));
    assert(length == sizeof(input));
    assert(sequence == 10U);
    assert(timestamp_ms == 100U);
    assert(memcmp(output, (uint8_t[]){0xAAU, 0x55U, 0x01U}, length) == 0);

    assert(radar_frame_fifo_pop(&fifo, output, sizeof(output), &length,
                                &sequence, &timestamp_ms));
    assert(sequence == 11U);
    assert(timestamp_ms == 200U);
    radar_frame_fifo_get_stats(&fifo, &stats);
    assert(stats.capacity == 4U);
    assert(stats.count == 0U);
    assert(stats.high_watermark == 2U);
    assert(stats.dropped_oldest_count == 0U);
}

static void test_full_fifo_drops_oldest_only(void)
{
    radar_frame_fifo_t fifo;
    static radar_frame_fifo_entry_t entries[RADAR_FRAME_FIFO_DEPTH];
    uint8_t input[1] = {0U};
    uint8_t output[RADAR_PARSER_MAX_FRAME_SIZE] = {0};
    size_t length = 0U;
    uint32_t sequence = 0U;
    radar_frame_fifo_stats_t stats;

    assert(radar_frame_fifo_init(&fifo, entries, RADAR_FRAME_FIFO_DEPTH));
    for (uint32_t value = 1U; value <= RADAR_FRAME_FIFO_DEPTH; ++value) {
        input[0] = (uint8_t)value;
        assert(radar_frame_fifo_push(&fifo, input, sizeof(input), value, value));
    }
    input[0] = 9U;
    assert(radar_frame_fifo_push(&fifo, input, sizeof(input), 9U, 9U));

    radar_frame_fifo_get_stats(&fifo, &stats);
    assert(stats.capacity == RADAR_FRAME_FIFO_DEPTH);
    assert(stats.count == RADAR_FRAME_FIFO_DEPTH);
    assert(stats.high_watermark == RADAR_FRAME_FIFO_DEPTH);
    assert(stats.dropped_oldest_count == 1U);
    assert(radar_frame_fifo_pop(&fifo, output, sizeof(output), &length,
                                &sequence, NULL));
    assert(sequence == 2U);
    assert(output[0] == 2U);
    while (radar_frame_fifo_pop(&fifo, output, sizeof(output), &length,
                                &sequence, NULL)) {
    }
    assert(length == 0U);
}

static void test_short_output_does_not_consume(void)
{
    radar_frame_fifo_t fifo;
    static radar_frame_fifo_entry_t entries[1];
    uint8_t input[4] = {1U, 2U, 3U, 4U};
    uint8_t output[2] = {0};
    size_t length = 0U;
    uint32_t sequence = 0U;

    assert(radar_frame_fifo_init(&fifo, entries, 1U));
    assert(radar_frame_fifo_push(&fifo, input, sizeof(input), 7U, 70U));
    assert(!radar_frame_fifo_pop(&fifo, output, sizeof(output), &length,
                                 &sequence, NULL));
    assert(length == sizeof(input));
    assert(radar_frame_fifo_pop(&fifo, input, sizeof(input), &length,
                                &sequence, NULL));
    assert(length == sizeof(input));
    assert(sequence == 7U);
}

static void test_invalid_storage_is_rejected(void)
{
    radar_frame_fifo_t fifo;
    radar_frame_fifo_entry_t entries[1];

    assert(!radar_frame_fifo_init(&fifo, NULL, 1U));
    assert(!radar_frame_fifo_init(&fifo, entries, 0U));
}

int main(void)
{
    test_fifo_preserves_order_and_metadata();
    test_full_fifo_drops_oldest_only();
    test_short_output_does_not_consume();
    test_invalid_storage_is_rejected();
    return 0;
}
