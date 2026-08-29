#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "radar_parser.h"

typedef struct {
    uint32_t count;
    size_t last_length;
    uint8_t last_frame[RADAR_PARSER_MAX_FRAME_SIZE];
} frame_capture_t;

static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static void write_le16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static uint16_t make_checksum(const uint8_t *frame,
                              size_t sample_count,
                              size_t sample_bytes)
{
    uint16_t checksum = 0x55AAU;
    checksum ^= read_le16(&frame[4]);
    for (size_t index = 0U; index < sample_count; ++index) {
        const size_t offset = RADAR_X3PRO_HEADER_BYTES + index * sample_bytes;
        if (sample_bytes == 2U) {
            checksum ^= read_le16(&frame[offset]);
        } else {
            checksum ^= frame[offset];
            checksum ^= read_le16(&frame[offset + 1U]);
        }
    }
    checksum ^= (uint16_t)(((uint16_t)frame[3] << 8U) | frame[2]);
    checksum ^= read_le16(&frame[6]);
    return checksum;
}

static size_t make_frame(uint8_t *frame,
                         uint8_t ct,
                         uint8_t sample_count,
                         size_t sample_bytes)
{
    const size_t length = RADAR_X3PRO_HEADER_BYTES +
                          (size_t)sample_count * sample_bytes;
    memset(frame, 0, length);
    frame[0] = RADAR_X3PRO_HEADER_BYTE_0;
    frame[1] = RADAR_X3PRO_HEADER_BYTE_1;
    frame[2] = ct;
    frame[3] = sample_count;
    write_le16(&frame[4], 0xAE53U);
    write_le16(&frame[6], 0xAE53U);
    for (size_t index = 0U; index < sample_count; ++index) {
        const size_t offset = RADAR_X3PRO_HEADER_BYTES + index * sample_bytes;
        if (sample_bytes == 2U) {
            write_le16(&frame[offset], (uint16_t)(0x0400U + index * 4U));
        } else {
            frame[offset] = (uint8_t)(0x10U + index);
            write_le16(&frame[offset + 1U],
                       (uint16_t)(0x0400U + index * 4U));
        }
    }
    write_le16(&frame[8], make_checksum(frame, sample_count, sample_bytes));
    return length;
}

static void capture_frame(const uint8_t *data, size_t length, void *context)
{
    frame_capture_t *capture = context;
    assert(capture != NULL);
    assert(length <= sizeof(capture->last_frame));
    ++capture->count;
    capture->last_length = length;
    memcpy(capture->last_frame, data, length);
}

static void test_split_and_combined_distance_frames(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    const size_t length = make_frame(frame, 0x01U, 2U, 2U);
    radar_parser_t parser;
    frame_capture_t capture = {0};
    radar_parser_stats_t stats;

    radar_parser_init(&parser);
    radar_parser_feed(&parser, frame, 1U, capture_frame, &capture);
    radar_parser_feed(&parser, &frame[1], length - 1U, capture_frame, &capture);
    radar_parser_feed(&parser, frame, length, capture_frame, &capture);

    radar_parser_get_stats(&parser, &stats);
    assert(capture.count == 2U);
    assert(capture.last_length == length);
    assert(memcmp(capture.last_frame, frame, length) == 0);
    assert(stats.valid_frame_count == 2U);
    assert(stats.valid_distance_frame_count == 2U);
    assert(stats.valid_intensity_frame_count == 0U);
    assert(stats.last_sample_bytes == RADAR_X3PRO_SAMPLE_BYTES);
    assert(stats.checksum_error_count == 0U);
}

static void test_official_protocol_example(void)
{
    const uint8_t frame[] = {
        0xAAU, 0x55U, 0x01U, 0x01U, 0x53U, 0xAEU,
        0x53U, 0xAEU, 0xABU, 0x54U, 0x00U, 0x00U,
    };
    radar_parser_t parser;
    frame_capture_t capture = {0};

    radar_parser_init(&parser);
    radar_parser_feed(&parser, frame, sizeof(frame), capture_frame, &capture);

    assert(capture.count == 1U);
    assert(capture.last_length == sizeof(frame));
    assert(memcmp(capture.last_frame, frame, sizeof(frame)) == 0);
}

static void test_auto_detects_intensity_frame(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    const size_t length = make_frame(frame, 0x00U, 2U, 3U);
    radar_parser_t parser;
    frame_capture_t capture = {0};
    radar_parser_stats_t stats;

    radar_parser_init(&parser);
    const size_t distance_candidate_length =
        RADAR_X3PRO_HEADER_BYTES + (2U * RADAR_X3PRO_SAMPLE_BYTES);
    radar_parser_feed(&parser, frame, distance_candidate_length,
                      capture_frame, &capture);
    radar_parser_get_stats(&parser, &stats);
    assert(capture.count == 0U);
    assert(stats.checksum_error_count == 0U);

    radar_parser_feed(&parser, &frame[distance_candidate_length],
                      length - distance_candidate_length,
                      capture_frame, &capture);

    radar_parser_get_stats(&parser, &stats);
    assert(capture.count == 1U);
    assert(capture.last_length == length);
    assert(stats.valid_frame_count == 1U);
    assert(stats.valid_distance_frame_count == 0U);
    assert(stats.valid_intensity_frame_count == 1U);
    assert(stats.last_sample_bytes == RADAR_X3PRO_MAX_SAMPLE_BYTES);
}

static void test_explicit_sample_mode(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    const size_t length = make_frame(frame, 0x00U, 1U, 3U);
    radar_parser_t parser;
    frame_capture_t capture = {0};
    radar_parser_stats_t stats;

    radar_parser_init(&parser);
    assert(radar_parser_set_sample_bytes(&parser, RADAR_X3PRO_MAX_SAMPLE_BYTES));
    radar_parser_feed(&parser, frame, length, capture_frame, &capture);
    radar_parser_get_stats(&parser, &stats);

    assert(capture.count == 1U);
    assert(stats.valid_intensity_frame_count == 1U);
    assert(radar_parser_set_sample_bytes(&parser,
                                         RADAR_X3PRO_SAMPLE_BYTES_AUTO));
    assert(!radar_parser_set_sample_bytes(&parser, 1U));
}

static void test_bad_checksum_and_resync(void)
{
    uint8_t bad_frame[RADAR_PARSER_MAX_FRAME_SIZE];
    uint8_t valid_frame[RADAR_PARSER_MAX_FRAME_SIZE];
    const size_t bad_length = make_frame(bad_frame, 0x00U, 1U, 2U);
    const size_t valid_length = make_frame(valid_frame, 0x01U, 1U, 2U);
    radar_parser_t parser;
    frame_capture_t capture = {0};
    radar_parser_stats_t stats;

    bad_frame[8] ^= 0x01U;
    radar_parser_init(&parser);
    radar_parser_feed(&parser, bad_frame, bad_length, capture_frame, &capture);
    radar_parser_feed(&parser, valid_frame, valid_length, capture_frame, &capture);

    radar_parser_get_stats(&parser, &stats);
    assert(capture.count == 1U);
    assert(stats.valid_frame_count == 1U);
    assert(stats.checksum_error_count == 1U);
    assert(stats.header_resync_count >= 1U);
}

static void test_noise_invalid_length_and_split_header(void)
{
    const uint8_t noise[] = {0x10U, 0x20U, 0xAAU};
    const uint8_t invalid_length[] = {0xAAU, 0x55U, 0x00U, 0x00U};
    uint8_t valid_frame[RADAR_PARSER_MAX_FRAME_SIZE];
    const size_t valid_length = make_frame(valid_frame, 0x00U, 1U, 2U);
    radar_parser_t parser;
    frame_capture_t capture = {0};
    radar_parser_stats_t stats;

    radar_parser_init(&parser);
    radar_parser_feed(&parser, noise, sizeof(noise), capture_frame, &capture);
    radar_parser_feed(&parser, &valid_frame[1], valid_length - 1U,
                      capture_frame, &capture);
    radar_parser_feed(&parser, invalid_length, sizeof(invalid_length),
                      capture_frame, &capture);
    radar_parser_feed(&parser, valid_frame, valid_length, capture_frame, &capture);

    radar_parser_get_stats(&parser, &stats);
    assert(capture.count == 2U);
    assert(stats.invalid_frame_count == 1U);
    assert(stats.header_resync_count >= 1U);
}

static void test_stream_reset_preserves_stats(void)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];
    const size_t length = make_frame(frame, 0x00U, 1U, 2U);
    radar_parser_t parser;
    frame_capture_t capture = {0};
    radar_parser_stats_t stats;

    radar_parser_init(&parser);
    radar_parser_feed(&parser, frame, length, capture_frame, &capture);
    radar_parser_feed(&parser, frame, 4U, capture_frame, &capture);
    radar_parser_reset_stream(&parser);
    radar_parser_feed(&parser, frame, length, capture_frame, &capture);

    radar_parser_get_stats(&parser, &stats);
    assert(capture.count == 2U);
    assert(stats.valid_frame_count == 2U);
}

int main(void)
{
    test_split_and_combined_distance_frames();
    test_official_protocol_example();
    test_auto_detects_intensity_frame();
    test_explicit_sample_mode();
    test_bad_checksum_and_resync();
    test_noise_invalid_length_and_split_header();
    test_stream_reset_preserves_stats();
    return 0;
}
