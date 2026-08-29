#include "radar_parser.h"

#include <stdio.h>
#include <string.h>

static size_t ring_index(const radar_parser_t *parser, size_t offset)
{
    return (parser->tail + offset) % RADAR_PARSER_RING_BUFFER_SIZE;
}

static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static bool radar_parser_default_checksum(const uint8_t *frame,
                                          size_t length,
                                          void *context)
{
    (void)context;

    if (frame == NULL || length < RADAR_X3PRO_HEADER_BYTES ||
        frame[0] != RADAR_X3PRO_HEADER_BYTE_0 ||
        frame[1] != RADAR_X3PRO_HEADER_BYTE_1) {
        return false;
    }

    const size_t sample_count = frame[3];
    if (sample_count == 0U || sample_count > RADAR_X3PRO_MAX_SAMPLES) {
        return false;
    }

    const size_t sample_payload_length = length - RADAR_X3PRO_HEADER_BYTES;
    if (sample_payload_length % sample_count != 0U) {
        return false;
    }

    const size_t sample_bytes = sample_payload_length / sample_count;
    if (sample_bytes != RADAR_X3PRO_SAMPLE_BYTES &&
        sample_bytes != RADAR_X3PRO_MAX_SAMPLE_BYTES) {
        return false;
    }

    uint16_t checksum = 0x55AAU;
    checksum ^= read_le16(&frame[4]);
    for (size_t index = 0U; index < sample_count; ++index) {
        const size_t offset = RADAR_X3PRO_HEADER_BYTES +
                              (index * sample_bytes);
        if (sample_bytes == RADAR_X3PRO_SAMPLE_BYTES) {
            checksum ^= read_le16(&frame[offset]);
        } else {
            checksum ^= frame[offset];
            checksum ^= read_le16(&frame[offset + 1U]);
        }
    }
    checksum ^= (uint16_t)(((uint16_t)frame[3] << 8U) | frame[2]);
    checksum ^= read_le16(&frame[6]);

    return checksum == read_le16(&frame[8]);
}

bool radar_parser_validate_frame(const uint8_t *frame,
                                 size_t length,
                                 size_t *sample_bytes)
{
    if (frame == NULL || length < RADAR_X3PRO_HEADER_BYTES ||
        frame[0] != RADAR_X3PRO_HEADER_BYTE_0 ||
        frame[1] != RADAR_X3PRO_HEADER_BYTE_1) {
        return false;
    }

    const size_t sample_count = frame[RADAR_X3PRO_LENGTH_OFFSET];
    if (sample_count == 0U || sample_count > RADAR_X3PRO_MAX_SAMPLES ||
        (frame[4] & 0x01U) == 0U || (frame[6] & 0x01U) == 0U) {
        return false;
    }

    const size_t payload_length = length - RADAR_X3PRO_HEADER_BYTES;
    if (payload_length % sample_count != 0U) {
        return false;
    }

    const size_t detected_sample_bytes = payload_length / sample_count;
    if (detected_sample_bytes != RADAR_X3PRO_SAMPLE_BYTES &&
        detected_sample_bytes != RADAR_X3PRO_MAX_SAMPLE_BYTES) {
        return false;
    }
    if (!radar_parser_default_checksum(frame, length, NULL)) {
        return false;
    }

    if (sample_bytes != NULL) {
        *sample_bytes = detected_sample_bytes;
    }
    return true;
}

static uint8_t ring_peek(const radar_parser_t *parser, size_t offset)
{
    return parser->buffer[ring_index(parser, offset)];
}

static void ring_drop(radar_parser_t *parser, size_t count)
{
    if (count >= parser->size) {
        parser->tail = parser->head;
        parser->size = 0U;
        return;
    }

    parser->tail = (parser->tail + count) % RADAR_PARSER_RING_BUFFER_SIZE;
    parser->size -= count;
}

static void ring_push(radar_parser_t *parser, uint8_t byte)
{
    if (parser->size == RADAR_PARSER_RING_BUFFER_SIZE) {
        parser->tail = (parser->tail + 1U) % RADAR_PARSER_RING_BUFFER_SIZE;
        --parser->size;
        ++parser->stats.overflow_count;
    }

    parser->buffer[parser->head] = byte;
    parser->head = (parser->head + 1U) % RADAR_PARSER_RING_BUFFER_SIZE;
    ++parser->size;
}

static bool find_header(const radar_parser_t *parser, size_t *offset)
{
    if (parser == NULL || offset == NULL || parser->size < 2U) {
        return false;
    }

    for (size_t index = 0U; index + 1U < parser->size; ++index) {
        if (ring_peek(parser, index) == RADAR_X3PRO_HEADER_BYTE_0 &&
            ring_peek(parser, index + 1U) == RADAR_X3PRO_HEADER_BYTE_1) {
            *offset = index;
            return true;
        }
    }
    return false;
}

static void copy_frame(const radar_parser_t *parser,
                       uint8_t *frame,
                       size_t frame_length)
{
    for (size_t index = 0U; index < frame_length; ++index) {
        frame[index] = ring_peek(parser, index);
    }
}

static bool candidate_checksum_valid(const radar_parser_t *parser,
                                     uint8_t *frame,
                                     size_t frame_length)
{
    copy_frame(parser, frame, frame_length);
    return parser->checksum_validator == NULL
               ? radar_parser_validate_frame(frame, frame_length, NULL)
               : parser->checksum_validator(frame,
                                            frame_length,
                                            parser->checksum_context);
}

static void process_frames(radar_parser_t *parser,
                           radar_frame_callback_t callback,
                           void *context)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];

    while (parser->size >= 2U) {
        size_t header_offset = 0U;
        if (!find_header(parser, &header_offset)) {
            /* Retain one trailing AA only: the next feed may complete an
             * AA 55 header, while all other noise can be discarded now. */
            const bool retain_header_prefix =
                ring_peek(parser, parser->size - 1U) == RADAR_X3PRO_HEADER_BYTE_0;
            const size_t drop_count = parser->size -
                                      (retain_header_prefix ? 1U : 0U);
            if (drop_count > 0U) {
                ++parser->stats.header_resync_count;
                ring_drop(parser, drop_count);
            }
            return;
        }

        if (header_offset > 0U) {
            ++parser->stats.header_resync_count;
            ring_drop(parser, header_offset);
        }

        /* CT and LSN are the first two fields after the two-byte header. */
        if (parser->size <= RADAR_X3PRO_LENGTH_OFFSET) {
            return;
        }

        size_t sample_count = ring_peek(parser, RADAR_X3PRO_LENGTH_OFFSET);
        if (sample_count == 0U || sample_count > RADAR_X3PRO_MAX_SAMPLES) {
            ++parser->stats.invalid_frame_count;
            ring_drop(parser, 1U);
            continue;
        }

        if (parser->size < RADAR_X3PRO_HEADER_BYTES) {
            return;
        }

        /* FSA and LSA carry the protocol check bit in bit 0. */
        if ((ring_peek(parser, 4U) & 0x01U) == 0U ||
            (ring_peek(parser, 6U) & 0x01U) == 0U) {
            ++parser->stats.invalid_frame_count;
            ++parser->stats.header_resync_count;
            ring_drop(parser, 1U);
            continue;
        }

        const size_t distance_frame_length = RADAR_X3PRO_HEADER_BYTES +
                                             (sample_count * RADAR_X3PRO_SAMPLE_BYTES);
        const size_t intensity_frame_length = RADAR_X3PRO_HEADER_BYTES +
                                              (sample_count * RADAR_X3PRO_MAX_SAMPLE_BYTES);
        size_t frame_length = 0U;
        size_t accepted_sample_bytes = 0U;

        if (parser->sample_bytes == RADAR_X3PRO_SAMPLE_BYTES_AUTO) {
            if (parser->size < distance_frame_length) {
                return;
            }
            if (candidate_checksum_valid(parser, frame, distance_frame_length)) {
                frame_length = distance_frame_length;
                accepted_sample_bytes = RADAR_X3PRO_SAMPLE_BYTES;
            } else {
                /* A failed two-byte candidate may be the prefix of an
                 * intensity frame. Do not resynchronise until the full
                 * three-byte candidate is available and has also failed. */
                if (parser->size < intensity_frame_length) {
                    return;
                }
                if (candidate_checksum_valid(parser, frame, intensity_frame_length)) {
                    frame_length = intensity_frame_length;
                    accepted_sample_bytes = RADAR_X3PRO_MAX_SAMPLE_BYTES;
                }
            }
        } else {
            accepted_sample_bytes = parser->sample_bytes;
            frame_length = RADAR_X3PRO_HEADER_BYTES +
                           (sample_count * accepted_sample_bytes);
            if (frame_length > RADAR_PARSER_MAX_FRAME_SIZE) {
                ++parser->stats.invalid_frame_count;
                ring_drop(parser, 1U);
                continue;
            }
            if (parser->size < frame_length) {
                return;
            }
            if (!candidate_checksum_valid(parser, frame, frame_length)) {
                frame_length = 0U;
                accepted_sample_bytes = 0U;
            }
        }

        if (accepted_sample_bytes == 0U) {
            ++parser->stats.checksum_error_count;
            ++parser->stats.header_resync_count;
            /* Keep searching from the next byte after this candidate header. */
            ring_drop(parser, 1U);
            continue;
        }

        ++parser->stats.valid_frame_count;
        parser->stats.last_sample_bytes = (uint8_t)accepted_sample_bytes;
        if (accepted_sample_bytes == RADAR_X3PRO_SAMPLE_BYTES) {
            ++parser->stats.valid_distance_frame_count;
        } else {
            ++parser->stats.valid_intensity_frame_count;
        }
        if (callback != NULL) {
            callback(frame, frame_length, context);
        }
        ring_drop(parser, frame_length);
    }
}

void radar_parser_init(radar_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->sample_bytes = RADAR_X3PRO_SAMPLE_BYTES_AUTO;
    parser->checksum_validator = radar_parser_default_checksum;
}

void radar_parser_reset_stream(radar_parser_t *parser)
{
    if (parser == NULL) {
        return;
    }

    parser->head = 0U;
    parser->tail = 0U;
    parser->size = 0U;
}

void radar_parser_set_checksum_validator(
    radar_parser_t *parser,
    radar_parser_checksum_validator_t validator,
    void *context)
{
    if (parser == NULL) {
        return;
    }

    parser->checksum_validator = validator == NULL ? radar_parser_default_checksum : validator;
    parser->checksum_context = context;
}

bool radar_parser_set_sample_bytes(radar_parser_t *parser, size_t sample_bytes)
{
    if (parser == NULL ||
        (sample_bytes != RADAR_X3PRO_SAMPLE_BYTES_AUTO &&
         sample_bytes != RADAR_X3PRO_SAMPLE_BYTES &&
         sample_bytes != RADAR_X3PRO_MAX_SAMPLE_BYTES)) {
        return false;
    }

    parser->sample_bytes = sample_bytes;
    return true;
}

void radar_parser_get_stats(const radar_parser_t *parser,
                            radar_parser_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    memset(stats, 0, sizeof(*stats));
    if (parser != NULL) {
        *stats = parser->stats;
    }
}

void radar_parser_feed(radar_parser_t *parser,
                       const uint8_t *data,
                       size_t length,
                       radar_frame_callback_t callback,
                       void *context)
{
    if (parser == NULL || (data == NULL && length > 0U)) {
        return;
    }

    for (size_t index = 0U; index < length; ++index) {
        ring_push(parser, data[index]);
    }
    process_frames(parser, callback, context);
}

bool radar_parser_parse_measurement(const uint8_t *frame,
                                    size_t length,
                                    radar_measurement_t *measurement)
{
    if (measurement != NULL) {
        memset(measurement, 0, sizeof(*measurement));
    }

    if (frame == NULL || measurement == NULL ||
        length < RADAR_X3PRO_HEADER_BYTES ||
        frame[0] != RADAR_X3PRO_HEADER_BYTE_0 ||
        frame[1] != RADAR_X3PRO_HEADER_BYTE_1) {
        return false;
    }

    /* Field conversion is intentionally deferred until the device variant is
     * confirmed from a captured X3PRO frame.  Do not emit made-up values. */
    return false;
}

void radar_parser_format_hex(const uint8_t *data,
                             size_t length,
                             char *output,
                             size_t output_size)
{
    if (output == NULL || output_size == 0U) {
        return;
    }

    output[0] = '\0';
    if (data == NULL) {
        return;
    }

    size_t written = 0U;
    for (size_t index = 0U; index < length; ++index) {
        if (written >= output_size) {
            break;
        }

        int count = snprintf(output + written,
                             output_size - written,
                             index == 0U ? "%02X" : " %02X",
                             data[index]);
        if (count < 0) {
            output[0] = '\0';
            return;
        }
        if ((size_t)count >= output_size - written) {
            output[output_size - 1U] = '\0';
            return;
        }
        written += (size_t)count;
    }
}
