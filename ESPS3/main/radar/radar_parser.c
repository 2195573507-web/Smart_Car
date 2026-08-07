#include "radar_parser.h"

#include <stdio.h>
#include <string.h>

static size_t ring_index(const radar_parser_t *parser, size_t offset)
{
    return (parser->tail + offset) % RADAR_PARSER_RING_BUFFER_SIZE;
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
        ++parser->overflow_count;
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

static void process_frames(radar_parser_t *parser,
                           radar_frame_callback_t callback,
                           void *context)
{
    uint8_t frame[RADAR_PARSER_MAX_FRAME_SIZE];

    while (parser->size >= 2U) {
        size_t header_offset = 0U;
        if (!find_header(parser, &header_offset)) {
            /* Keep unrecognized bytes in the ring until it fills.  The next
             * feed can then complete a split AA 55 header; ring_push() is the
             * only place that drops data, and it records overflow explicitly. */
            return;
        }

        if (header_offset > 0U) {
            ring_drop(parser, header_offset);
        }

        /* CT and LSN are the first two fields after the two-byte header. */
        if (parser->size <= RADAR_X3PRO_LENGTH_OFFSET) {
            return;
        }

        size_t sample_count = ring_peek(parser, RADAR_X3PRO_LENGTH_OFFSET);
        if (sample_count == 0U || sample_count > RADAR_X3PRO_MAX_SAMPLES) {
            ring_drop(parser, 1U);
            continue;
        }

        size_t frame_length = RADAR_X3PRO_HEADER_BYTES +
                              (sample_count * parser->sample_bytes);
        if (frame_length > RADAR_PARSER_MAX_FRAME_SIZE) {
            ring_drop(parser, 1U);
            continue;
        }
        if (parser->size < frame_length) {
            return;
        }

        copy_frame(parser, frame, frame_length);
        bool checksum_valid = parser->checksum_validator == NULL ||
                              parser->checksum_validator(frame,
                                                         frame_length,
                                                         parser->checksum_context);
        if (!checksum_valid) {
            /* Keep searching from the next byte after this candidate header. */
            ring_drop(parser, 1U);
            continue;
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
    parser->sample_bytes = RADAR_X3PRO_SAMPLE_BYTES;
}

void radar_parser_set_checksum_validator(
    radar_parser_t *parser,
    radar_parser_checksum_validator_t validator,
    void *context)
{
    if (parser == NULL) {
        return;
    }

    parser->checksum_validator = validator;
    parser->checksum_context = context;
}

bool radar_parser_set_sample_bytes(radar_parser_t *parser, size_t sample_bytes)
{
    if (parser == NULL ||
        (sample_bytes != RADAR_X3PRO_SAMPLE_BYTES &&
         sample_bytes != RADAR_X3PRO_MAX_SAMPLE_BYTES)) {
        return false;
    }

    parser->sample_bytes = sample_bytes;
    return true;
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
