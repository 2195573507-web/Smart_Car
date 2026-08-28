#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "srp_codec.h"
#include "srp_crc.h"
#include "srp_registry.h"
#include "srp_wire.h"

static uint32_t s_frames;
static uint8_t s_last_type;
static uint8_t s_last_sequence;
static uint32_t s_errors;
static srp_parser_error_t s_last_error;

static void on_frame(const srp_frame_t *frame, void *context)
{
    (void)context;
    assert(frame != NULL);
    ++s_frames;
    s_last_type = frame->type;
    s_last_sequence = frame->sequence;
}

static void on_error(srp_parser_error_t error, const uint8_t *data,
                     size_t length, void *context)
{
    (void)data;
    (void)length;
    (void)context;
    s_last_error = error;
    ++s_errors;
}

static void test_crc_vector(void)
{
    static const uint8_t vector[] = "123456789";

    assert(srp_crc16_ccitt_false(vector, sizeof(vector) - 1U) == UINT16_C(0x29B1));
}

static void test_encode_decode(void)
{
    const uint8_t payload[] = {0x10U, 0x20U, 0x30U, 0x40U};
    uint8_t bytes[SRP_MAX_FRAME_SIZE] = {0};
    uint16_t length = 0U;
    srp_frame_t input = {
        .priority = SRP_PRIORITY_COMMAND,
        .type = SRP_MSG_ID_MOTOR_CMD,
        .sequence = 0xFFU,
        .flags = 0U,
        .length = sizeof(payload),
        .payload = payload,
    };
    srp_frame_t output = {0};

    assert(srp_encode(&input, bytes, sizeof(bytes), &length) == SRP_CODEC_OK);
    assert(length == 16U);
    assert(bytes[0] == 0xAAU && bytes[1] == 0x55U);
    assert(bytes[2] == 0x04U && bytes[3] == 0x00U);
    assert(bytes[4] == 0x00U && bytes[5] == 0xFFU);
    assert(bytes[6] == SRP_MSG_ID_MOTOR_CMD && bytes[7] == 0x01U);
    assert(bytes[14U] == SRP_EOF_BYTE0 && bytes[15U] == SRP_EOF_BYTE1);
    assert(srp_decode(bytes, length, &output) == SRP_CODEC_OK);
    assert(output.priority == input.priority);
    assert(output.type == input.type);
    assert(output.sequence == input.sequence);
    assert(output.length == input.length);
    assert(memcmp(output.payload, payload, sizeof(payload)) == 0);
}

static void test_parser_fragmented_and_concatenated(void)
{
    const uint8_t first_payload[] = {1U, 2U, 3U};
    const uint8_t second_payload[] = {4U, 5U};
    uint8_t first[SRP_MAX_FRAME_SIZE] = {0};
    uint8_t second[SRP_MAX_FRAME_SIZE] = {0};
    uint8_t combined[SRP_MAX_FRAME_SIZE * 2U] = {0};
    uint16_t first_length = 0U;
    uint16_t second_length = 0U;
    srp_frame_t first_frame = {.priority = SRP_PRIORITY_TELEMETRY,
                               .type = SRP_MSG_ID_IMU_TELEMETRY,
                               .sequence = 0U,
                               .flags = 0U,
                               .length = sizeof(first_payload),
                               .payload = first_payload};
    srp_frame_t second_frame = {.priority = SRP_PRIORITY_LOG,
                                .type = SRP_MSG_ID_LOG,
                                .sequence = 1U,
                                .flags = 0U,
                                .length = sizeof(second_payload),
                                .payload = second_payload};
    srp_parser_t parser;

    assert(srp_encode(&first_frame, first, sizeof(first), &first_length) == 0);
    assert(srp_encode(&second_frame, second, sizeof(second), &second_length) == 0);
    memcpy(combined, first, first_length);
    memcpy(combined + first_length, second, second_length);
    s_frames = 0U;
    s_errors = 0U;
    srp_parser_init(&parser, on_frame, on_error, NULL);
    for (uint16_t index = 0U; index < first_length; ++index) {
        assert(srp_parser_feed(&parser, &combined[index], 1U) == 1U);
    }
    assert(s_frames == 1U && s_last_type == SRP_MSG_ID_IMU_TELEMETRY);
    assert(srp_parser_feed(&parser, combined + first_length, second_length) ==
           second_length);
    assert(s_frames == 2U && s_last_type == SRP_MSG_ID_LOG && s_errors == 0U);
}

static void test_sync_and_ack_golden_frames(void)
{
    static const uint8_t sync_payload[] = {4U, 0U, 0U, 0U};
    static const uint8_t ack_payload[] = {SRP_MSG_ID_CMD_SYNC_REQ, 0U,
                                          0x2AU, SRP_FAST_RESP_OK};
    static const uint8_t expected_sync[] = {
        0xAAU, 0x55U, 0x04U, 0x00U, 0x00U, 0x2AU, 0x08U, 0x01U,
        0x04U, 0x00U, 0x00U, 0x00U, 0x56U, 0xBCU, 0x0DU, 0x0AU
    };
    static const uint8_t expected_ack[] = {
        0xAAU, 0x55U, 0x04U, 0x00U, 0x04U, 0x2BU, 0x7EU, 0x01U,
        0x08U, 0x00U, 0x2AU, 0x00U, 0x38U, 0x65U, 0x0DU, 0x0AU
    };
    uint8_t bytes[SRP_MAX_FRAME_SIZE] = {0};
    uint16_t length = 0U;
    srp_frame_t frame = {.priority = SRP_PRIORITY_COMMAND,
                         .type = SRP_MSG_ID_CMD_SYNC_REQ,
                         .sequence = 0x2AU,
                         .flags = SRP_FLAG_STREAM_DATA,
                         .length = sizeof(sync_payload),
                         .payload = sync_payload};
    srp_parser_t parser;

    assert(srp_encode(&frame, bytes, sizeof(bytes), &length) == SRP_CODEC_OK);
    assert(length == sizeof(expected_sync));
    assert(memcmp(bytes, expected_sync, sizeof(expected_sync)) == 0);
    assert(bytes[12] == 0x56U && bytes[13] == 0xBCU);
    assert(srp_decode(bytes, length, &frame) == SRP_CODEC_OK);

    frame.type = SRP_MSG_ID_ACK;
    frame.sequence = 0x2BU;
    frame.flags = SRP_FLAG_ACK;
    frame.payload = ack_payload;
    assert(srp_encode(&frame, bytes, sizeof(bytes), &length) == SRP_CODEC_OK);
    assert(length == sizeof(expected_ack));
    assert(memcmp(bytes, expected_ack, sizeof(expected_ack)) == 0);
    assert(bytes[12] == 0x38U && bytes[13] == 0x65U);
    assert(srp_decode(bytes, length, &frame) == SRP_CODEC_OK);

    bytes[12] ^= 0x01U;
    s_errors = 0U;
    s_last_error = 0;
    srp_parser_init(&parser, on_frame, on_error, NULL);
    (void)srp_parser_feed(&parser, bytes, sizeof(expected_ack));
    assert(s_errors == 1U && s_last_error == SRP_PARSER_ERROR_CRC);
}

static void test_startup_sync_golden_frame(void)
{
    static const uint8_t payload[] = {4U, 0U, 0U, 0U};
    static const uint8_t expected[] = {
        0xAAU, 0x55U, 0x04U, 0x00U, 0x00U, 0x00U, 0x08U, 0x01U,
        0x04U, 0x00U, 0x00U, 0x00U, 0xEEU, 0x21U, 0x0DU, 0x0AU
    };
    uint8_t bytes[SRP_MAX_FRAME_SIZE] = {0};
    uint16_t length = 0U;
    const srp_frame_t frame = {
        .priority = SRP_PRIORITY_COMMAND,
        .type = SRP_MSG_ID_CMD_SYNC_REQ,
        .sequence = 0U,
        .flags = SRP_FLAG_STREAM_DATA,
        .length = sizeof(payload),
        .payload = payload,
    };

    assert(srp_encode_frame(&frame, bytes, sizeof(bytes), &length) == SRP_CODEC_OK);
    assert(length == sizeof(expected));
    assert(memcmp(bytes, expected, sizeof(expected)) == 0);
}

static void test_chassis_heading_command_payload(void)
{
    uint8_t payload[SRP_PAYLOAD_CHASSIS_HEADING_CMD_SIZE] = {0};
    uint8_t bytes[SRP_MAX_FRAME_SIZE] = {0};
    uint16_t length = 0U;
    srp_frame_t decoded = {0};
    const srp_frame_t frame = {
        .priority = SRP_PRIORITY_COMMAND,
        .type = SRP_MSG_ID_CHASSIS_HEADING_CMD,
        .sequence = 0x31U,
        .flags = SRP_FLAG_ACK_REQUIRED,
        .length = sizeof(payload),
        .payload = payload,
    };

    srp_wire_write_f32_le(&payload[0], 320.0f);
    srp_wire_write_f32_le(&payload[4], -179.5f);
    srp_wire_write_u32_le(&payload[8], SRP_CHASSIS_HEADING_FLAGS_NONE);
    assert(sizeof(payload) == sizeof(srp_chassis_heading_cmd_payload_t));
    assert(srp_encode(&frame, bytes, sizeof(bytes), &length) == SRP_CODEC_OK);
    assert(srp_decode(bytes, length, &decoded) == SRP_CODEC_OK);
    assert(decoded.type == SRP_MSG_ID_CHASSIS_HEADING_CMD);
    assert(decoded.flags == SRP_FLAG_ACK_REQUIRED);
    assert(decoded.length == SRP_PAYLOAD_CHASSIS_HEADING_CMD_SIZE);
    assert(srp_wire_read_f32_le(&decoded.payload[0]) == 320.0f);
    assert(srp_wire_read_f32_le(&decoded.payload[4]) == -179.5f);
    assert(srp_wire_read_u32_le(&decoded.payload[8]) ==
           SRP_CHASSIS_HEADING_FLAGS_NONE);
}

static void test_crc_and_eof_rejection(void)
{
    const uint8_t payload[] = {0xABU};
    uint8_t bytes[SRP_MAX_FRAME_SIZE] = {0};
    uint16_t length = 0U;
    srp_frame_t frame = {.priority = SRP_PRIORITY_COMMAND,
                         .type = SRP_MSG_ID_MOTOR_CMD,
                         .sequence = 0U,
                         .flags = 0U,
                         .length = sizeof(payload),
                         .payload = payload};
    srp_parser_t parser;

    assert(srp_encode(&frame, bytes, sizeof(bytes), &length) == 0);
    bytes[8] ^= 0x01U;
    assert(srp_decode(bytes, length, &frame) == SRP_CODEC_BAD_CRC);
    assert(srp_encode(&frame, bytes, sizeof(bytes), &length) == 0);
    bytes[length - 1U] = 0x00U;
    assert(srp_decode(bytes, length, &frame) == SRP_CODEC_BAD_EOF);
    s_errors = 0U;
    srp_parser_init(&parser, on_frame, on_error, NULL);
    (void)srp_parser_feed(&parser, bytes, length);
    assert(s_errors == 1U);
}

static void test_header_rejection_diagnostics(void)
{
    const uint8_t payload[] = {0x01U};
    uint8_t bytes[SRP_MAX_FRAME_SIZE] = {0};
    uint16_t length = 0U;
    srp_frame_t frame = {.priority = SRP_PRIORITY_COMMAND,
                         .type = SRP_MSG_ID_MOTOR_CMD,
                         .sequence = 0U,
                         .flags = 0U,
                         .length = sizeof(payload),
                         .payload = payload};
    srp_parser_t parser;

    assert(srp_encode(&frame, bytes, sizeof(bytes), &length) == SRP_CODEC_OK);
    bytes[7] |= SRP_FLAG_RESERVED_MASK;
    s_errors = 0U;
    s_last_error = 0;
    srp_parser_init(&parser, on_frame, on_error, NULL);
    (void)srp_parser_feed(&parser, bytes, length);
    assert(s_errors == 1U && s_last_error == SRP_PARSER_ERROR_HEADER);
    assert(parser.last_error_state == SRP_PARSER_READ_HEADER);
    assert(parser.last_drop_byte == bytes[7]);
}

static void test_parser_reset_after_discontinuity(void)
{
    const uint8_t payload[] = {0x01U, 0x02U};
    uint8_t bytes[SRP_MAX_FRAME_SIZE] = {0};
    uint16_t length = 0U;
    srp_frame_t frame = {.priority = SRP_PRIORITY_COMMAND,
                         .type = SRP_MSG_ID_RSP_BOOT_INFO,
                         .sequence = 2U,
                         .flags = SRP_FLAG_STREAM_DATA,
                         .length = sizeof(payload),
                         .payload = payload};
    srp_parser_t parser;

    assert(srp_encode(&frame, bytes, sizeof(bytes), &length) == SRP_CODEC_OK);
    s_frames = 0U;
    srp_parser_init(&parser, on_frame, on_error, NULL);
    (void)srp_parser_feed(&parser, bytes, 4U);
    srp_parser_reset(&parser);
    (void)srp_parser_feed(&parser, bytes + 4U, length - 4U);
    assert(s_frames == 0U);
    (void)srp_parser_feed(&parser, bytes, length);
    assert(s_frames == 1U && s_last_type == SRP_MSG_ID_RSP_BOOT_INFO);
}

static void test_unknown_tlv_is_skipped(void)
{
    const uint8_t data[] = {SRP_TLV_TAG_BAUDRATE, 4U, 0x00U, 0x10U, 0x0EU, 0x00U,
                            0xF0U, 1U, 0xAAU};
    srp_tlv_iter_t iterator;
    uint8_t tag = 0U;
    uint8_t length = 0U;
    const uint8_t *value = NULL;

    srp_tlv_iter_init(&iterator, data, sizeof(data));
    assert(srp_tlv_next(&iterator, &tag, &length, &value));
    assert(tag == SRP_TLV_TAG_BAUDRATE && length == 4U && value[0] == 0x00U);
    assert(srp_tlv_next(&iterator, &tag, &length, &value));
    assert(tag == 0xF0U && length == 1U && value[0] == 0xAAU);
    assert(!srp_tlv_next(&iterator, &tag, &length, &value));
}

static void test_alignment_and_sequence_wrap(void)
{
    assert(_Alignof(srp_wire_header_t) >= 4U);
    assert(offsetof(srp_wire_header_t, header) == 4U);
    assert(_Alignof(srp_parser_t) >= 4U);
    assert(SRP_HDR_SEQ(SRP_HDR_MAKE(0U, 1U, 0xFFU, 0U)) == 0xFFU);
    assert(SRP_HDR_SEQ(SRP_HDR_MAKE(0U, 1U, 0x00U, 0U)) == 0x00U);
}

int main(void)
{
    test_crc_vector();
    test_encode_decode();
    test_parser_fragmented_and_concatenated();
    test_sync_and_ack_golden_frames();
    test_startup_sync_golden_frame();
    test_chassis_heading_command_payload();
    test_crc_and_eof_rejection();
    test_header_rejection_diagnostics();
    test_parser_reset_after_discontinuity();
    test_unknown_tlv_is_skipped();
    test_alignment_and_sequence_wrap();
    return 0;
}
