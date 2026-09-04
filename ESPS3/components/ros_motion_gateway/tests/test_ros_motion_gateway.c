#include "ros_motion_protocol.h"
#include "ros_motion_state.h"
#include "ros_motion_stream.h"

#include <assert.h>
#include <math.h>
#include <string.h>

typedef struct {
    const char *name;
    uint8_t type;
    const char *hex;
} golden_vector_t;

typedef struct {
    const char *name;
    const char *hex;
} invalid_vector_t;

typedef struct {
    const uint8_t *frame;
    size_t length;
} auth_context_t;

typedef struct {
    const uint8_t *frames[4];
    size_t lengths[4];
    size_t count;
} multi_auth_context_t;

typedef struct {
    unsigned frames;
    unsigned errors;
    uint8_t last_type;
} stream_result_t;

#include "ros_motion_golden_vectors.inc"

static const golden_vector_t *find_valid_vector(uint8_t type)
{
    for (size_t index = 0U; index < sizeof(k_valid_vectors) / sizeof(k_valid_vectors[0]);
         ++index) {
        if (k_valid_vectors[index].type == type) {
            return &k_valid_vectors[index];
        }
    }
    return NULL;
}

static const invalid_vector_t *find_invalid_vector(const char *name)
{
    for (size_t index = 0U;
         index < sizeof(k_invalid_vectors) / sizeof(k_invalid_vectors[0]); ++index) {
        if (strcmp(k_invalid_vectors[index].name, name) == 0) {
            return &k_invalid_vectors[index];
        }
    }
    return NULL;
}

static uint8_t hex_nibble(char value)
{
    if (value >= '0' && value <= '9') {
        return (uint8_t)(value - '0');
    }
    if (value >= 'a' && value <= 'f') {
        return (uint8_t)(value - 'a' + 10);
    }
    assert(value >= 'A' && value <= 'F');
    return (uint8_t)(value - 'A' + 10);
}

static size_t decode_hex(const char *hex, uint8_t *output, size_t capacity)
{
    const size_t hex_length = strlen(hex);

    assert((hex_length % 2U) == 0U);
    assert(hex_length / 2U <= capacity);
    for (size_t index = 0U; index < hex_length / 2U; ++index) {
        output[index] = (uint8_t)((hex_nibble(hex[index * 2U]) << 4U) |
                                  hex_nibble(hex[index * 2U + 1U]));
    }
    return hex_length / 2U;
}

static uint16_t read_u16_le(const uint8_t data[2])
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static bool vector_auth(const uint8_t *data, size_t length,
                        uint8_t tag[ROS_MOTION_AUTH_TAG_SIZE], void *context)
{
    const auth_context_t *expected = context;
    const size_t auth_length = expected->length - 2U - ROS_MOTION_AUTH_TAG_SIZE -
                               ROS_MOTION_CRC_SIZE;

    if (length != auth_length || memcmp(data, &expected->frame[2], length) != 0) {
        return false;
    }
    memcpy(tag, &expected->frame[2U + auth_length], ROS_MOTION_AUTH_TAG_SIZE);
    return true;
}

static bool multi_vector_auth(const uint8_t *data, size_t length,
                              uint8_t tag[ROS_MOTION_AUTH_TAG_SIZE], void *context)
{
    const multi_auth_context_t *expected = context;

    for (size_t index = 0U; index < expected->count; ++index) {
        const size_t auth_length = expected->lengths[index] - 2U -
                                   ROS_MOTION_AUTH_TAG_SIZE - ROS_MOTION_CRC_SIZE;
        if (length == auth_length &&
            memcmp(data, &expected->frames[index][2], length) == 0) {
            memcpy(tag, &expected->frames[index][2U + auth_length],
                   ROS_MOTION_AUTH_TAG_SIZE);
            return true;
        }
    }
    return false;
}

static void on_frame(const ros_motion_frame_view_t *frame, void *context)
{
    stream_result_t *result = context;

    ++result->frames;
    result->last_type = frame->type;
}

static void on_error(int status, void *context)
{
    stream_result_t *result = context;

    assert(status != ROS_MOTION_OK);
    ++result->errors;
}

static void test_golden_vectors(void)
{
    for (size_t index = 0U; index < sizeof(k_valid_vectors) / sizeof(k_valid_vectors[0]);
         ++index) {
        uint8_t input[ROS_MOTION_MAX_FRAME_SIZE];
        uint8_t output[ROS_MOTION_MAX_FRAME_SIZE];
        ros_motion_frame_view_t frame;
        const size_t input_length = decode_hex(k_valid_vectors[index].hex, input,
                                               sizeof(input));
        const auth_context_t auth = {.frame = input, .length = input_length};
        size_t output_length = 0U;

        assert(ros_motion_decode(input, input_length, vector_auth, (void *)&auth,
                                 &frame) == ROS_MOTION_OK);
        assert(frame.version == ROS_MOTION_PROTOCOL_VERSION);
        assert(frame.type == k_valid_vectors[index].type);
        assert(frame.flags == ROS_MOTION_FLAG_AUTH_PRESENT);
        assert(input[22] == 0U && input[23] == 0U);
        assert(read_u16_le(&input[input_length - ROS_MOTION_CRC_SIZE]) ==
               ros_motion_crc16(&input[2], input_length - 4U));
        assert(ros_motion_encode(&frame, vector_auth, (void *)&auth, output,
                                 sizeof(output), &output_length) == ROS_MOTION_OK);
        assert(output_length == input_length);
        assert(memcmp(output, input, input_length) == 0);
    }
}

static void test_float_and_negative_frames(void)
{
    uint8_t motion[ROS_MOTION_MAX_FRAME_SIZE];
    uint8_t invalid_crc[ROS_MOTION_MAX_FRAME_SIZE];
    uint8_t invalid_hmac[ROS_MOTION_MAX_FRAME_SIZE];
    uint8_t malformed[ROS_MOTION_MAX_FRAME_SIZE];
    float linear = 0.0f;
    float angular = 0.0f;
    ros_motion_frame_view_t frame;
    const golden_vector_t *motion_vector =
        find_valid_vector(ROS_MOTION_TYPE_MOTION_CMD);
    const invalid_vector_t *invalid_crc_vector = find_invalid_vector("invalid_crc");
    const invalid_vector_t *invalid_hmac_vector = find_invalid_vector("invalid_hmac");
    assert(motion_vector != NULL && invalid_crc_vector != NULL &&
           invalid_hmac_vector != NULL);
    const size_t motion_length = decode_hex(motion_vector->hex, motion,
                                            sizeof(motion));
    const auth_context_t auth = {.frame = motion, .length = motion_length};

    assert(ros_motion_read_f32_le(&motion[24], &linear));
    assert(ros_motion_read_f32_le(&motion[28], &angular));
    assert(fabsf(linear - 0.10f) < 0.000001f);
    assert(fabsf(angular + 0.30f) < 0.000001f);
    ros_motion_write_f32_le(&malformed[0], linear);
    ros_motion_write_f32_le(&malformed[4], angular);
    assert(memcmp(malformed, &motion[24], 8U) == 0);

    const size_t invalid_crc_length = decode_hex(invalid_crc_vector->hex, invalid_crc,
                                                 sizeof(invalid_crc));
    assert(ros_motion_decode(invalid_crc, invalid_crc_length, vector_auth,
                             (void *)&auth, &frame) == ROS_MOTION_BAD_CRC);
    const size_t invalid_hmac_length = decode_hex(invalid_hmac_vector->hex, invalid_hmac,
                                                  sizeof(invalid_hmac));
    assert(ros_motion_decode(invalid_hmac, invalid_hmac_length, vector_auth,
                             (void *)&auth, &frame) == ROS_MOTION_BAD_AUTH);

    memcpy(malformed, motion, motion_length);
    malformed[3] = 0x0AU;
    assert(ros_motion_decode(malformed, motion_length, vector_auth,
                             (void *)&auth, &frame) == ROS_MOTION_UNSUPPORTED);
    memcpy(malformed, motion, motion_length);
    malformed[22] = 1U;
    assert(ros_motion_decode(malformed, motion_length, vector_auth,
                             (void *)&auth, &frame) == ROS_MOTION_BAD_FLAGS);
    memcpy(malformed, motion, motion_length);
    malformed[6] = (uint8_t)(ROS_MOTION_MAX_PAYLOAD_SIZE + 1U);
    malformed[7] = 0U;
    assert(ros_motion_decode(malformed, motion_length, vector_auth,
                             (void *)&auth, &frame) == ROS_MOTION_BAD_LENGTH);
}

static void test_tcp_stream_recovery(void)
{
    uint8_t motion[ROS_MOTION_MAX_FRAME_SIZE];
    uint8_t heartbeat[ROS_MOTION_MAX_FRAME_SIZE];
    uint8_t hello_ack[ROS_MOTION_MAX_FRAME_SIZE];
    uint8_t invalid_crc[ROS_MOTION_MAX_FRAME_SIZE];
    ros_motion_stream_t stream;
    stream_result_t result = {0};
    uint8_t prefix = 0U;
    const golden_vector_t *motion_vector =
        find_valid_vector(ROS_MOTION_TYPE_MOTION_CMD);
    const golden_vector_t *heartbeat_vector =
        find_valid_vector(ROS_MOTION_TYPE_HEARTBEAT);
    const golden_vector_t *hello_ack_vector =
        find_valid_vector(ROS_MOTION_TYPE_HELLO_ACK);
    const invalid_vector_t *invalid_crc_vector = find_invalid_vector("invalid_crc");
    assert(motion_vector != NULL && heartbeat_vector != NULL &&
           hello_ack_vector != NULL && invalid_crc_vector != NULL);
    const size_t motion_length = decode_hex(motion_vector->hex, motion,
                                            sizeof(motion));
    const size_t heartbeat_length = decode_hex(heartbeat_vector->hex, heartbeat,
                                               sizeof(heartbeat));
    const size_t hello_ack_length = decode_hex(hello_ack_vector->hex, hello_ack,
                                               sizeof(hello_ack));
    const size_t invalid_crc_length = decode_hex(invalid_crc_vector->hex, invalid_crc,
                                                 sizeof(invalid_crc));
    const multi_auth_context_t auth = {
        .frames = {motion, heartbeat, hello_ack},
        .lengths = {motion_length, heartbeat_length, hello_ack_length},
        .count = 3U,
    };

    ros_motion_stream_init(&stream);
    assert(ros_motion_stream_feed(&stream, &prefix, 1U, multi_vector_auth,
                                  (void *)&auth, on_frame, on_error, &result) == 0U);
    assert(ros_motion_stream_feed(&stream, motion, 5U, multi_vector_auth,
                                  (void *)&auth, on_frame, on_error, &result) == 0U);
    assert(ros_motion_stream_feed(&stream, &motion[5], motion_length - 5U,
                                  multi_vector_auth, (void *)&auth, on_frame,
                                  on_error, &result) == 1U);
    assert(ros_motion_stream_feed(&stream, heartbeat, heartbeat_length,
                                  multi_vector_auth, (void *)&auth, on_frame,
                                  on_error, &result) == 1U);
    assert(ros_motion_stream_feed(&stream, hello_ack, hello_ack_length,
                                  multi_vector_auth, (void *)&auth, on_frame,
                                  on_error, &result) == 1U);
    assert(ros_motion_stream_feed(&stream, invalid_crc, invalid_crc_length,
                                  multi_vector_auth, (void *)&auth, on_frame,
                                  on_error, &result) == 0U);
    assert(ros_motion_stream_feed(&stream, heartbeat, heartbeat_length,
                                  multi_vector_auth, (void *)&auth, on_frame,
                                  on_error, &result) == 1U);
    assert(result.frames == 4U && result.errors >= 1U);
    assert(result.last_type == ROS_MOTION_TYPE_HEARTBEAT);
}

static void establish_lease(ros_motion_state_machine_t *state, uint32_t session,
                            uint32_t lease, uint32_t sequence, uint16_t ttl_ms,
                            uint64_t now_ms)
{
    ros_motion_state_connected(state, session);
    assert(ros_motion_state_hello_ack(state, session, sequence));
    assert(ros_motion_state_request_lease(state, session, lease, sequence + 1U,
                                          ttl_ms, now_ms));
}

static void test_state_machine(void)
{
    ros_motion_state_machine_t state;
    ros_motion_command_t command = {
        .linear_m_s = 0.10f,
        .angular_rad_s = -0.30f,
        .session_id = 0x55U,
        .lease_id = 0x77U,
        .sequence = 3U,
        .ttl_ms = 220U,
    };
    ros_motion_command_t latest;

    ros_motion_state_init(&state);
    ros_motion_state_connected(&state, 0U);
    assert(!ros_motion_state_hello_ack(&state, 0U, 1U));

    establish_lease(&state, 0x55U, 0x77U, 1U, 220U, 1000U);
    assert(ros_motion_state_accept_command(&state, &command, 1000U, 100, 300));
    command.sequence = 4U;
    command.linear_m_s = 0.09f;
    assert(ros_motion_state_accept_command(&state, &command, 1001U, 100, 300));
    assert(ros_motion_state_take_latest(&state, &latest));
    assert(latest.sequence == 4U);
    assert(!ros_motion_state_accept_command(&state, &command, 1002U, 100, 300));
    command.sequence = 5U;
    command.session_id = 0x56U;
    assert(!ros_motion_state_accept_command(&state, &command, 1002U, 100, 300));
    command.session_id = 0x55U;
    command.linear_m_s = 0.101f;
    assert(!ros_motion_state_accept_command(&state, &command, 1002U, 100, 300));
    assert(ros_motion_state_accept_heartbeat(&state, 0x55U, 0x77U, 5U, 1002U));
    assert(!ros_motion_state_expired(&state, 1221U));
    assert(ros_motion_state_expired(&state, 1222U));
    assert(!state.lease_active);

    establish_lease(&state, 0x56U, 0x78U, 1U, 20U, 2000U);
    assert(!ros_motion_state_accept_heartbeat(&state, 0x56U, 0x78U, 3U, 2020U));
    assert(!state.lease_active);

    establish_lease(&state, 0x57U, 0x79U, UINT32_MAX - 2U, 220U, 3000U);
    command = (ros_motion_command_t){
        .linear_m_s = 0.10f,
        .angular_rad_s = 0.30f,
        .session_id = 0x57U,
        .lease_id = 0x79U,
        .sequence = UINT32_MAX,
        .ttl_ms = 220U,
    };
    assert(ros_motion_state_accept_command(&state, &command, 3001U, 100, 300));
    command.sequence = 0U;
    assert(ros_motion_state_accept_command(&state, &command, 3002U, 100, 300));
    assert(!ros_motion_state_accept_command(&state, &command, 3003U, 100, 300));
    command.sequence = 1U;
    assert(ros_motion_state_accept_stop(&state, 0x57U, 0x79U, command.sequence,
                                         3004U));
    assert(!state.lease_active && state.state == ROS_MOTION_STATE_FAULT);
}

int main(void)
{
    test_golden_vectors();
    test_float_and_negative_frames();
    test_tcp_stream_recovery();
    test_state_machine();
    return 0;
}
