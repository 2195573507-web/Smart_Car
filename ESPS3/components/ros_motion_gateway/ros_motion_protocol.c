#include "ros_motion_protocol.h"

#include <float.h>
#include <string.h>

_Static_assert(sizeof(float) == 4U && FLT_RADIX == 2 && FLT_MANT_DIG == 24 &&
                   FLT_MAX_EXP == 128,
               "ROS Motion Control v1 requires IEEE754 binary32 float");

static void write_u16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8U);
}

static void write_u32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8U);
    dst[2] = (uint8_t)(value >> 16U);
    dst[3] = (uint8_t)(value >> 24U);
}

static uint16_t read_u16(const uint8_t *src)
{
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8U);
}

static uint32_t read_u32(const uint8_t *src)
{
    return (uint32_t)src[0] | ((uint32_t)src[1] << 8U) |
           ((uint32_t)src[2] << 16U) | ((uint32_t)src[3] << 24U);
}

bool ros_motion_message_type_is_valid(uint8_t type)
{
    return type >= ROS_MOTION_TYPE_HELLO && type <= ROS_MOTION_TYPE_HEARTBEAT;
}

bool ros_motion_read_f32_le(const uint8_t data[4], float *value)
{
    uint32_t bits;

    if (data == NULL || value == NULL) {
        return false;
    }
    bits = read_u32(data);
    memcpy(value, &bits, sizeof(bits));
    return true;
}

void ros_motion_write_f32_le(uint8_t data[4], float value)
{
    uint32_t bits;

    if (data == NULL) {
        return;
    }
    memcpy(&bits, &value, sizeof(bits));
    write_u32(data, bits);
}

uint16_t ros_motion_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = UINT16_C(0xFFFF);

    if (data == NULL && length != 0U) {
        return 0U;
    }
    for (size_t index = 0U; index < length; ++index) {
        crc ^= (uint16_t)data[index] << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & UINT16_C(0x8000)) != 0U
                      ? (uint16_t)((crc << 1U) ^ UINT16_C(0x1021))
                      : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

static int validate_header(const uint8_t *data, size_t length)
{
    if (data == NULL || length < ROS_MOTION_FIXED_HEADER_SIZE +
                                      ROS_MOTION_AUTH_TAG_SIZE +
                                      ROS_MOTION_CRC_SIZE) {
        return ROS_MOTION_BAD_LENGTH;
    }
    if (read_u16(data) != ROS_MOTION_MAGIC) {
        return ROS_MOTION_BAD_MAGIC;
    }
    if (data[2] != ROS_MOTION_PROTOCOL_VERSION) {
        return ROS_MOTION_BAD_VERSION;
    }
    if (!ros_motion_message_type_is_valid(data[3])) {
        return ROS_MOTION_UNSUPPORTED;
    }
    if ((read_u16(&data[4]) & (uint16_t)~ROS_MOTION_FLAG_AUTH_PRESENT) != 0U ||
        (read_u16(&data[4]) & ROS_MOTION_FLAG_AUTH_PRESENT) == 0U) {
        return ROS_MOTION_BAD_FLAGS;
    }
    if (read_u16(&data[22]) != 0U) {
        return ROS_MOTION_BAD_FLAGS;
    }
    const size_t payload_length = read_u16(&data[6]);
    const size_t expected = ROS_MOTION_FIXED_HEADER_SIZE + payload_length +
                            ROS_MOTION_AUTH_TAG_SIZE + ROS_MOTION_CRC_SIZE;
    if (payload_length > ROS_MOTION_MAX_PAYLOAD_SIZE || length != expected) {
        return ROS_MOTION_BAD_LENGTH;
    }
    return ROS_MOTION_OK;
}

static bool auth_tags_equal(const uint8_t left[ROS_MOTION_AUTH_TAG_SIZE],
                            const uint8_t right[ROS_MOTION_AUTH_TAG_SIZE])
{
    volatile uint8_t difference = 0U;

    for (size_t index = 0U; index < ROS_MOTION_AUTH_TAG_SIZE; ++index) {
        difference |= (uint8_t)(left[index] ^ right[index]);
    }
    return difference == 0U;
}

int ros_motion_encode(const ros_motion_frame_view_t *frame,
                      ros_motion_auth_fn auth_fn, void *auth_context,
                      uint8_t *output, size_t capacity, size_t *output_length)
{
    size_t total;

    if (frame == NULL || output == NULL || output_length == NULL ||
        (frame->payload == NULL && frame->payload_length != 0U) ||
        auth_fn == NULL || frame->version != ROS_MOTION_PROTOCOL_VERSION ||
        !ros_motion_message_type_is_valid(frame->type) ||
        frame->payload_length > ROS_MOTION_MAX_PAYLOAD_SIZE ||
        (frame->flags & ROS_MOTION_FLAG_AUTH_PRESENT) == 0U ||
        (frame->flags & (uint16_t)~ROS_MOTION_FLAG_AUTH_PRESENT) != 0U) {
        return ROS_MOTION_INVALID_ARG;
    }
    total = ROS_MOTION_FIXED_HEADER_SIZE + frame->payload_length +
            ROS_MOTION_AUTH_TAG_SIZE + ROS_MOTION_CRC_SIZE;
    if (capacity < total) {
        return ROS_MOTION_BAD_LENGTH;
    }
    memset(output, 0, total);
    write_u16(output, ROS_MOTION_MAGIC);
    output[2] = frame->version;
    output[3] = frame->type;
    write_u16(&output[4], frame->flags);
    write_u16(&output[6], frame->payload_length);
    write_u32(&output[8], frame->session_id);
    write_u32(&output[12], frame->sequence);
    write_u32(&output[16], frame->lease_id);
    write_u16(&output[20], frame->ttl_ms);
    if (frame->payload_length != 0U) {
        memcpy(&output[ROS_MOTION_FIXED_HEADER_SIZE], frame->payload,
               frame->payload_length);
    }
    if (!auth_fn(&output[2], ROS_MOTION_FIXED_HEADER_SIZE - 2U +
                              frame->payload_length,
                 &output[ROS_MOTION_FIXED_HEADER_SIZE + frame->payload_length],
                 auth_context)) {
        return ROS_MOTION_BAD_AUTH;
    }
    write_u16(&output[total - ROS_MOTION_CRC_SIZE],
              ros_motion_crc16(&output[2], total - ROS_MOTION_CRC_SIZE - 2U));
    *output_length = total;
    return ROS_MOTION_OK;
}

int ros_motion_decode(const uint8_t *data, size_t length,
                      ros_motion_auth_fn auth_fn, void *auth_context,
                      ros_motion_frame_view_t *frame)
{
    int status;
    size_t payload_length;
    size_t auth_offset;

    if (frame == NULL || data == NULL || auth_fn == NULL) {
        return ROS_MOTION_INVALID_ARG;
    }
    status = validate_header(data, length);
    if (status != ROS_MOTION_OK) {
        return status;
    }
    payload_length = read_u16(&data[6]);
    auth_offset = ROS_MOTION_FIXED_HEADER_SIZE + payload_length;
    const size_t total = ROS_MOTION_FIXED_HEADER_SIZE + payload_length +
                         ROS_MOTION_AUTH_TAG_SIZE + ROS_MOTION_CRC_SIZE;
    if (read_u16(&data[total - ROS_MOTION_CRC_SIZE]) !=
        ros_motion_crc16(&data[2], total - ROS_MOTION_CRC_SIZE - 2U)) {
        return ROS_MOTION_BAD_CRC;
    }
    uint8_t expected_tag[ROS_MOTION_AUTH_TAG_SIZE];
    if (!auth_fn(&data[2], ROS_MOTION_FIXED_HEADER_SIZE - 2U + payload_length,
                 expected_tag, auth_context) ||
        !auth_tags_equal(expected_tag, &data[auth_offset])) {
        return ROS_MOTION_BAD_AUTH;
    }
    frame->version = data[2];
    frame->type = data[3];
    frame->flags = read_u16(&data[4]);
    frame->payload_length = (uint16_t)payload_length;
    frame->session_id = read_u32(&data[8]);
    frame->sequence = read_u32(&data[12]);
    frame->lease_id = read_u32(&data[16]);
    frame->ttl_ms = read_u16(&data[20]);
    frame->payload = &data[ROS_MOTION_FIXED_HEADER_SIZE];
    frame->auth_tag = &data[auth_offset];
    return ROS_MOTION_OK;
}

const char *ros_motion_protocol_status_name(int status)
{
    switch (status) {
    case ROS_MOTION_OK: return "OK";
    case ROS_MOTION_BAD_MAGIC: return "BAD_MAGIC";
    case ROS_MOTION_BAD_VERSION: return "BAD_VERSION";
    case ROS_MOTION_BAD_LENGTH: return "BAD_LENGTH";
    case ROS_MOTION_BAD_FLAGS: return "BAD_FLAGS";
    case ROS_MOTION_BAD_CRC: return "BAD_CRC";
    case ROS_MOTION_BAD_AUTH: return "BAD_AUTH";
    case ROS_MOTION_UNSUPPORTED: return "UNSUPPORTED";
    default: return "INVALID_ARG";
    }
}
