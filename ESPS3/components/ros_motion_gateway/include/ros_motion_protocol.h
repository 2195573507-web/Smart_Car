#ifndef ROS_MOTION_PROTOCOL_H
#define ROS_MOTION_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ROS_MOTION_PROTOCOL_VERSION UINT8_C(1)
#define ROS_MOTION_MAGIC UINT16_C(0x4D52)
#define ROS_MOTION_MAX_FRAME_SIZE UINT16_C(256)
#define ROS_MOTION_FIXED_HEADER_SIZE UINT16_C(24)
#define ROS_MOTION_AUTH_TAG_SIZE UINT16_C(16)
#define ROS_MOTION_CRC_SIZE UINT16_C(2)
#define ROS_MOTION_MAX_PAYLOAD_SIZE \
    (ROS_MOTION_MAX_FRAME_SIZE - ROS_MOTION_FIXED_HEADER_SIZE - \
     ROS_MOTION_AUTH_TAG_SIZE - ROS_MOTION_CRC_SIZE)

#define ROS_MOTION_FLAG_AUTH_PRESENT UINT16_C(0x0001)

typedef enum {
    ROS_MOTION_TYPE_HELLO = 0x01,
    ROS_MOTION_TYPE_HELLO_ACK = 0x02,
    ROS_MOTION_TYPE_LEASE_REQUEST = 0x03,
    ROS_MOTION_TYPE_LEASE_RESPONSE = 0x04,
    ROS_MOTION_TYPE_MOTION_CMD = 0x05,
    ROS_MOTION_TYPE_STOP = 0x06,
    ROS_MOTION_TYPE_STATUS = 0x07,
    ROS_MOTION_TYPE_ERROR = 0x08,
    ROS_MOTION_TYPE_HEARTBEAT = 0x09
} ros_motion_message_type_t;

typedef enum {
    ROS_MOTION_OK = 0,
    ROS_MOTION_INVALID_ARG = -1,
    ROS_MOTION_BAD_MAGIC = -2,
    ROS_MOTION_BAD_VERSION = -3,
    ROS_MOTION_BAD_LENGTH = -4,
    ROS_MOTION_BAD_FLAGS = -5,
    ROS_MOTION_BAD_CRC = -6,
    ROS_MOTION_BAD_AUTH = -7,
    ROS_MOTION_UNSUPPORTED = -8
} ros_motion_protocol_status_t;

typedef struct {
    uint8_t version;
    uint8_t type;
    uint16_t flags;
    uint16_t payload_length;
    uint32_t session_id;
    uint32_t sequence;
    uint32_t lease_id;
    uint16_t ttl_ms;
    const uint8_t *payload;
    const uint8_t *auth_tag;
} ros_motion_frame_view_t;

typedef bool (*ros_motion_auth_fn)(const uint8_t *data, size_t length,
                                   uint8_t tag[ROS_MOTION_AUTH_TAG_SIZE],
                                   void *context);

uint16_t ros_motion_crc16(const uint8_t *data, size_t length);
bool ros_motion_message_type_is_valid(uint8_t type);
bool ros_motion_read_f32_le(const uint8_t data[4], float *value);
void ros_motion_write_f32_le(uint8_t data[4], float value);

int ros_motion_encode(const ros_motion_frame_view_t *frame,
                      ros_motion_auth_fn auth_fn, void *auth_context,
                      uint8_t *output, size_t capacity, size_t *output_length);

int ros_motion_decode(const uint8_t *data, size_t length,
                      ros_motion_auth_fn auth_fn, void *auth_context,
                      ros_motion_frame_view_t *frame);

const char *ros_motion_protocol_status_name(int status);

#endif /* ROS_MOTION_PROTOCOL_H */
