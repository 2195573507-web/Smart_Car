#include "ros_motion_stream.h"

#include <string.h>

static void drop_prefix(ros_motion_stream_t *stream, size_t count)
{
    if (count >= stream->length) {
        stream->length = 0U;
        return;
    }
    memmove(stream->buffer, &stream->buffer[count], stream->length - count);
    stream->length -= count;
}

void ros_motion_stream_init(ros_motion_stream_t *stream)
{
    if (stream != NULL) {
        stream->length = 0U;
    }
}

size_t ros_motion_stream_feed(ros_motion_stream_t *stream,
                               const uint8_t *data, size_t length,
                               ros_motion_auth_fn auth_fn, void *auth_context,
                               ros_motion_stream_frame_fn frame_fn,
                               ros_motion_stream_error_fn error_fn,
                               void *context)
{
    size_t frames = 0U;

    if (stream == NULL || (data == NULL && length != 0U) || auth_fn == NULL) {
        return 0U;
    }
    for (size_t index = 0U; index < length; ++index) {
        if (stream->length == sizeof(stream->buffer)) {
            if (error_fn != NULL) {
                error_fn(ROS_MOTION_BAD_LENGTH, context);
            }
            stream->length = 0U;
        }
        stream->buffer[stream->length++] = data[index];
        while (stream->length >= ROS_MOTION_FIXED_HEADER_SIZE +
                                  ROS_MOTION_AUTH_TAG_SIZE +
                                  ROS_MOTION_CRC_SIZE) {
            if (((uint16_t)stream->buffer[0] |
                 ((uint16_t)stream->buffer[1] << 8U)) != ROS_MOTION_MAGIC) {
                drop_prefix(stream, 1U);
                continue;
            }
            const size_t payload_length = (uint16_t)stream->buffer[6] |
                                           ((uint16_t)stream->buffer[7] << 8U);
            const size_t frame_length = ROS_MOTION_FIXED_HEADER_SIZE +
                                        payload_length +
                                        ROS_MOTION_AUTH_TAG_SIZE +
                                        ROS_MOTION_CRC_SIZE;
            if (payload_length > ROS_MOTION_MAX_PAYLOAD_SIZE ||
                frame_length > sizeof(stream->buffer)) {
                if (error_fn != NULL) {
                    error_fn(ROS_MOTION_BAD_LENGTH, context);
                }
                drop_prefix(stream, 1U);
                continue;
            }
            if (stream->length < frame_length) {
                break;
            }
            ros_motion_frame_view_t frame;
            const int status = ros_motion_decode(
                stream->buffer, frame_length, auth_fn, auth_context, &frame);
            if (status == ROS_MOTION_OK) {
                if (frame_fn != NULL) {
                    frame_fn(&frame, context);
                }
                ++frames;
                drop_prefix(stream, frame_length);
            } else {
                if (error_fn != NULL) {
                    error_fn(status, context);
                }
                drop_prefix(stream, 1U);
            }
        }
    }
    return frames;
}
