#ifndef ROS_MOTION_STREAM_H
#define ROS_MOTION_STREAM_H

#include <stddef.h>
#include <stdint.h>

#include "ros_motion_protocol.h"

typedef struct {
    uint8_t buffer[ROS_MOTION_MAX_FRAME_SIZE];
    size_t length;
} ros_motion_stream_t;

typedef void (*ros_motion_stream_frame_fn)(
    const ros_motion_frame_view_t *frame, void *context);
typedef void (*ros_motion_stream_error_fn)(int status, void *context);

void ros_motion_stream_init(ros_motion_stream_t *stream);
size_t ros_motion_stream_feed(ros_motion_stream_t *stream,
                               const uint8_t *data, size_t length,
                               ros_motion_auth_fn auth_fn, void *auth_context,
                               ros_motion_stream_frame_fn frame_fn,
                               ros_motion_stream_error_fn error_fn,
                               void *context);

#endif /* ROS_MOTION_STREAM_H */
