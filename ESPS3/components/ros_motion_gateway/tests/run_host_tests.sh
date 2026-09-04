#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
COMPONENT_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
PROJECT_ROOT=$(CDPATH= cd -- "$COMPONENT_DIR/../../.." && pwd)
BUILD_DIR=${TMPDIR:-/tmp}/smartcar-ros-motion-tests
GOLDEN_VECTORS=$PROJECT_ROOT/DOCS/protocol/ros-motion-control-v1-golden-vectors.json

mkdir -p "$BUILD_DIR"
jq -r '
  "static const golden_vector_t k_valid_vectors[] = {",
  (.valid_frames[] | "    {\"\(.name)\", \(.type)U, \"\(.frame_hex)\"},"),
  "};",
  "static const invalid_vector_t k_invalid_vectors[] = {",
  (.invalid_frames[] | "    {\"\(.name)\", \"\(.frame_hex)\"},"),
  "};"
' "$GOLDEN_VECTORS" > "$BUILD_DIR/ros_motion_golden_vectors.inc"

cc -std=c11 -Wall -Wextra -Werror -I"$COMPONENT_DIR/include" \
   -I"$BUILD_DIR" \
   "$COMPONENT_DIR/ros_motion_protocol.c" \
   "$COMPONENT_DIR/ros_motion_stream.c" \
   "$COMPONENT_DIR/ros_motion_state.c" \
   "$SCRIPT_DIR/test_ros_motion_gateway.c" -lm \
   -o "$BUILD_DIR/test_ros_motion_gateway"
"$BUILD_DIR/test_ros_motion_gateway"

cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
   -fno-omit-frame-pointer -I"$COMPONENT_DIR/include" \
   -I"$BUILD_DIR" \
   "$COMPONENT_DIR/ros_motion_protocol.c" \
   "$COMPONENT_DIR/ros_motion_stream.c" \
   "$COMPONENT_DIR/ros_motion_state.c" \
   "$SCRIPT_DIR/test_ros_motion_gateway.c" -lm \
   -o "$BUILD_DIR/test_ros_motion_gateway_sanitized"
ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1 "$BUILD_DIR/test_ros_motion_gateway_sanitized"
