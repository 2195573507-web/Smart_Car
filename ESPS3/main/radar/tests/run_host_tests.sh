#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
RADAR_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
COMMON_SRP_DIR=$(CDPATH= cd -- "$RADAR_DIR/../../../Common/SRP" && pwd)
BUILD_DIR=${TMPDIR:-/tmp}/smartcar-radar-parser-tests

mkdir -p "$BUILD_DIR"
cc -std=c11 -Wall -Wextra -Werror \
   -I"$RADAR_DIR" \
   "$RADAR_DIR/radar_parser.c" \
   "$SCRIPT_DIR/test_radar_parser.c" \
   -o "$BUILD_DIR/test_radar_parser"
"$BUILD_DIR/test_radar_parser"

cc -std=c11 -Wall -Wextra -Werror \
   -I"$RADAR_DIR" \
   "$RADAR_DIR/radar_frame_fifo.c" \
   "$SCRIPT_DIR/test_radar_frame_fifo.c" \
   -o "$BUILD_DIR/test_radar_frame_fifo"
"$BUILD_DIR/test_radar_frame_fifo"

cc -std=c11 -Wall -Wextra -Werror \
   -I"$RADAR_DIR" \
   -I"$COMMON_SRP_DIR/include" \
   "$RADAR_DIR/radar_parser.c" \
   "$RADAR_DIR/radar_uplink_protocol.c" \
   "$SCRIPT_DIR/test_radar_uplink_protocol.c" \
   -o "$BUILD_DIR/test_radar_uplink_protocol"
"$BUILD_DIR/test_radar_uplink_protocol"

cc -std=c11 -Wall -Wextra -Werror \
   -I"$RADAR_DIR" \
   "$RADAR_DIR/radar_uplink_tx.c" \
   "$SCRIPT_DIR/test_radar_uplink_tx.c" \
   -o "$BUILD_DIR/test_radar_uplink_tx"
"$BUILD_DIR/test_radar_uplink_tx"

cc -std=c11 -Wall -Wextra -Werror \
   -I"$RADAR_DIR" \
   -I"$COMMON_SRP_DIR/include" \
   "$COMMON_SRP_DIR/srp_crc.c" \
   "$COMMON_SRP_DIR/srp_codec.c" \
   "$COMMON_SRP_DIR/srp_wire.c" \
   "$RADAR_DIR/radar_telemetry_queue.c" \
   "$SCRIPT_DIR/test_radar_telemetry_queue.c" \
   -lm \
   -o "$BUILD_DIR/test_radar_telemetry_queue"
"$BUILD_DIR/test_radar_telemetry_queue"
