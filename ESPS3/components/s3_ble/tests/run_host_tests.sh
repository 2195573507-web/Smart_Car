#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
COMPONENT_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
COMMON_LOG_DIR=$(CDPATH= cd -- "$COMPONENT_DIR/../../../Common/SmartCarLog" && pwd)
BUILD_DIR=${TMPDIR:-/tmp}/smartcar-s3-ble-log-tx-tests

mkdir -p "$BUILD_DIR"

cc -std=c11 -Wall -Wextra -Werror \
   -I"$COMPONENT_DIR" \
   -I"$COMMON_LOG_DIR" \
   "$COMPONENT_DIR/s3_ble_log_tx.c" \
   "$SCRIPT_DIR/test_s3_ble_log_tx.c" \
   -o "$BUILD_DIR/test_s3_ble_log_tx"
"$BUILD_DIR/test_s3_ble_log_tx"

cc -std=c11 -Wall -Wextra -Werror \
   -fsanitize=address,undefined -fno-omit-frame-pointer \
   -I"$COMPONENT_DIR" \
   -I"$COMMON_LOG_DIR" \
   "$COMPONENT_DIR/s3_ble_log_tx.c" \
   "$SCRIPT_DIR/test_s3_ble_log_tx.c" \
   -o "$BUILD_DIR/test_s3_ble_log_tx_sanitized"
ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1 \
    "$BUILD_DIR/test_s3_ble_log_tx_sanitized"
