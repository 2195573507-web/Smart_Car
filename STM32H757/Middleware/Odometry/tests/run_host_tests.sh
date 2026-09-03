#!/usr/bin/env sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
ODOMETRY_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
BUILD_DIR=${TMPDIR:-/tmp}/smartcar-chassis-odometry-tests

mkdir -p "$BUILD_DIR"
cc -std=c11 -Wall -Wextra -Werror -pedantic \
   -I"$ODOMETRY_DIR" \
   "$ODOMETRY_DIR/chassis_odometry.c" \
   "$SCRIPT_DIR/test_chassis_odometry.c" \
   -lm \
   -o "$BUILD_DIR/test_chassis_odometry"
"$BUILD_DIR/test_chassis_odometry"
