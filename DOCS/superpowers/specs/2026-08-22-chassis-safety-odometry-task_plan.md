# Task Plan: CM7 Chassis Heading, Attitude Safety, and Odometry

## Goal

Implement the approved chassis heading lock, attitude safety fuse, 50 ms
wheel-speed plus IMU odometry, and the `0x211` / `0x29` telemetry path without
changing tuned wheel-control parameters or the startup safety state machine.

## Phases

- [x] Phase 1: Inspect live CM7, S3, App, MotorBoard, and SCBP-CAN boundaries
- [x] Phase 2: Freeze the approved protocol and control design
- [x] Phase 3: Implement CM7 control, safety, odometry, and telemetry
- [x] Phase 4: Implement S3 relay and App decoding/display
- [x] Phase 5: Build and report cross-endpoint evidence

## Constraints

- Preserve `Core/Inc/wheel_control_params.h` unchanged.
- Preserve `attitude_startup_coordinator` lifecycle criteria and initial
  zero-PWM gate.
- Use calibrated `MSPD` actual speeds, not unscaled `MTEP` pulse counts, for
  odometry displacement integration.
- Keep the existing `0x201` DualAHRS schema-2 and `0x210` wheel-speed payloads
  byte-for-byte unchanged.

## Verification Evidence

- Shared SCBP-CAN host test: passed with the `0x211` exact payload-layout,
  little-endian, and frame round-trip assertions.
- CM7: clean Debug configure/build passed with no compiler diagnostics;
  `Smart_Car_H757_CM7.elf` reports Flash `179908 B`, RAM `59960 B`.
- ESP32-S3: ESP-IDF `5.5.4` `fullclean` build passed; the app image is
  `0xb1bd0 B`, leaving `0x64e430 B` of its 7 MiB OTA partition free.
- macOS App: `swift build` passed. `swift test` cannot run because this package
  has no test target (`error: no tests found`).
- `git diff --check` passed, and protected wheel-control and startup-state
  files have no diff.

## Physical Acceptance Still Required

- Flash matching CM7 and S3 images, then capture `0x211` over UART and BLE
  type `0x29` in the App.
- Verify tilt trip/zero-PWM and below-15-degree zero-command recovery on the
  vehicle.
- Check heading correction sign and tune acceptance during forward and reverse
  driving, then validate the odometry coordinate convention and distance scale.
