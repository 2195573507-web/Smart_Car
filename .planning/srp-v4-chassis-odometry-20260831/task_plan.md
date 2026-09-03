# Task Plan: SRP v4 Chassis Odometry

## Goal

Produce SRP v4 `CHASSIS_STATE` (`0x15`) on CM7 and relay it through the
existing ESP32-S3 S3RD telemetry path, without changing motion-control or
radar contracts.

## Frozen Boundaries

- Wheel order remains `[RR, RF, LR, LF]`.
- RF encoder polarity remains corrected exactly once in MotorBoard feedback.
- Track width, trim, PID, ramp, safety gates, stop paths, and radar S3RD raw
  frames are unchanged.
- Odometry uses calibrated MSPD feedback plus fresh Primary DualAHRS yaw.
- `CHASSIS_STATE` remains schema 1 and 24 bytes.
- No firmware flashing or vehicle acceptance is part of this task.

## Phases

- [x] Inspect current SRP, MotorBoard, DualAHRS, scheduling, and S3 queue paths.
- [x] Confirm the previously approved MSPD plus Primary-Yaw design.
- [x] Write the SRP v4 migration design.
- [x] Add shared contract assertions and golden-vector coverage.
- [x] Add atomic MotorBoard feedback freshness snapshot.
- [x] Add pure odometry module and low-priority chassis-state publisher task.
- [x] Add S3 validation, latest-only queueing, and TCP relay.
- [x] Run host tests, CM7 Debug build, ESP-IDF 5.5.4 build, and diff checks.
- [x] Record source/build evidence separately from hardware acceptance.

## Error Log

- The first odometry host-test compile lacked `<stddef.h>` for `size_t`; the
  standard include was added and both normal and sanitizer runs passed.
- One documentation patch expected an older English S3 comment and was
  rejected atomically. It was split into exact-context patches; no partial
  edit occurred.
