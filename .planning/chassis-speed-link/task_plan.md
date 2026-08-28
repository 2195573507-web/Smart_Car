# Task Plan: S3 Chassis Speed Link Repair

## Goal

Restore the App BLE `0x2D` -> SRP `0x06` -> CM7 differential chassis ->
MotorBoard target path using the confirmed 193.0 mm track width.

## Phases

- [x] Inspect current protocol, safety, MotorBoard, and build boundaries.
- [x] Confirm track width and implementation scope.
- [x] Add kinematics and chassis task with safety admission.
- [x] Dispatch `CHASSIS_SPEED_CMD` from `s3_service.c` with diagnostics.
- [x] Add sources/includes to the CM7 build and add MotorBoard output logs.
- [x] Run host tests, static checks, and a CM7 clean build.
- [x] Report source/build evidence separately from flashed hardware behavior.

## Safety Boundary

Non-zero targets require synchronized SRP, `g_attitude_is_ready`, and the
MotorBoard task to be admitted. Timeout, BUS_OFF, invalid input, or stop
commands continue to force zero targets/PWM.
