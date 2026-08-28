# Task Plan: Chassis Differential Command

## Goal

Add the SCBP-CAN `0x114 CHASSIS_SPEED_CMD` and App BLE `0x2D` path while
preserving the existing `0x110` / `0x15` independent-wheel contract.

## Scope

- Shared SCBP definitions and canonical command reference.
- STM32 chassis runtime and S3 service dispatch.
- ESP32-S3 App BLE parser/bridge mapping.
- Active macOS App BLE encoder and control/status UI.
- No changes to hardware pins, generated code, PID gains, or the independent
  wheel payload.

## Phases

- [x] Read project rules, protocol reference, active source paths, and dirty state.
- [x] Confirm design and compatibility boundary.
- [x] Implement shared definitions and endpoint dispatch.
- [x] Implement chassis differential state and IMU-invalid open-loop gate.
- [x] Implement App BLE type and active macOS UI behavior.
- [x] Update protocol docs/tests where the live contract is maintained.
- [x] Run static checks, host tests, and available firmware/app builds.
- [x] Report source/build evidence separately from hardware acceptance.

## Verification Targets

- `0x110` remains exactly 16 bytes and enters independent mode.
- `0x114` and App `0x2D` reject any payload length other than 16.
- All f32 fields use explicit little-endian helpers.
- Diff mode calls heading control when `primary_valid` is true.
- Invalid Primary IMU bypasses heading correction with the required warning.
- App mode status decodes `0x2C` offset 1 and drives the two UI colors.

## Completion Status

Source, host protocol, CM7, ESP-IDF, and Swift build evidence is complete.
Physical BLE/UART capture, flashed-image validation, IMU runtime behavior, and
vehicle straight-line acceptance remain pending.
