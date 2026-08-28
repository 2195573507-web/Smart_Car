# CM7 Chassis Heading, Attitude Safety, and Odometry Design

Status: CONFIRMED for implementation on 2026-08-22.

## Boundaries

The STM32H757 CM7 remains the final motion and safety authority. The ESP32-S3
only validates and relays telemetry; the macOS App only displays it. The tuned
wheel PID gains, trim, feedforward, friction compensation, and startup safety
state machine are protected.

## Control and Safety

`chassis_heading_control_step()` is a deterministic, allocation-free function.
When a nonzero forward command is straight, it latches the current Primary
Yaw and applies the approved degree-domain PD correction. A nonzero left/right
target difference denotes active turning and refreshes the latch.

The attitude-safety module owns `g_attitude_safety_fused`. Any finite Primary
Roll or Pitch above 45 degrees latches the fuse and invokes
`motor_board_force_stop()`. It permits re-arm only when both angles are within
15 degrees and all requested wheel speeds are zero. While fused, CM7 rejects
nonzero wheel targets.

## Odometry

At 50 ms, CM7 averages the four calibrated actual `MSPD` wheel speeds to form
the forward displacement. It projects that increment using Primary Yaw:
`x += ds * cos(yaw)` and `y += ds * sin(yaw)`. Total distance accumulates
`abs(ds)` in metres. This is wheel-speed plus IMU-Yaw odometry; no unverified
`MTEP` pulse-to-distance scale is used.

## Chassis State Telemetry

CM7 publishes the 24-byte schema-1 `CHASSIS_STATE` payload under SCBP-CAN
message `0x211`. S3 validates it and sends the same payload to the App as BLE
type `0x29`. Existing `0x201` and `0x210` payload contracts are unchanged.

## Verification

Static checks cover source inclusion, payload length, schema, reserved bytes,
and finite fields. CM7, S3, and SwiftPM builds are separate build evidence;
hardware tilt-stop response, UART/BLE transfer, and odometry scale require
device captures after flashing.
