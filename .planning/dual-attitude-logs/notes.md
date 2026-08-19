# Findings

## App

- `VehicleState.swift` decodes schema=2 DualAttitude payloads only at exactly
  80 bytes and validates reserved bytes and finite quaternion/vector values.
- `TelemetryStore.DualAttitudeState` already coalesces incoming data on a
  50 ms MainActor timer, so the log can be sampled from that pending snapshot
  without touching BLE parsing or the protocol contract.
- `DeveloperModeView` shows the DualAttitude comparison but does not currently
  mount the private decoded-message console in its overview.

## STM32

- `STM32H757/CM7/CMakeLists.txt` compiles
  `Middleware/Sensor/BMI323/bmi323.c` and `bmi323_port.c`.
- Active BMI323 configuration uses ACC `0x4018` (4 g) and GYR `0x4028`
  (500 dps). The conversion functions divide the signed 16-bit full-scale by
  32768; gyro conversion multiplies by pi/180 to produce rad/s.
- `imu_manager.c` converts BMI raw samples before calibration, vibration, and
  `imu_dual_ahrs_feed_bmi`; `dual_ahrs.c` subtracts the stored physical-unit
  biases before filtering and quaternion integration.
- `imu_debug_task` is the lower-rate telemetry/log task. It can own a 1000 ms
  DualAHRS log gate without adding work to the 10 ms sample task.

## Verification

- `swift build` completed successfully with no compiler warnings after the
  MainActor timer callbacks were made explicit.
- `cmake --preset Debug` completed successfully; the CM7 target then compiled
  `imu_runtime.c` and linked successfully. A final build reported no work to do.
- `git diff --check` passed for the touched source files.
- Hardware execution, BLE/FFE3 delivery, UART capture, and live BMI323 response
  remain unverified.
