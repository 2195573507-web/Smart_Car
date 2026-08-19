# Dual Attitude Log Delivery

## Implementation

- `TelemetryStore.DualAttitudeState` samples pending schema=2 frames at most
  once per 250 ms and retains 200 formatted degree log lines.
- `DeveloperModeView` displays the newest lines first in the developer
  overview.
- `imu_runtime.c` emits the requested degree-formatted `[DUAL_AHRS]` line only
  for READY/TRACKING output and only once per 1000 ms debug-task interval.
- `SmartCarViewModel` and all telemetry-store timer callbacks explicitly hop
  to MainActor; this removes the Swift concurrency warnings without changing
  their existing periods.

## BMI323 audit

The active CMake source is `Middleware/Sensor/BMI323`. Its ACC `0x4018` and
GYR `0x4028` configuration constants correspond to the source conversion
constants of +/-4 g and +/-500 dps. Acceleration uses `raw * (range_g * g /
32768)`. Gyroscope uses `raw * (range_dps / 32768) * pi / 180`, producing
rad/s before `imu_manager.c` forwards samples to calibration, vibration, and
DualAHRS. Calibration biases are accumulated and subtracted in m/s^2 and
rad/s respectively; `dual_ahrs.c` subtracts the same-unit bias before filter
and quaternion integration.

## Static evidence

- `swift build`: PASS, no compiler warnings.
- `cmake --preset Debug`: PASS.
- `cmake --build build/Debug --target Smart_Car_H757_CM7 -j2`: PASS; final
  incremental check reported no work to do.
- No protocol bytes, BLE framing, UART routing, sensor configuration, or
  hardware behavior was changed or accepted by these builds.
