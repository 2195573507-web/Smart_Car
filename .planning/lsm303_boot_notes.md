# LSM303 Boot Notes

## Baseline findings

- The fitted-path driver uses I2C4 with 7-bit addresses `0x19/0x18` for the
  accelerometer and `0x1E/0x1D` for the magnetometer, and validates the
  LSM303DLHC identity values before setting ready.
- The manager creates the LSM data mutex before initialization and publishes a
  complete accelerometer plus magnetometer snapshot with `timer_get_ms()`.
- Calibration already waits 2000 ms without blocking and collects 1000 samples,
  but its state collapses waiting and collection into one value, only logs the
  final sample marker, and still produces calibrated output during collection.
- The manager currently sends that in-progress calibrated output to the filter,
  so the filter is active before calibration success.
- Runtime data logging has no one-second status line and can print zero/stale
  data when the sensor is offline.

## Implemented

- LSM303 startup now emits the requested POWER_ON, DEVICE_CHECK, ACC_CONFIG,
  MAG_CONFIG, and READY markers after each successful stage.
- The initial manager read was removed; the first sample is taken only after
  `imu_task` starts, and debug output waits for a complete sample count.
- Device readiness is kept separate from data validity: the manager leaves the
  published `online` flag clear until the first complete read succeeds.
- Calibration now exposes `WAIT_STABLE` and `COLLECTING`, logs 100/500/1000
  samples, and publishes no calibrated sample until a stable `DONE` result.
- Calibration and filter state/snapshot access is mutex-protected on the CM7
  FreeRTOS build. The filter is seeded by the first post-calibration sample.
- Runtime emits `[IMU_STATUS]` once per second with LSM303 state, raw count,
  timestamp, calibration state, accelerometer bias, and filter state.
