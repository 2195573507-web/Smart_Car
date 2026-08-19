# Progress

## 2026-08-18 Dual Lifecycle Upgrade

- Replaced the legacy transition authority with `DUAL_IMU_BOOT` phases
  `IDLE -> INIT -> SELF_TEST -> STATIC_CALIBRATION -> VIBRATION_CAPTURE ->
  VERIFY -> READY`, with `FAILED` as the terminal error state.
- INIT creates paired FreeRTOS workers for LSM303/I2C4 and BMI323/SPI1; the
  boot FSM advances only after both report success and starts BMI acquisition
  only after that barrier.
- Static and vibration completion is based on common time-window completion
  plus per-sensor valid-observation flags, not sample-count thresholds.
- Added `SCBP_MSG_ID_DUAL_IMU_STATUS=0x0208` / App `0x28`, S3 relay support,
  App decode/store/reset behavior, and a Developer Mode lifecycle card.
- CM7, ESP-IDF S3, and SwiftPM builds passed. No device was flashed and no
  sensor, PWM, UART, BLE, or App runtime capture was performed.
- Replaced the former priority-dependent INIT start assumption with two task
  notifications posted while the scheduler is suspended. Neither LSM303 nor
  BMI323 initialization can begin before both worker tasks exist.
- Current CM7 build passed with `SMARTCAR_BMI323_DEBUG_ONLY=OFF` (FLASH 114260
  B / 1 MB, RAM 52120 B / 128 KB); current S3 and SwiftPM builds also passed.
- A running staged macOS App binary differs from the current Swift build. It
  was not stopped or overwritten, so bundle/runtime UI evidence is pending.
- Tightened `VERIFY`: it now independently re-checks retained LSM303 and
  BMI323 calibration plus final shared-window results before sending
  `CAL_EVENT_COMPLETE`; `READY` requires both local completion flags and the
  correlated completion ACK.

## 2026-08-18

- User approved the independent BMI323 task and manager-owned 512-entry software ring buffer design.
- Implemented independent BMI323 sampling without changing the LSM303 100 Hz manager cadence: the BMI task reads one raw accel/gyro sample, timestamps it, and pushes it into the manager-owned 512-entry ring.
- `imu_update_bmi323()` consumes only the newest queued item once per 10 ms manager tick. A no-new-sample tick clears BMI validity flags, preventing the prior numeric values from being reused as a new calibration/vibration observation.
- ODR configuration accepts 100/200/400/800 Hz and defaults to 100 Hz. Fractional scheduling uses 2/3 ticks at 400 Hz and 1/1/1/2 ticks at 800 Hz with `configTICK_RATE_HZ=1000`.
- CM7 rebuild and link passed after the producer non-blocking correction: FLASH 110180 B / 1 MB, RAM 51328 B / 128 KB. The ring symbol is 10256 B; capture statistics are 28 B; the BMI task requests 384 FreeRTOS words (1536 B), while the compiler reports a 56 B static task frame.
- S3 full rebuild and SwiftPM build passed. Static checks confirm `SC_TYPE_IMU_TELEMETRY=0x27` retains the 30-byte source/flags/timestamp/six-float payload through the S3 bridge and App parser.
- The producer uses zero-wait driver/ring locks. `contention_drop_count` distinguishes lock-contention skips from full-ring overflow and SPI read failures.
- No device was flashed. ODR timing, sensor data quality, ring overflow, timestamp deltas, contention drops, and runtime stack high-water marks remain hardware verification items.

## 2026-08-19 Dual-AHRS Gate and Comparison UI

- Added `DUAL_AHRS_STATE_WAIT_CAL`, `READY`, and `TRACKING` aliases while
  retaining the existing state values used by the middleware.
- Added `dual_ahrs_set_bias()`. Passing NULL clears filter/history/timestamps,
  invalidates both poses, and returns the estimator to `WAIT_CAL`; a finite
  calibration result mapping arms the estimator at `READY`.
- `imu_dual_ahrs_feed_bmi()` now checks `imu_boot_manager_is_ready()` before
  calling `dual_ahrs_update()`. Bias injection and reset happen in the BMI
  acquisition task, avoiding a new cross-task lock. The internal Dual-AHRS
  gate also rejects direct calls before Bias hand-off.
- Added `DualAttitudeComparisonView.swift` and replaced the old composite card
  in `DeveloperModeView`. Primary and redundant pose cards have independent
  3D transforms, Euler readouts, source/status labels, and a separate
  divergence panel with 3/6 degree severity thresholds.
- CM7 target passed: FLASH 131924 B / 1 MB, RAM 55688 B / 128 KB.
- SwiftPM `swift build` passed with no warnings in the final build;
  `script/build_and_run.sh --verify` passed and staged binary hash matched the
  SwiftPM binary.
- `git diff --check` passed. No flash, UART/BLE capture, sensor bench run, or
  physical attitude validation was performed.
