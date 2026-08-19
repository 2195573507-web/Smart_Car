# Findings

## Dual Lifecycle Audit (2026-08-18)

- `imu_boot_manager` currently owns a legacy `WAIT_SYNC` / static / vibration
  flow. Its static calibration is time-windowed, but public state, progress,
  and vibration completion still expose the legacy sample-oriented model.
- `imu_manager` currently invokes `lsm303_init()` before `bmi323_init()`.
  This does not meet the requested synchronized INIT requirement.
- BMI323 already has a separate acquisition task and feeds static/vibration
  collectors without entering `imu_filter` or `attitude_update`.
- The STM-S3-to-App bridge can transport a new source status frame without
  changing existing attitude or calibration payloads.
- Target design: `IDLE -> INIT -> SELF_TEST -> STATIC_CALIBRATION ->
  VIBRATION_CAPTURE -> VERIFY -> READY/FAILED`, with phase timestamps,
  individual progress, and common time-window completion.

- The active BMI323 implementation is `STM32H757/Middleware/Sensor/BMI323`.
- The existing 10 ms manager already calls LSM303 and BMI323 in normal mode,
  but LSM publishes calibration/filter data before the BMI sample is added.
- `imu_raw_data_t` is owned by `imu_calibration.h`; its legacy LSM fields must
  remain populated for the existing filter and attitude path.
- The current working tree contains unrelated user edits. Targeted patches
  must preserve them and no reset/checkout operation is allowed.
- The S3 relay converts SCBP telemetry to App BLE types in
  `smartcar_service/command_bridge.c`.

## 2026-08-19 Dual-AHRS Gate/UI Findings

- `imu_manager.c::imu_dual_ahrs_feed_bmi()` currently calls
  `dual_ahrs_update()` from the BMI acquisition task before checking the boot
  phase. This advances filters, timestamps, and gyro integration during INIT,
  STATIC_CALIBRATION, and VIBRATION_CAPTURE.
- `dual_ahrs.c` has no calibration injection API. Its current state enum uses
  RESET/WARMUP/RUNNING/DEGRADED/FAULT and starts the quaternion at identity.
- `imu_calibration_result_t` already contains `lsm_accel_bias`,
  `bmi_accel_bias`, and `bmi_gyro_bias`; `imu_boot_manager_is_ready()` is the
  authoritative full-lifecycle barrier.
- The existing App parser accepts the unchanged 80-byte payload and
  `TelemetryStore.dualAttitude` already separates `primary` and `redundant`.
  `DeveloperModeView` currently renders both inside one private card with
  incomplete quaternion/readout details.
- The requested UI will use a new internal comparison view with two separate
  pose cards and a separate divergence panel. `ViewThatFits` will keep the
  approved side-by-side layout on wide windows and stack it on narrow windows.
