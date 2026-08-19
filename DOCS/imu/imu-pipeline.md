# IMU Module

## Function

Acquire LSM303 and BMI323 samples under one `DUAL_IMU_BOOT` lifecycle, retain
a coherent dual-sensor snapshot, perform shared-window calibration/vibration
capture, then publish lifecycle and sensor status. The existing LSM303 filter
and attitude path remains unchanged.

## Source Location

`STM32H757/Middleware/Sensor/imu_manager.c/.h` and
`STM32H757/Application/RTOS/imu_runtime.c`.

## Entry Functions

`imu_init`, `imu_update`, `imu_manager_start_dual_initialization`,
`imu_manager_finalize_dual_initialization`, `imu_runtime_start`, `imu_task`,
and `imu_debug_task`.

## Inputs

LSM303 I2C accel/mag reads, BMI323 SPI accel/gyro reads, `imu_time` timestamps,
radar calibration events, and shared calibration/vibration time windows.

## Outputs

Raw complete snapshots, sensor readiness/statistics, filtered data, attitude,
logs, and source frame service inputs.

## Interfaces

`imu_update`, `imu_manager_get_snapshot`, LSM303/BMI323 getters,
`imu_boot_manager_update`, `imu_calibration_*`, `imu_vibration_*`,
`imu_filter_update`, and `attitude_update`.

## Dependencies

LSM303 and BMI323 drivers, calibration/vibration/filter/attitude middleware,
FreeRTOS, BSP locks/timer, boot/log services.

## Time Base and Sampling Statistics

All IMU acquisition and shared-window timestamps use
`STM32H757/BSP/TIMER/imu_time.h`. `imu_time_now_us()` is the monotonic DWT
microsecond source and `imu_time_now_ms()` is derived from that value. Each
LSM303 and BMI323 snapshot carries `timestamp_us`; legacy millisecond fields
are derived compatibility values, not a separate time source. The DWT 32-bit
counter is extended to 64 bits under a short critical section so the two IMU
tasks cannot race a counter wrap.

`bsp_timer_get_ms()` and HAL tick remain outside the IMU timestamp contract.
The BMI323 port also uses `imu_time` for reset-settle waits; HAL tick is only
an internal peripheral-timeout dependency and never stamps samples or windows.

BMI323 capture statistics expose `configured_rate_hz`, `measured_rate_hz`, and
`sample_count`. Supported configured rates are 100, 200, 400, and 800 Hz.
`measured_rate_hz` is calculated from the first and last successful acquisition
timestamps and the statistics reset after an ODR change.

## Dual Lifecycle

`imu_boot_manager` owns the only lifecycle transition authority:

```text
IDLE -> INIT -> SELF_TEST -> STATIC_CALIBRATION -> VIBRATION_CAPTURE
     -> VERIFY -> READY
                    \-> FAILED
```

`INIT` creates the LSM303/I2C4 and BMI323/SPI1 workers before releasing either
one through a common FreeRTOS task-notification gate. Each later phase enters
for both sensors together. The phase advances only after the shared time window
has closed and both source completion flags are true. Completion means each
source reaches its per-window sampling-quality floor, not merely that one valid
sample exists. The `VERIFY` phase re-checks both retained calibration and final
vibration results before sending the final completion event. The final event
ACK is retried a bounded number of times; when both local result barriers pass,
the STM accepts those local results and enters `READY` even if that final
notification ACK is lost. Missing local results still enter `FAILED`.
`dual_imu_manager_t` records each phase start/end timestamp and exposes LSM,
BMI, and overall progress through the STM-S3/App
`DUAL_IMU_STATUS` frame.

Static calibration and each vibration profile require valid sample count at or
above `ceil(configured_rate_hz * window_duration_s * 0.90)`. The static window
tracks LSM accel, BMI accel, and BMI gyro separately. Vibration datasets track
`captured_count`, `invalid_count`, `actual_rate_hz`, and `quality_ok` per
sensor. A window closes on time even when quality fails, allowing the boot FSM
to report a bounded quality failure instead of waiting indefinitely.

Every five seconds the IMU debug task emits `IMU_RES` with FreeRTOS
`heap_free` and high-water marks in stack words for `imu_task`, the debug task,
and `bmi323_task`. This is diagnostic logging only and adds no task or wire
field.

## Current Status

Source/build-established: LSM303 and BMI323 are initialized and observed under
the shared lifecycle. LSM303 still supplies the legacy calibrated/filter/AHRS
path; BMI323 data is limited to the dual lifecycle, calibration, vibration,
diagnostics, and telemetry paths. No BMI323 value is sent into `attitude.c`,
AHRS, or fusion.

## Known Issues

Separate accel/mag getter calls are not an atomic hardware snapshot. Full
reset-to-ready, sensor electrical behavior, task timing at each BMI ODR,
PWM/ACK exchange, STM-S3/UART/BLE delivery, and App rendering are not evidenced
without hardware execution.

## Modification Notes

Preserve sequence/lock semantics, readiness gating, update cadence, and the
distinction between source/build and live sensor evidence.
