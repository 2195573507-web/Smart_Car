# Calibration Module

## Function

Gate IMU readiness through static calibration, radar/PWM vibration steps,
ACK/event handling, bias application, and terminal error states.

## Source Location

`STM32H757/Middleware/Calibration/imu_calibration.c`, `imu_boot_manager.c`,
`imu_vibration.c` and matching headers.

## Entry Functions

`imu_boot_manager_init/reset/step/update`,
`imu_boot_manager_on_radar_pwm_ready`,
`imu_boot_manager_on_cal_event_ack`, `imu_calibration_start/update/apply`.

## Inputs

LSM303 and BMI323 raw samples, S3 radar PWM ready/ACK/event frames, timer
windows, and calibration constants (a fixed 10,000 ms static window).

## Outputs

Boot states, progress, RMS/bias, calibrated data, transport events, and error
status for App telemetry.

## Public Interfaces

`imu_boot_manager_*`, `imu_calibration_*`, and the vibration-profile interfaces.

## Dependencies

LSM303 manager data, STM-S3 transport callbacks, timer/RTOS context, filter,
attitude, and radar calibration manager events.

## Dual IMU Lifecycle

`imu_boot_manager` owns `DUAL_IMU_BOOT` as the only transition authority:

```text
IDLE -> INIT -> SELF_TEST -> STATIC_CALIBRATION -> VIBRATION_CAPTURE
     -> VERIFY -> READY
                    \-> FAILED
```

The legacy `imu_boot_state_t` values remain a derived compatibility view for
the existing calibration status, logging, and filter gate. They do not decide
lifecycle transitions.

`INIT` releases two temporary FreeRTOS workers together for LSM303/I2C4 and
BMI323/SPI1. `SELF_TEST` uses a common fixed window. `STATIC_CALIBRATION`
keeps PWM at zero, waits the existing settling delay, then opens one 10-second
window for both sources. `VIBRATION_CAPTURE` uses one shared 30-second window
per PWM profile. Every forward transition requires both source result flags and
their configured sampling-quality floor after the time window has closed. Each
phase records start/end timestamps in `dual_imu_manager_t.phase_timing`.

## Current Status

Source/build-established lifecycle model. Full reset-to-ready capture,
physical PWM feedback, S3 ACK timing, sensor quality, BLE notification, and
App rendering remain unverified without hardware execution.

## Static Dual-IMU Window

After the existing zero-PWM stabilization wait, `STATIC_CAL_SAMPLE` opens one
fixed `IMU_CALIBRATION_WINDOW_MS` window (10,000 ms by default). The 100 Hz
manager accumulates LSM303 accelerometer samples. The independent BMI323 task
accumulates each successful accel and gyro read at its configured ODR (100,
200, 400, or 800 Hz). Each stream retains only axis `sum`, `sum_square`, and
`sample_count`; no calibration sample array is stored. Square accumulation
rejects non-finite values and additions without a finite `double`/`DBL_MAX`
margin.

The window is timed in `timestamp_us` using `imu_time_now_us()`. At the time
boundary the calibration module freezes all three actual counts, then computes
LSM accel, BMI accel, and BMI gyro biases only when every stream meets:

`actual_sample_count >= ceil(configured_rate_hz * window_duration_s * 0.90)`.

LSM uses its configured 100 Hz rate; BMI uses the active 100, 200, 400, or
800 Hz ODR. `imu_calibration_get_quality()` exposes configured rate, expected
count, minimum count, actual count, actual rate, and `quality_ok` for LSM
accel, BMI accel, and BMI gyro. The boundary log is bounded to the existing
text transport: `IMU_CAL cfg=<lsm>/<bmi> act=<lsm>/<bmi_accel>/<bmi_gyro>
min=<lsm>/<bmi> q=<lsm>/<bmi_accel>/<bmi_gyro>`. The existing nominal LSM
total remains a compatibility progress value; it is not used as an independent
completion shortcut.

## Vibration Sample Quality

Each 30-second PWM profile records `captured_count` for attempts inside the
shared window, `invalid_count` for failed/non-finite/rejected samples,
`actual_rate_hz` from valid sample count over the complete window, and
`quality_ok`. The same 90% floor applies independently to LSM and BMI. The
window always closes at its time boundary; only a dataset with `quality_ok=1`
reports complete. The boot manager emits one bounded `IMU_VIB` line with the
configured rates, captured counts, invalid counts, actual rates, and quality
bits before accepting or rejecting that PWM profile.

## Known Issues

Radar PWM readiness is a protocol/LED C readiness signal, not proof of physical
radar rotation. Event timing, retries, and error paths require full capture.

## Modification Notes

Preserve the existing PWM/ACK event contract and no-filter-before-ready gate.
Do not add BMI323 data to the LSM303 filter, AHRS, fusion, or attitude path.
