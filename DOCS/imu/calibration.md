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

Complete LSM303 raw samples, S3 radar PWM ready/ACK/event frames, timer windows,
and calibration constants (5000 accel samples at 100 Hz).

## Outputs

Boot states, progress, RMS/bias, calibrated data, transport events, and error
status for App telemetry.

## Public Interfaces

`imu_boot_manager_*`, `imu_calibration_*`, and the vibration-profile interfaces.

## Dependencies

LSM303 manager data, STM-S3 transport callbacks, timer/RTOS context, filter,
attitude, and radar calibration manager events.

## State Chain

`IMU_BOOT_INIT -> WAIT_RADAR_ZERO -> STATIC_CAL_WAIT -> STATIC_CAL_SAMPLE ->
STATIC_CAL_DONE -> WAIT_RADAR_LEVEL -> VIBRATION_SAMPLE ->
VIBRATION_LEVEL_DONE -> VIBRATION_ALL_DONE -> FILTER_READY -> IMU_READY`, with
`IMU_ERROR` as terminal failure.

## Current Status

Source-established state model; full reset-to-ready capture and physical PWM
feedback are unverified.

## Known Issues

Radar PWM readiness is a protocol/LED C readiness signal, not proof of physical
radar rotation. Event timing, retries, and error paths require full capture.

## Modification Notes

Preserve existing transitions, ACK/retry/error semantics, and no-filter-before-
ready gating. Do not create a parallel state machine.
