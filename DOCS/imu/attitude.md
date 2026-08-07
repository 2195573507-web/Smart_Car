# Attitude Module

## Function

Compute and expose roll, pitch, yaw, and quaternion state after calibration and
filter readiness.

## Source Location

`STM32H757/Middleware/Attitude/attitude.c/.h`.

## Entry Functions

`attitude_init`, `attitude_zero_init`, `attitude_zero_is_ready`,
`attitude_update`, `attitude_get_state`, `attitude_get_status`.

## Inputs

Filtered calibrated IMU data and readiness state.

## Outputs

`attitude_state_t`, `AHRS_WAIT_CAL/AHRS_READY`, logs, and telemetry source data.

## Public Interfaces

`attitude_init`, `attitude_zero_init`, `attitude_update`, state getter, and
status getter.

## Dependencies

IMU filter, calibration, timer/math, communication service.

## Current Status

Source layer exists and is called only after filter readiness in `imu_manager`.
No live attitude stream acceptance is asserted.

## Known Issues

Zeroing and quaternion conventions must remain stable across App decoding.

## Modification Notes

Do not publish “ready” before calibration/filter gates. Preserve units,
quaternion order, and update cadence.
