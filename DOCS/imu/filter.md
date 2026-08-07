# Filter Module

## Function

Apply bounded median/vibration-profile handling and EMA filtering to calibrated
IMU data before attitude calculation.

## Source Location

`STM32H757/Middleware/Filter/imu_filter.c/.h`.

## Entry Functions

`imu_filter_init`, `filter_set_vibration_profile`, `imu_filter_update`,
`imu_filter_get_output`, `imu_filter_is_ready`.

## Inputs

`imu_calibrated_data_t`, timestamp/online state, and vibration profiles.

## Outputs

`imu_filtered_data_t` and readiness.

## Public Interfaces

`imu_filter_init`, `filter_set_vibration_profile`, `imu_filter_update`,
`imu_filter_get_output`, and `imu_filter_is_ready`.

## Current Parameters

Median window `5`, EMA alpha `0.95f` in the current header. These are source
facts, not performance acceptance.

## Dependencies

Calibration/vibration types, FreeRTOS lock wrapper, attitude.

## Current Status

Source layer exists and is gated by calibration. Runtime stability and latency
are unverified.

## Known Issues

Memory and task cadence must be checked when changing sample history or profiles.

## Modification Notes

Preserve readiness semantics and thread-safety. Do not hide missing samples by
silently reusing stale snapshots.
