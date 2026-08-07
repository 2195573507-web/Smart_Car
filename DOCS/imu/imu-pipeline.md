# IMU Module

## Function

Acquire LSM303 samples, retain coherent manager state, run calibration and
filter layers, then publish attitude/status data to the communication layer.

## Source Location

`STM32H757/Middleware/Sensor/imu_manager.c/.h` and
`STM32H757/Application/RTOS/imu_runtime.c`.

## Entry Functions

`imu_init`, `imu_update`, `imu_get_data`, `imu_runtime_start`, `imu_task`, and
`imu_debug_task`.

## Inputs

LSM303 accel/mag reads, timer ticks, boot manager events, calibration state.

## Outputs

Raw complete snapshots, sensor readiness/statistics, filtered data, attitude,
logs, and source frame service inputs.

## Interfaces

`imu_update`, `imu_get_data`, LSM303 getters, `imu_boot_manager_update`,
`imu_calibration_apply`, `imu_filter_update`, `attitude_update`.

## Dependencies

LSM303 driver, calibration/vibration/filter/attitude middleware, FreeRTOS, BSP
locks/timer, boot/log services.

## Current Status

LSM303 is the active path. The manager publishes complete samples only after
both accel and magnetometer reads report success; BMI323 runtime is skipped.

## Known Issues

Separate accel/mag getter calls are not an atomic hardware snapshot. Full
reset-to-ready and hardware sensor acceptance are not evidenced here.

## Modification Notes

Preserve sequence/lock semantics, readiness gating, update cadence, and the
distinction between source/build and live sensor evidence.
