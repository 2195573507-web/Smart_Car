# LSM303 Boot Flow Repair Report

## Changed files

- `STM32H757/Drivers/IMU/LSM303/lsm303.c`
- `STM32H757/Middleware/Sensor/imu_manager.c`
- `STM32H757/Middleware/Calibration/imu_calibration.c`
- `STM32H757/Middleware/Calibration/imu_calibration.h`
- `STM32H757/Middleware/Calibration/README.md`
- `STM32H757/Middleware/Filter/imu_filter.c`
- `STM32H757/Middleware/Filter/imu_filter.h`
- `STM32H757/Middleware/Filter/README.md`
- `STM32H757/Application/RTOS/imu_runtime.c`

## Verification

- `cmake --preset CM7` from `STM32H757` is unavailable in the current project;
  the checked-in presets are `Debug` and `Release`.
- Equivalent CM7 verification passed with `cmake -S CM7 --preset Debug` followed
  by `cmake --build --preset Debug -j8` from `STM32H757/CM7`.
- A clean rebuild compiled 51 objects and linked
  `CM7/build/Debug/Smart_Car_H757_CM7.elf` successfully.
- No flash, monitor capture, live WHO_AM_I, sample cadence, or long-duration
  hardware evidence was collected in this run.
