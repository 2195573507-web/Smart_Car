# LSM303 Accel/Mag Data Split Design

## Scope

Split the LSM303 sampling output in `imu_manager` into independent raw
accelerometer and magnetometer data sources. This change does not modify the
LSM303 driver, sampling period, FreeRTOS task structure, calibration logic,
filter logic, or attitude logic.

## Data ownership and interfaces

`imu_manager` owns two private caches:

```c
typedef struct { float ax; float ay; float az; } lsm_accel_data_t;
typedef struct { float mx; float my; float mz; } lsm_mag_data_t;
```

The existing LSM lock protects both caches because the values are published as
one LSM303 sample. The public APIs `imu_manager_get_lsm_accel()` and
`imu_manager_get_lsm_mag()` copy their respective cache while holding that
lock. The mixed `lsm303_data_t` public getter is removed.

For the unchanged legacy calibration/filter boundary, the manager takes the
two coherent cache snapshots and assembles a temporary `imu_raw_data_t`. No
algorithm or calibration behavior changes.

## Sampling and logging

The existing `imu_task` continues to call the same LSM303 accel and mag driver
reads at the same 10 ms cadence. Successful reads update their independent
caches and the existing LSM statistics. The existing debug task continues to
run at the same 100 ms cadence and emits one bounded raw block containing:

```text
[DATA][LSM_ACC] ax=... ay=... az=...
[DATA][LSM_MAG] mx=... my=... mz=...
```

Existing status, calibration, filter, and attitude logs remain unchanged.

## Verification

Run a clean CM7 CMake/Ninja build and scan the changed source for the removed
mixed getter, the two new getter declarations/definitions, the two raw log
labels, and unchanged task periods. Do not flash or connect hardware; build
success is source integration evidence only.
