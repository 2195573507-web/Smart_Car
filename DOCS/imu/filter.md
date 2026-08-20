# Filter Module

## Function

Apply the fixed standard filter to calibrated IMU data before attitude
calculation.

## Source Location

`STM32H757/Middleware/Filter/imu_filter.c/.h`.

## Pipeline

The module keeps its bounded median window of five samples followed by the
fixed `IMU_FILTER_ALPHA` EMA parameter. The filter has no external speed or
runtime parameter-selection input.

## Interfaces

`imu_filter_init`, `imu_filter_update`, `imu_filter_get_output`, and
`imu_filter_is_ready` are the complete public interface. The existing
FreeRTOS lock wrapper protects the shared history and output snapshot.

## Dependencies

Calibrated LSM303/BMI323 data, the standard attitude module, and the existing
RTOS lock primitives. Readiness remains gated by static calibration.
