# Independent BMI323 Sampling Design

Status: APPROVED FOR IMPLEMENTATION (2026-08-18)

## Scope

Decouple BMI323 acquisition from the existing 100 Hz LSM303/boot-manager task
while preserving the current LSM303 calibration, filter, attitude, protocol,
and App contracts. BMI323 remains telemetry/calibration/vibration input only;
no attitude fusion is added.

## Data Flow

```text
BMI323 SPI -> bmi323_acquisition_task (100/200/400/800 Hz)
           -> imu_manager-owned 512-entry ring buffer
           -> 100 Hz imu_task consumes newest sample
           -> unified snapshot / calibration / vibration / existing telemetry
LSM303 I2C -> existing 100 Hz imu_task -> same manager snapshot and legacy path
```

## Buffer Contract

`bmi323_raw_sample_t` contains a millisecond timestamp, six raw `int16_t`
axes, and a valid flag. The fixed 512-entry buffer is non-blocking. When full,
the oldest entry is discarded, `overflow_count` is incremented, and the newest
sample is inserted. Manager statistics expose sample count, overflow count,
last timestamp, and maximum observed manager latency.

## Scheduling

The default BMI323 ODR is 100 Hz. Supported rates are 100, 200, 400, and
800 Hz. The BMI task uses `vTaskDelayUntil` and the existing 1 ms tick. At
each manager tick only the newest available BMI sample is copied into the
snapshot; the manager does not loop over the high-rate queue.

## Boundaries

No ISR, hardware FIFO, SPI/I2C owner change, `attitude.c` change, fusion
algorithm, or App UI change. Existing `imu_boot_manager` remains the sole
calibration state machine.
