# Calibration Module

## Function

`imu_boot_manager` owns the static calibration gate for the active LSM303
attitude path. BMI323 remains independently sampled and reported as telemetry;
it does not replace the LSM303 attitude/calibration authority.

## Lifecycle

```text
IDLE -> INIT -> SELF_TEST -> STATIC_CALIBRATION -> READY
                                           \\-> READY (IMU_DEGRADED)
                                           \\-> FAILED
```

The STM32 sends transactional `BOOT_READY(0x007)` when it waits for the S3
zero-PWM handshake. The S3 applies zero calibration PWM and sends
transactional `RADAR_PWM_READY(0x21, speed_percent=0)`. The STM32 admits the
message only at the expected state, keeps PWM at zero through the static
window, and returns a fast ACK or ERROR.

After static calibration closes, STM emits transactional
`CAL_EVENT(0x001, STATIC_DONE=1)`. S3 releases its calibration lock only after
admitting that event. Retry handling is idempotent at both admission points.

The startup manager waits 2000 ms before admitting the BMI323 static window and
screens 20 raw gyro samples (300 LSB per-axis limit, 5 bad samples to reject).
Up to three unreasonable windows are retried at 2000 ms intervals. A missing
BMI323 sample stream is treated as a disconnect and promotes the LSM303 legacy
AHRS path, publishes `degraded=1`, and still completes the existing static-done
transaction. This does not change the 11-byte status payload or any BLE/SRP
schema.

## Published Status

The active calibration status is only `IMU_CAL_STATUS(0x12)`, an 11-byte
payload:

```text
stage_u8 | radar_pwm_u8 | sample_count_u32_le |
sample_total_u32_le | error_code_u8
```

`0x0208` dual-lifecycle status, calibration bias/result transport frames, and
V3 adapter events are not active on the STM32-S3 link. The static window and
the quality/readiness checks remain local calibration behavior rather than a
new telemetry schema.

## Evidence Boundary

Build evidence establishes source integration only. Sensor values, radar PWM,
UART acknowledgement, BLE delivery, and vehicle attitude behavior require
separate device validation.
