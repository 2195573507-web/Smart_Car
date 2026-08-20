# IMU Module

## Function

The pipeline acquires LSM303 and BMI323 data with explicit sensor identity and
timestamps, performs the existing static calibration gate, and preserves the
LSM303 calibrated/filter/AHRS path as the primary attitude source.

## Data Flow

```text
LSM303/BMI323 acquisition
    -> imu_manager snapshots and locks
    -> imu_boot_manager static calibration gate
    -> calibration/filter/LSM303 AHRS
    -> SCBP-CAN telemetry and schema-2 DualAHRS attitude
```

BMI323 is sampled independently. Its failure must not block LSM303 processing
or the established 10 ms manager cadence.

## SCBP-CAN Output

- `IMU_CAL_STATUS(0x202)`: 11-byte lifecycle status.
- `IMU_TELEMETRY(0x207)`: one 30-byte frame per sensor. Sensor ID 1 is
  LSM303 and sensor ID 2 is BMI323.
- `ATTITUDE(0x201)`: one 80-byte schema-2 DualAHRS payload only.
- `CAL_EVENT(0x001, STATIC_DONE=1)`: emitted after the admitted static window.

No `0x0208` dual-status, legacy IMU status, bias/result payload, or 30-byte
ATTITUDE frame is published on UART2.

## Evidence Boundary

Source and build checks prove compilation and symbol closure only. They do not
establish sensor operation, UART transfer, BLE transfer, or attitude quality.
