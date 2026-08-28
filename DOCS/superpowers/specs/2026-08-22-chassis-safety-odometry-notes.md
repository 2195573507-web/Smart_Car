# Findings: CM7 Chassis Heading, Attitude Safety, and Odometry

## Confirmed Source Facts

- Motor order is `M1=RR`, `M2=RF`, `M3=LR`, `M4=LF`.
- `MSPD` provides four actual wheel speeds. The RF sign is applied once before
  the PID and the resulting values are in the existing speed-control unit
  (`mm/s`).
- BMI323 gyro data is available in `rad/s`; Primary DualAHRS Euler angles are
  in radians and must be converted before degree-based control and telemetry.
- The current wheel command carries four speed targets. A left/right speed
  difference is the available active-turn request; equal left/right speed is
  the available straight-drive request.
- Existing startup ownership is `attitude_startup_coordinator`; it keeps the
  MotorBoard force-stopped until the attitude lifecycle is ready.

## Protocol

`SCBP_MSG_ID_CHASSIS_STATE = 0x211`, stream, STM -> S3, 24 bytes:

| Offset | Field |
| ---: | --- |
| 0 | schema `1` |
| 1 | flags: safety-fused, heading-lock, odometry-valid, attitude-ready |
| 2..3 | reserved, zero |
| 4..7 | timestamp ms, uint32 LE |
| 8..11 | x mm, float32 LE |
| 12..15 | y mm, float32 LE |
| 16..19 | yaw deg, float32 LE |
| 20..23 | total distance m, float32 LE |

S3 translates this unchanged payload into App BLE frame type `0x29` after
schema, reserved-byte, length, and finite-float validation.
