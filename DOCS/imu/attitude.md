# Attitude Module

## Function

The calibrated LSM303 path computes the primary attitude state after its
readiness gate. DualAHRS output retains an explicit primary/redundant layout;
it does not introduce BMI323 as a replacement primary attitude source.
LSM303 samples reach this layer in the vehicle Body Frame; DualAHRS rebuilds
the redundant quaternion whenever final Euler values are reference-adjusted.

## UART Output

UART2 publishes only `ATTITUDE(0x201)` with the 80-byte SCBP-CAN schema-2
payload. Its fields are little-endian:

| Offset | Field |
| ---: | --- |
| 0 | `schema=2` |
| 1 | validity flags |
| 2 | reserved `u16=0` |
| 4 | timestamp milliseconds |
| 8 | sample sequence |
| 12 | primary Euler roll/pitch/yaw, radians |
| 24 | primary quaternion W/X/Y/Z |
| 40 | redundant Euler roll/pitch/yaw, radians |
| 52 | redundant quaternion W/X/Y/Z |
| 68 | delta Euler roll/pitch/yaw, radians |

S3 validates the exact length/schema pair and places the same 80 payload bytes
inside App BLE type `0x11`. It does not convert units, quaternion ordering, or
payload structure. The previous 30-byte attitude format is not emitted or
accepted on the STM32-S3 transport.

## Constraints

Do not report ready before the calibration/filter gates complete. Preserve the
current radian convention, quaternion ordering, source timestamps, and update
cadence. Build evidence does not establish live attitude-stream acceptance.
