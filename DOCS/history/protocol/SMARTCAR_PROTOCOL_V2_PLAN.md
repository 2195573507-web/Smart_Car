# SmartCar Protocol V2

This document freezes the telemetry/control identifiers used by STM32H757,
ESP32-S3, and SmartCar Control MAC. Frames use the existing `AA` head, version
`01`, little-endian `uint16` length and CRC, and `55` tail. Floating-point
values are IEEE-754 `float32` little-endian values.

| Type | Name | Direction | Payload | Encoding |
| --- | --- | --- | ---: | --- |
| `0x10` | `IMU_STATUS` | STM32 -> S3 -> App | 43 | sensor/status, 9 float32 values, calibration state/sample/total |
| `0x11` | `ATTITUDE` | STM32 -> S3 -> App | 14 | roll, pitch, yaw float32 LE; valid u8; source u8 |
| `0x12` | `IMU_CAL_STATUS` | STM32 -> S3 -> App | 7 | state u8, sample mode u8, current PWM u8, current sample uint16 LE, total progress u8, error u8 |
| `0x14` | `RADAR_PWM_CONTROL` | App -> S3 | 1 | PWM percent u8, range 0..100 |
| `0x15` | `RADAR_STATUS` | S3 -> STM32/App | 2 | online u8 (0/1), current PWM percent u8 (0..100) |
| `0x18` | `RADAR_CAL_STATUS` | S3 -> App | 2 | current calibration PWM u8 (0..100), active u8 (0/1) |

Calibration PWM application is an internal STM32-S3 transaction. `PWM_APPLIED`
means that S3 successfully applied the requested LEDC duty; it is not radar
feedback:

| Type | Name | Direction | Payload | Encoding |
| --- | --- | --- | ---: | --- |
| `0x16` | `PWM_SET` | STM32 -> S3 | 1 | requested PWM percent u8, 0..100 |
| `0x17` | `PWM_APPLIED` | S3 -> STM32 | 2 | result u8 (0 success/1 failure), applied PWM u8 |

`0x10` owns only IMU sensor telemetry. It does not update calibration state in
the App. `0x12` is the sole source for state, sample mode, progress, current
PWM, sample progress, error code, and timestamped calibration freshness.
`0x18` belongs to the App RadarState and is not a calibration state.

All multi-byte fields are serialized explicitly at the documented offsets;
compiler structure padding is never part of a payload. Values outside the
documented ranges are rejected by the endpoint parser.
