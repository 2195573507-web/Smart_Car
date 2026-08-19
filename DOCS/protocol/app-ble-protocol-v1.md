# App BLE Protocol V1

## Function

Record the frame model currently implemented by the macOS App parser/encoder.

## Source Location

`IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/Model/SmartCarProtocol.swift`

## Frame

```text
AA | 01 | TYPE | LEN_LO | LEN_HI | PAYLOAD[LEN] | CRC_LO | CRC_HI | 55
```

CRC16-MODBUS starts at the version byte and covers version, type, length, and
payload. Maximum payload is 128 bytes. The App parser resynchronizes on `AA`,
checks version, length, tail, and CRC.

## Types Visible in App Source

`CONTROL 0x01`, `STATUS 0x02`, `PING 0x05`, `ACK 0x06`, `IMU_STATUS 0x10`,
`ATTITUDE 0x11`, calibration types `0x12/0x13`, `RADAR_STATUS 0x15`,
`RADAR_VIBRATION_STATUS 0x18`, dual calibration result/profile telemetry
`0x25/0x26/0x27`, and `DUAL_IMU_STATUS 0x28`.

Telemetry payloads are: `0x10` sensor id/online plus nine float32 values
(38 bytes); `0x11` ATTITUDE with the following 30-byte payload (all float32
values are IEEE-754 little-endian):

| Offset | Field | Unit |
| ---: | --- | --- |
| 0 | `roll_rad` | radian |
| 4 | `pitch_rad` | radian |
| 8 | `yaw_rad` | radian |
| 12 | `roll_deg` | degree |
| 16 | `pitch_deg` | degree |
| 20 | `yaw_deg` | degree |
| 24 | `timestamp_ms` | u32, little-endian |
| 28 | `source` | u8, source ID |
| 29 | `valid` | u8, 0/1 |

The same App type `0x11` also carries the schema=2 DualAHRS payload when the
payload length is exactly 80 bytes:

| Offset | Field | Type |
| ---: | --- | --- |
| 0 | schema (`0x02`) | u8 |
| 1 | validity/gate flags | u8 |
| 2 | reserved | u16 LE, zero |
| 4 | timestamp_ms | u32 LE |
| 8 | sample_sequence | u32 LE |
| 12 | primary RPY | 3 x float32 LE, radian |
| 24 | primary quaternion wxyz | 4 x float32 LE |
| 40 | redundant RPY | 3 x float32 LE, radian |
| 52 | redundant quaternion wxyz | 4 x float32 LE |
| 68 | delta RPY | 3 x float32 LE, wrapped radian |

Receivers select the decoder by exact length, reject unknown schema/non-finite
values, and retain the 30/26-byte compatibility branches.

`0x12` stage/pwm/sample_count u32 LE/total_sample u32 LE/error (11 bytes);
`0x13` accel_offset_x/y/z IEEE-754 float32 LE values (12 bytes); `0x15`
online/speed_percent (2 bytes); and `0x18` speed_percent plus four
float32 LE RMS values (17 bytes). The legacy two-byte `0x18` PWM/active
payload remains accepted for the radar PWM display.

The S3 gateway maps STM-S3 types `0x20/0x21/0x22/0x24` to App types
`0x10/0x11/0x12/0x18`, and maps static calibration bias `0x13` directly to
App `0x13`, rebuilding the App envelope and CRC. It generates App `0x15` from
its radar-control state. The static bias frame does not define noise RMS.

`0x27` source-tagged telemetry is 30 bytes. Payload byte 1 uses bit 0 for
accel validity, bit 1 for gyro/magnetometer validity, and bit 2 for the
explicit sensor-online state. The App falls back to the two validity bits for
older firmware that leaves bit 2 clear.

`0x28` is the `DUAL_IMU_BOOT` lifecycle status and is relayed by S3 unchanged
except for the App frame envelope. Its payload is exactly 16 bytes:

| Offset | Field | Type |
| ---: | --- | --- |
| 0 | phase (`IDLE` through `FAILED`) | u8 |
| 1 | LSM303 phase progress | u8 percent |
| 2 | BMI323 phase progress | u8 percent |
| 3 | overall lifecycle progress | u8 percent |
| 4 | lifecycle error | u8 |
| 5 | completion/active/ACK flags | u8 |
| 6 | vibration profile index | u8 |
| 7 | radar PWM | u8 percent |
| 8 | phase start time | u32 LE ms |
| 12 | phase end time (zero while active) | u32 LE ms |

## Status

Implemented in the App model and named S3 translation boundary. BLE delivery
and device behavior remain unverified without hardware.

## Modification Notes

Keep explicit little-endian serialization and the 128-byte bound. Any approved
contract change must update the S3 bridge, STM endpoint, App decoder, and
canonical documentation together.
