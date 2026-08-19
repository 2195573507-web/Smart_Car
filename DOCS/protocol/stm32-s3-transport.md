# STM32-S3 Transport Frame

## Function

Record the C frame currently used by the STM32 UART link and S3 service.

## Source Location

- STM32: `STM32H757/Middleware/Communication/SmartCar_Frame/sc_frame.c`
- S3: `ESPS3/components/smartcar_protocol/frame.c`

## Current Contract

Status: CONFIRMED source/build contract.

The active STM32-S3 UART frame is [SCBP-V3](SCBP_V3_REFERENCE.md):

```text
AA | 55 | VER | PRIORITY | SRC | DST | MSG_ID_LO | MSG_ID_HI |
SEQ | FLAGS | LEN_LO | LEN_HI | PAYLOAD[LEN] | CRC_LO | CRC_HI
```

It has 14 bytes of overhead and CRC16-MODBUS covers `VER` through the payload.
`BOOT_READY=0x0007` is STM32 to S3 with `state,result`; `RADAR_PWM_READY=0x0302`
is S3 to STM32 with `speed_percent`; and `PWM_SET=0x0101` remains an explicit
active-PWM command. `ATTITUDE=0x0201` accepts the legacy 30-byte layout and the
schema=2 80-byte DualAHRS layout; exact length/schema validation is required.
Its field layout, ACK/ERROR behavior, parser recovery, and full message table
are authoritative in the linked reference.

## Legacy Frame (Pre-SCBP-V3)

```text
AA | 55 | 01 | TYPE | LEN_LO | LEN_HI | PAYLOAD[LEN] | CRC_LO | CRC_HI
```

`SC_FRAME_MAX_PAYLOAD` is 128 and overhead is 8. CRC16-MODBUS covers the
version through payload bytes. The C parsers accept fragmented chunks and
report header/version/length/CRC errors through callbacks.

## Legacy Type Set (Adapter Inputs Only)

`PING 0x01`, `PONG 0x02`, `ACK 0x03`, `PWM_READY 0x10`,
`IMU_CAL_BIAS 0x13`,
`RADAR_PWM_READY 0x16`, `RADAR_PWM_ACK 0x17`, `CAL_EVENT 0x18`,
`CAL_EVENT_ACK 0x19`, `STM_BOOT_READY 0x1C`, `IMU_STATUS 0x20`,
`ATTITUDE 0x21`, `IMU_CAL_STATUS 0x22`, `RADAR_STATUS 0x23` (gateway-owned
reservation), `RADAR_VIBRATION_STATUS 0x24`, and `LOG 0x30`.

`0x13` is 12 bytes: static `accel_offset_x/y/z` IEEE-754 float32 LE values.
STM emits it once after the final static `0x22` status and before the static
`CAL_EVENT`; it deliberately has no static noise-RMS field. `0x22` is 11 bytes:
stage, PWM, sample_count u32 LE, total_sample u32 LE, error. Stages are
`0 WAIT_RADAR_READY`, `1 STATIC_STABLE_WAIT`,
`2 STATIC_SAMPLE`, `3 VIBRATION_STABLE_WAIT`, `4 VIBRATION_SAMPLE`,
`5 COMPLETE`, `6 ERROR`. `0x24` is 17 bytes: speed_percent followed by
`rms_x/y/z/total_rms` IEEE-754 float32 LE values, emitted after each completed
PWM profile. STM `CAL_EVENT 0x18` remains the calibration handshake.

`0x27 IMU_TELEMETRY` is a source-tagged 30-byte payload: sensor id, flags,
timestamp u32 LE, accel xyz, and either LSM303 mag xyz or BMI323 gyro xyz.
Flags bit 0/1 remain the two channel-valid bits; bit 2 is the explicit sensor
online state. The S3 bridge forwards the payload unchanged.

`0x21 ATTITUDE` is the legacy adapter input. The active STM32 producer and
SCBP-V3 wire payload use the same 30-byte layout. STM32 keeps the AHRS state in
radians and the communication boundary derives degree values without changing
the attitude algorithm:

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

The S3 bridge accepts exactly 30 bytes for SCBP-V3 `ATTITUDE=0x0201` and also
accepts exactly 80 bytes when byte 0 is schema `0x02`. Both payloads are
forwarded unchanged into the App BLE telemetry envelope. S3 performs no unit
conversion or field translation.

## Migration Status

The legacy table above is retained only as a callback-adapter migration record.
The active wire contract is SCBP-V3. Both endpoints compile against it; UART,
BLE, and radar runtime behavior remain UNVERIFIED.

## Modification Notes

Keep the C frame independent from App BLE framing unless a named translation
layer is added and tested. Preserve bounded buffers, parser resynchronization,
CRC diagnostics, and local safety ownership.
