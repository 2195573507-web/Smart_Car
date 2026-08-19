# SCBP-V3 Reference

Status: CONFIRMED source contract. This document defines the STM32H757 to
ESP32-S3 UART protocol only. It does not replace the separate App BLE envelope.

## 1. Introduction

SCBP-V3 is the SmartCar Bus Protocol Version 3. It replaces the former
one-byte transport TYPE with a 16-bit MSG_ID and adds priority, source,
destination, sequence, and flags to every UART frame. Physical transport stays
115200 baud, 8N1, and no hardware flow control.

## 2. Design Principles

- The UART envelope is fixed after V3. New functions use a new MSG_ID.
- CRC16-MODBUS protects every field after the SOF bytes.
- A byte-stream parser accepts fragmented input and resynchronizes on `AA 55`.
- Sequence diagnostics detect gaps, duplicates, and out-of-order frames
  without suppressing valid application messages.
- The STM32 and S3 preserve their existing ownership; protocol adapters keep
  protected business callbacks source-compatible.

## 3. Complete Frame

```text
AA | 55 | VER | PRIORITY | SRC | DST | MSG_ID_L | MSG_ID_H |
SEQ | FLAGS | LEN_L | LEN_H | PAYLOAD[LEN] | CRC16_L | CRC16_H
```

The fixed overhead is 14 bytes. The maximum payload is 128 bytes and the
maximum frame is 142 bytes.

## 4. Fields

| Offset | Field | Width | Meaning |
| ---: | --- | ---: | --- |
| 0-1 | SOF | 2 | Fixed `AA 55` |
| 2 | VER | 1 | `0x01` |
| 3 | PRIORITY | 1 | Emergency=0, Realtime=1, Normal=2, Debug=3 |
| 4 | SRC | 1 | Sending node ID |
| 5 | DST | 1 | Receiving node ID or broadcast |
| 6-7 | MSG_ID | 2 | Little-endian 16-bit message ID |
| 8 | SEQ | 1 | Per-sender counter, wraps at 255 |
| 9 | FLAGS | 1 | ACK, retry, stream, configuration flags |
| 10-11 | LEN | 2 | Little-endian payload length |
| 12.. | PAYLOAD | LEN | Message-specific bytes |
| 12+LEN | CRC16 | 2 | Little-endian CRC16-MODBUS |

Defined flag bits are `ACK_REQUIRED=0x01`, `ACK_FRAME=0x02`,
`ERROR_FRAME=0x04`, `RETRY=0x08`, `STREAM_DATA=0x10`, and `CONFIG=0x20`.
Bits 6-7 are reserved and rejected by the parser.

## 5. Nodes

| Node | ID |
| --- | ---: |
| STM32H757 | `0x01` |
| ESP32-S3 | `0x02` |
| App | `0x03` |
| ROS2 | `0x04` |
| C5 node | `0x10` |
| Broadcast | `0xFF` |

Current STM32-originated messages use `SRC=0x01`, `DST=0x02`. Current
S3-originated messages use `SRC=0x02`, `DST=0x01`.

## 6. MSG_ID Table

| Group | MSG_ID | Name |
| --- | ---: | --- |
| System | `0x0001` | PING |
| System | `0x0002` | PONG |
| System | `0x0003` | VERSION |
| System | `0x0004` | RESET |
| System | `0x0005` | ACK |
| System | `0x0006` | ERROR |
| System | `0x0007` | BOOT_READY |
| Control | `0x0100` | MOTOR_CONTROL |
| Control | `0x0101` | PWM_SET |
| Control | `0x0102` | PARAM_SET |
| Sensor | `0x0200` | IMU_STATUS |
| Sensor | `0x0201` | ATTITUDE |
| Sensor | `0x0202` | IMU_CAL_STATUS |
| Sensor | `0x0203` | IMU_BIAS |
| Sensor | `0x0204` | VIBRATION_STATUS |
| Actuator | `0x0300` | RADAR_CONTROL |
| Actuator | `0x0301` | RADAR_STATUS |
| Actuator | `0x0302` | RADAR_PWM_READY |
| Calibration | `0x0400` | CAL_START |
| Calibration | `0x0401` | CAL_EVENT |
| Debug | `0xF000` | LOG |

### Dual-IMU telemetry payloads

`IMU_TELEMETRY (0x0207)` is a source-tagged 30-byte stream payload:

```text
sensor_id | flags | timestamp_u32_le | accel_xyz_float32_le |
           sensor-specific vector (LSM303 mag_xyz or BMI323 gyro_xyz)
```

`flags` preserves bit 0 for accel validity and bit 1 for the second vector
(magnetometer or gyro) validity. Bit 2 is the explicit sensor `ONLINE` flag.
Receivers must retain the legacy validity-derived fallback when bit 2 is not
present.

`BOOT_READY (0x0007)` is STM32 to S3 with payload `state, result`.
`RADAR_PWM_READY (0x0302)` is S3 to STM32 with payload `speed_percent`.
`PWM_SET (0x0101)` remains reserved for an active PWM-set command and is not
used as a calibration-ready substitute.

## 7. ACK

All acknowledgements use `MSG_ID=0x0005`, `FLAGS.ACK_FRAME=1`, and this
five-byte payload:

```text
ACK_MSG_ID_L | ACK_MSG_ID_H | ACK_SEQ | RESULT | ERROR_CODE
```

`RESULT=0` means success and `RESULT=1` means failure. `ERROR_CODE=0` means
OK. The legacy calibration callbacks retain their original two-byte local
arguments, but their UART representation is a V3 ACK:

- `RADAR_PWM_ACK` acknowledges `RADAR_PWM_READY (0x0302)`.
- `CAL_EVENT_ACK` acknowledges `CAL_EVENT (0x0401)`.

The adapter correlates the ACK sequence before invoking the existing callback.

## 8. ERROR

An ERROR is `MSG_ID=0x0006`, `FLAGS.ERROR_FRAME=1`, and has this payload:

```text
ERROR_SOURCE | ERROR_CODE | ERROR_MSG_ID_L | ERROR_MSG_ID_H | ERROR_SEQ
```

Defined error codes are `UNKNOWN_MSG=0x01`, `INVALID_LENGTH=0x02`,
`CRC_ERROR=0x03`, `BUSY=0x04`, `TIMEOUT=0x05`, `NOT_READY=0x06`,
`SENSOR_ERROR=0x07`, and `PARAM_ERROR=0x08`.

The service sends ERROR only for a syntactically valid unicast frame with an
unsupported MSG_ID or invalid application payload length. CRC/header failures
are reported through parser diagnostics because a reply cannot safely trust
the malformed source fields.

## 9. CRC

SCBP-V3 uses CRC16-MODBUS with polynomial `0xA001` and initial value `0xFFFF`.
The covered bytes are `VER` through the final payload byte. The two SOF bytes
and the CRC bytes are excluded. CRC is placed little-endian.

## 10. Parser State Machine

The parser is non-blocking and consumes arbitrary UART chunks. It seeks `AA`,
then `55`, reads the fixed header through `LEN_H`, bounds-checks the payload
length, and collects exactly the declared frame size. It validates version,
priority, reserved flags, length, and CRC before dispatch. Invalid length or
CRC preserves the latest possible `AA` and resumes seeking a new `AA 55`.

For each source ID it records the latest accepted SEQ as `FIRST`, `IN_ORDER`,
`GAP`, `DUPLICATE`, or `OUT_OF_ORDER`. Diagnostics do not discard a valid
frame, preserving retry and duplicate-ACK behavior.

## 11. Hex Example

This PING has `SRC=STM32`, `DST=S3`, `SEQ=0x2A`, Normal priority, no payload,
and CRC `0x67D3` stored little-endian:

```text
AA 55 01 02 01 02 01 00 2A 00 00 00 D3 67
```

## 12. ATTITUDE

`MSG_ID=0x0201` uses Realtime priority and the Stream flag. Legacy producers
send the 30-byte payload below. DualAHRS producers send an explicit schema=2
80-byte payload; receivers select by exact length and preserve the legacy
30-byte branch:

| Offset | Field | Width |
| ---: | --- | ---: |
| 0 | roll_rad | float32 LE |
| 4 | pitch_rad | float32 LE |
| 8 | yaw_rad | float32 LE |
| 12 | roll_deg | float32 LE |
| 16 | pitch_deg | float32 LE |
| 20 | yaw_deg | float32 LE |
| 24 | timestamp_ms | uint32 LE |
| 28 | source | uint8 |
| 29 | status | uint8 |

The active STM32 attitude algorithm remains radian-based internally. The
protocol carries both radian and degree values. `timestamp_ms` is the monotonic
millisecond timestamp captured when the attitude frame is produced.

### Schema 2 dual attitude (80 bytes)

| Offset | Field | Width |
| ---: | --- | ---: |
| 0 | schema (`0x02`) | u8 |
| 1 | validity/gate flags | u8 |
| 2 | reserved | u16 LE, zero |
| 4 | timestamp_ms | u32 LE |
| 8 | sample_sequence | u32 LE |
| 12 | primary roll/pitch/yaw | 3 x float32 LE, radian |
| 24 | primary quaternion w/x/y/z | 4 x float32 LE |
| 40 | redundant roll/pitch/yaw | 3 x float32 LE, radian |
| 52 | redundant quaternion w/x/y/z | 4 x float32 LE |
| 68 | delta roll/pitch/yaw | 3 x float32 LE, wrapped radian |

The schema=2 payload is rejected for any other length or schema. S3 forwards
the 80 payload bytes unchanged inside App BLE type `0x11`; it does not convert
units or quaternion order.

## 13. Adding Messages

Future GPS, BMI323, ROS2, and C5 functions must receive a new 16-bit MSG_ID.
They must not repurpose a defined ID or change this frame layout. Document the
direction, payload layout, priority, ACK requirement, and error behavior with
the new ID.

## 14. STM32 Implementation

`STM32H757/Middleware/Communication/SmartCar_Frame/` owns V3 encoding,
decoding, CRC, sequence diagnostics, and legacy-event adaptation.
`STM32H757/Middleware/Communication/Services/s3_service.c` owns the STM UART
service mapping. It emits `BOOT_READY=0x0007`, receives
`RADAR_PWM_READY=0x0302`, and associates V3 ACK frames with existing local
calibration callbacks. It accepts the active 30-byte attitude payload, the
schema=2 80-byte DualAHRS payload, and retains a 26-byte adapter conversion
for older producers.

## 15. ESP32-S3 Implementation

`ESPS3/components/smartcar_protocol/` owns the matching V3 codec and parser.
`ESPS3/components/smartcar_service/command_bridge.c` maps V3 IDs to existing
calibration callbacks and re-envelopes telemetry for the separate App BLE
protocol. For ATTITUDE it performs no field, length, or unit conversion: the
validated 30-byte legacy or 80-byte schema=2 V3 payload is placed unchanged
inside the BLE envelope.

Hardware and end-to-end acceptance remain UNVERIFIED until an authorized
flash, UART capture, BLE capture, and device test are performed.
