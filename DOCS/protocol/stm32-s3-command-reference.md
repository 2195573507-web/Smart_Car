# STM32-S3 SRPv4 Command Reference

Status: **Active**

This document is the source-facing command catalog for the STM32H757 CM7 to
ESP32-S3 UART2 link. The wire format, CRC, flags, retry policy and sync rules
are defined in [`../SRP_v4_Spec.md`](../SRP_v4_Spec.md). The single registry is
[`../../Common/SRP/include/srp_registry.h`](../../Common/SRP/include/srp_registry.h).

## Physical Link

| Item | Contract |
| --- | --- |
| STM32 endpoint | USART2, PA2/PA3 |
| S3 endpoint | UART2, GPIO17/GPIO18 |
| Default serial | 921600 baud, 8 data bits, no parity, 1 stop bit |
| Framing | `AA 55 | LEN_LE | HEADER_LE | PAYLOAD | CRC16-CCITT-FALSE_LE | 0D 0A` |
| Direction | Point-to-point; node IDs are local link configuration, not wire fields |

## Header And Reliability

The 32-bit header is `[priority:8][type:8][sequence:8][flags:8]`. Priorities
are `0=EMERGENCY`, `1=COMMAND`, `2=TELEMETRY`, `3=LOG`. `SRP_FLAG_ACK_REQUIRED`
marks a transaction; `SRP_FLAG_STREAM_DATA` is zero and is used for streams.
Reserved flag bits must be zero. A transaction has four pending slots, a
500 ms ACK timeout and up to three retransmissions. Parser errors feed REC;
transport and timeout failures feed TEC and can enter `SRP_LINK_BUS_OFF`.

## Message Catalog

All payload lengths are exact bytes. Multi-byte fields are little-endian and
must be serialized with `srp_wire_*` helpers.

| Type | Name | Direction | Length | Delivery |
| ---: | --- | --- | ---: | --- |
| `0x01` | `CAL_EVENT` | STM -> S3 | 1 | ACK required |
| `0x02` | `MOTOR_CMD` / `WHEEL_SPEED_CMD` | S3 -> STM | 16 | ACK required |
| `0x03` | `PID_PARAMS_CMD` | S3 -> STM | 16 | ACK required |
| `0x04` | `WHEEL_SPEED_SINGLE_CMD` | S3 -> STM | 5 | ACK required |
| `0x05` | `MASTER_SPEED_CMD` | S3 -> STM | 4 | ACK required |
| `0x06` | `CHASSIS_SPEED_CMD` | S3 -> STM | 16 | ACK required |
| `0x07` | `BOOT_READY` | STM -> S3 | 2 | ACK required |
| `0x08` | `CMD_SYNC_REQ` | S3 -> STM | 4 | stream/handshake |
| `0x09` | `RSP_BOOT_INFO` | STM -> S3 | 8 | stream/handshake |
| `0x10` | `IMU_TELEMETRY` | STM -> S3 | 30 | stream |
| `0x11` | `ATTITUDE` | STM -> S3 | 80 | stream |
| `0x12` | `IMU_CAL_STATUS` | STM -> S3 | 11 | stream |
| `0x13` | `POWER_STATUS` | STM -> S3 | 4 | stream |
| `0x14` | `WHEEL_SPEED_STATUS` | STM -> S3 | 16 | stream |
| `0x15` | `CHASSIS_STATE` | STM -> S3 | 24 | stream |
| `0x16` | `WHEEL_CONTROL_STATUS` | STM -> S3 | 44 | stream |
| `0x17` | `CHASSIS_HEADING_CMD` | S3 -> STM | 12 | ACK required |
| `0x20` | `RADAR_STATUS` | STM -> S3 | 2 | stream |
| `0x21` | `RADAR_PWM_READY` | S3 -> STM | 1 | ACK required |
| `0x30` | `LOG` | STM -> S3 | variable | stream |
| `0x70` | `SYS_CONFIG` | both | TLV | ACK required |
| `0x7E` | `ACK` | both | 4 | fast response |
| `0x7F` | `ERROR` | both | 4 | fast response |

## Command Payloads

`WHEEL_SPEED_CMD` contains four float32 values in the fixed order
`[RR, RF, LR, LF]`, in mm/s. `WHEEL_SPEED_SINGLE_CMD` contains a wheel index
and one float32 target. `MASTER_SPEED_CMD` contains one float32 master scale.
`CHASSIS_SPEED_CMD` contains linear and angular float32 values followed by two
reserved u32 fields. `CHASSIS_HEADING_CMD` is
`target_v_mm_s_f32 | target_yaw_deg_f32 | flags_u32`; flags must be zero.
`PID_PARAMS_CMD` is `kp_f32 | ki_f32 | kd_f32 | max_accel_f32`, with the ranges
defined by the SRP registry. Motion commands are accepted only after sync and
the local CM7 attitude/safety gates are satisfied.

Calibration uses `BOOT_READY(state,result)`, `RADAR_PWM_READY(speed_percent)`
and `CAL_EVENT(event)`; the zero-PWM admission gate remains owned by CM7.
`IMU_CAL_STATUS` is `stage | radar_pwm | sample_count_u32 | sample_total_u32 |
error_code`. `ATTITUDE` is the 80-byte schema-2 DualAHRS payload, and
`IMU_TELEMETRY` is a source-tagged 30-byte sample. Their schemas and flags are
specified in `SRP_v4_Spec.md` and must not be inferred from receive time.

`CHASSIS_STATE` is the 24-byte schema-1 payload
`schema_u8 | flags_u8 | reserved_u16 | timestamp_ms_u32 | x_mm_f32 |
y_mm_f32 | yaw_deg_f32 | total_dist_m_f32`. Defined flags are attitude safety
fused `0x01`, heading locked `0x02`, odometry valid `0x04`, and attitude ready
`0x08`; all other bits and the reserved field must be zero. The timestamp is
the CM7 monotonic arrival time of the MotorBoard MSPD source sample. Consumers
must stop odometry publication when `ODOMETRY_VALID` is clear and must not
reinterpret S3/host receipt time as source freshness.

## Fast Responses And Configuration

The four-byte response payload is:

```text
ack_type_u8 | reserved_u8 | ack_sequence_u8 | status_code_u8
```

`status_code=0` is success. Nonzero values use the `SRP_FAST_RESP_*` registry
constants. `SYS_CONFIG` currently supports TLV tag `0x01`, a u32 little-endian
baud rate of `921600` or `115200`. The receiver acknowledges at the current
rate, waits the guard window, flushes pending transport data and switches.

## Endpoint Ownership

- CM7 owns sensor, calibration, attitude, chassis, MotorBoard, safety and
  STM-originated telemetry/log production.
- S3 owns UART2 transport, App BLE translation, radar calibration sequencing,
  BLE/Wi-Fi gateway behavior and S3-side recovery.
- App BLE uses its own envelope and type space; it never consumes raw SRPv4
  bytes. YDLIDAR `AA 55`, SmartCarLog and experimental S3RD are also separate
  contracts.

## Verification

```sh
cc -std=c11 -Wall -Wextra -Werror -pedantic \
  -ICommon/SRP/include Common/SRP/srp_crc.c Common/SRP/srp_wire.c \
  Common/SRP/srp_codec.c Common/SRP/srp_link.c \
  Common/SRP/tests/test_srp_codec.c -o /tmp/test_srp_codec
/tmp/test_srp_codec
```

Then build `STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf` and the ESP-IDF
image. These checks prove source integration only. A matching flashed-image
pair, bidirectional UART capture, BLE delivery and controlled vehicle tests
are required for runtime and physical acceptance.
