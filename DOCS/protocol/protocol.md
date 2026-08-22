# SCBP-CAN Protocol Module

## Scope

This document is the active source contract for the STM32H757 CM7 <->
ESP32-S3 UART2 link. The shared implementation is under
`Common/SCBP_CAN/`; both firmware targets compile the same codec, stream
parser, and link-health manager.

The App BLE envelope is intentionally independent. ESP32-S3 validates
SCBP-CAN payloads and creates a new App BLE frame for selected telemetry; it
does not forward a raw UART frame to BLE.

For command additions, use the detailed
[STM32-S3 SCBP-CAN Command Reference](stm32-s3-command-reference.md). It is
the registry for message direction, exact payload offsets, endpoint dispatch,
transaction behavior, extension rules, and verification evidence.

## UART Frame

```text
5A | A5 | CAN_ID_LO | CAN_ID_HI | FLAGS | LEN | HCS | SEQ |
PAYLOAD[LEN] | FCS_LO | FCS_HI | 0D | 0A
```

The header and trailer are fixed at 8 and 4 bytes. `HCS` is CRC-8-ITU over
`CAN_ID_LO..LEN`; `FCS` is CRC16-MODBUS over the payload only. Both CRC fields
are little-endian where their width exceeds one byte.

`scbp_parser_t` owns a 4-byte-aligned frame buffer and payload begins at
offset 8, so the payload base is naturally 4-byte aligned. Payload structures
are packed to preserve the wire contract. Consumers must use byte-wise or
`memcpy` access for packed fields whose member offset is not naturally aligned
(for example `IMU_TELEMETRY.timestamp` at offset 2).

## CAN ID And Flags

```text
CAN_ID[15:14] priority:    0 emergency, 1 realtime, 2 normal, 3 debug
CAN_ID[13:12] source:      1 STM32H757, 2 ESP32-S3, 3 reserved/App identity
CAN_ID[11:10] destination: 1 STM32H757, 2 ESP32-S3, 3 broadcast
CAN_ID[9:0]   message:     0x001..0x3FF
```

The physical UART link uses destination value 3 only as broadcast. App node
addressing is not used on this link because App traffic uses BLE instead.

| Flag | Bit | Meaning |
| --- | ---: | --- |
| `ACK_REQUIRED` | 0 | Sender creates a retried transaction. |
| `IS_ACK` | 1 | `0x005` fast success response. |
| `IS_ERROR` | 2 | `0x006` fast failure response. |
| `STREAM_DATA` | 3 | Non-transactional telemetry or status stream. |

Bits 4 through 7 are reserved and rejected by the parser.

## Active Message Map

| Direction | Message | CAN message ID | Payload |
| --- | --- | ---: | --- |
| STM -> S3 | `CAL_EVENT` | `0x001` | 1 B event ID |
| Both | `ACK` | `0x005` | 4 B fast response |
| Both | `ERROR` | `0x006` | 4 B fast response |
| STM -> S3 | `BOOT_READY` | `0x007` | 2 B state/result |
| S3 -> STM | `WHEEL_SPEED_CMD` | `0x110` | 16 B, transaction |
| S3 -> STM | `PID_PARAMS_CMD` | `0x111` | 16 B, four f32 LE, transaction |
| STM -> S3 | `ATTITUDE` | `0x201` | 80 B schema-2 DualAHRS |
| STM -> S3 | `IMU_CAL_STATUS` | `0x202` | 11 B |
| STM -> S3 | `IMU_TELEMETRY` | `0x207` | 30 B, one frame per sensor |
| STM -> S3 | `POWER_STATUS` | `0x209` | 4 B float32 voltage |
| STM -> S3 | `WHEEL_SPEED_STATUS` | `0x210` | 16 B, stream |
| S3 -> STM, S3 -> App | `RADAR_STATUS` | `0x301` | 2 B |
| S3 -> STM | `RADAR_PWM_READY` | `0x302` | 1 B, transaction |
| STM -> S3 | `LOG` | `0x3F0` | existing bounded log payload |

There are no active `SC_TYPE_*` adapters, PING/PONG, `0x0200`, `0x0208`,
30-byte legacy ATTITUDE, IMU bias/result transport frames, or inactive control
IDs on this UART contract. The historical V3 numeric value `0x0401` is not a
SCBP-CAN message ID; `CAL_EVENT` is `0x001` because SCBP-CAN has a 10-bit
message field.

## Payload Contracts

| Payload | Exact layout |
| --- | --- |
| Fast response | `ack_can_id_u16_le, ack_seq_u8, status_code_u8` |
| `BOOT_READY` | `state_u8, result_u8` |
| `CAL_EVENT` | `event_id_u8`: 1 static done, 2 vibration step done, 3 complete |
| `RADAR_STATUS` | `online_u8, speed_percent_u8` |
| `RADAR_PWM_READY` | `speed_percent_u8` |
| `WHEEL_SPEED_CMD` | `wheel_speed[4]_f32_le` in M1=RR, M2=RF, M3=LR, M4=LF order |
| `PID_PARAMS_CMD` | `kp_f32_le, ki_f32_le, kd_f32_le, max_accel_f32_le` (16 B) |
| `POWER_STATUS` | `battery_voltage_f32_le` |
| `WHEEL_SPEED_STATUS` | `actual_speed[4]_f32_le` in M1=RR, M2=RF, M3=LR, M4=LF order |
| `IMU_CAL_STATUS` | `stage_u8, radar_pwm_u8, sample_count_u32_le, sample_total_u32_le, error_code_u8` |
| `IMU_TELEMETRY` | `sensor_id_u8, flags_u8, timestamp_ms_u32_le, accel[3]_f32_le, vector[3]_f32_le` |
| `ATTITUDE` | `schema_u8=2, flags_u8, reserved_u16=0, timestamp_ms_u32_le, sample_sequence_u32_le, primary_euler[3], primary_quat[4], redundant_euler[3], redundant_quat[4], delta_euler[3]` float32 LE |

Sensor IDs are global: `1=LSM303`, `2=BMI323`. `IMU_TELEMETRY.flags` has
bit 0 accel-valid, bit 1 second-vector-valid (LSM303 magnetometer or BMI323
gyro), and bit 2 online. The current runtime produces two 30-byte telemetry
frames and one 80-byte `ATTITUDE` frame; it does not emit a dual-status frame.

## Reliability Rules

The non-blocking parser accepts fragmented and concatenated byte streams.
After HCS failure it immediately resumes SOF search without consuming the
advertised length. FCS/EOF failures also resynchronize at the most recent
possible byte.

The link manager has four pending transaction slots. `ACK_REQUIRED` messages
use a 500 ms timeout and at most three retransmissions. ACK correlation uses
the exact `(ack_can_id, ack_seq)` pair. HCS/FCS errors and transaction timeout
add 8 to REC or TEC respectively; successful receive and successful ACK reduce
their corresponding counter by 1. Link state is active below 32, warning from
32, passive from 128, and bus-off from 256. Bus-off invokes the platform UART
recovery callback before a controlled link recovery.

## Evidence Boundary

Host codec tests and CM7/S3 builds validate source integration only. They do
not prove UART electrical behavior at 921600 baud, target DMA behavior, BLE
notifications, radar/PWM response, sensor output, or vehicle behavior.
