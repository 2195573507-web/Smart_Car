# SRP v4 Protocol Specification

Status: **Active**

SRP v4 (Streamlined Robot Protocol) is the point-to-point UART protocol
between STM32H757 CM7 and ESP32-S3. The App BLE envelope is a separate
protocol and is not a raw SRP frame.

## Frame

Every frame is 12 bytes plus payload, little-endian:

```text
MAGIC u16 | LEN u16 | HEADER u32 | PAYLOAD[N] | CRC16 u16 | EOF u16
AA 55       N         4 bytes       N bytes       CRC LO HI    0D 0A
```

`MAGIC=0x55AA`, `EOF=0x0A0D`, and `LEN` is 0..500. CRC-16/CCITT-FALSE
(`poly=0x1021`, `init=0xFFFF`, `refin=false`, `refout=false`, `xorout=0`)
covers the six bytes beginning at `LEN` through the complete payload. CRC and
EOF are excluded from the CRC input.

The shared implementation is `Common/SRP/`. C structures use
`#pragma pack(push, 4)` and alignment assertions. Wire encoding is byte-wise;
consumers must not dereference an unaligned packed member directly.

## Header

The 32-bit header is accessed through macros in `srp_def.h`:

```text
[31:24] priority   [23:16] type   [15:8] sequence   [7:0] flags
```

Priority values are `0=EMERGENCY`, `1=COMMAND`, `2=TELEMETRY`, and `3=LOG`.
The sequence is an 8-bit value and wraps naturally. Flags are:

| Bit | Name | Meaning |
| ---: | --- | --- |
| 0 | `SRP_FLAG_TLV` | Payload contains TLVs rather than a fixed struct |
| 1 | `SRP_FLAG_ACK_REQUIRED` | Sender tracks and retries the transaction |
| 2 | `SRP_FLAG_ACK` | Payload is a successful fast response |
| 3 | `SRP_FLAG_ERROR` | Payload is an error fast response |
| 4..7 | reserved | Must be zero |

UART is point-to-point, so source and destination IDs are not serialized.

## Registry And Payloads

Message IDs and exact active payload lengths are defined in
`Common/SRP/include/srp_registry.h`. Important IDs include:

| ID | Name | Direction |
| ---: | --- | --- |
| `0x01` | `CAL_EVENT` | STM -> S3 |
| `0x02` | `MOTOR_CMD` | S3 -> STM |
| `0x03` | `PID_PARAMS_CMD` | S3 -> STM |
| `0x04..0x06` | wheel/chassis commands | S3 -> STM |
| `0x17` | `CHASSIS_HEADING_CMD` | S3 -> STM |
| `0x07` | `BOOT_READY` | STM -> S3 |
| `0x10..0x16` | IMU, attitude, chassis and wheel status | STM -> S3 |
| `0x20..0x21` | radar status/ready | both |
| `0x30` | `LOG` | STM -> S3 |
| `0x70` | `SYS_CONFIG` | both |
| `0x7E..0x7F` | `ACK`/`ERROR` | both |

Fixed payloads retain the existing business layouts. Floating-point values
are IEEE-754 binary32 in little-endian order and are serialized with the
`srp_wire_*` helpers.

`CHASSIS_HEADING_CMD` is an ACK-required 12-byte command:

```text
target_v_mm_s_f32_le | target_yaw_deg_f32_le | flags_u32_le
```

`flags` must be zero. The STM32 accepts only finite values and applies the
target in its local attitude/safety-gated heading controller; it does not
restore the target after a link-loss or stop event.

## TLV

TLV payloads are repeated `tag u8 | length u8 | value[length]` records. An
unknown tag is skipped. A truncated record invalidates the message. The
current `SYS_CONFIG` record is:

```text
tag=0x01, length=4, value=u32 baud rate (little-endian)
```

The firmware accepts `921600` and `115200` baud. A valid configuration is ACKed
at the current rate, then each endpoint flushes pending transport data and
switches after a 20 ms guard window. Invalid or duplicate baud tags receive
`SRP_FAST_RESP_INVALID_PARAM`.

## Responses And Reliability

The four-byte fast response payload is `ack_type u8 | reserved u8 |
ack_sequence u8 | status_code u8`. Status `0` is success. A transaction has
four pending slots, a 500 ms timeout, and at most three retries. REC/TEC
counters classify parser/transport health and trigger UART recovery at bus-off.

## Implementation And Evidence

`srp_codec.c` provides encode/decode, a fragmented stream parser, CRC checks,
and the TLV iterator. `srp_link.c` provides transaction tracking and recovery.
Host tests cover the CCITT-FALSE vector `123456789 -> 0x29B1`, exact framing,
fragmentation, CRC/EOF rejection, unknown TLVs, sequence wrap, and 4-byte
alignment. Firmware builds validate source integration; UART electrical
behavior, DMA runtime, EMG latency, dynamic baud synchronization, and 24-hour
stress remain bench acceptance tests.
