# Protocol Module

## Function

Describe the framing and parser ownership at each transport boundary.

## Source Location

- STM32: `STM32H757/Middleware/Communication/SmartCar_Frame/`
- S3: `ESPS3/components/smartcar_protocol/`
- App: `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/Model/SmartCarProtocol.swift`

## Entry Files

`sc_frame.c`, `frame.c`, `parser.c`, and Swift `SmartCarProtocol.swift`.

## Data Input

Arbitrary fragmented byte chunks from UART or BLE.

## Data Output

Validated frame views, parser callbacks, CRC/error counters, and encoded bytes.

## External Interfaces

`sc_frame_encode/decode`, `sc_frame_parser_init/feed`, Swift
`SmartCarProtocol.encode`, and `SmartCarProtocol.Parser.feed`.

## Dependencies

`stdint`, bounded buffers, CRC16-MODBUS, transport-owned callbacks.

## Current STM32-S3 Contract

Status: CONFIRMED source/build contract.

The STM32H757 to ESP32-S3 UART boundary now uses
[SCBP-V3](SCBP_V3_REFERENCE.md). Its fixed frame has 14 bytes of overhead,
uses 16-bit `MSG_ID`, and carries explicit priority, source, destination,
sequence, flags, length, and CRC fields. `BOOT_READY=0x0007` is STM32 to S3;
`RADAR_PWM_READY=0x0302` is S3 to STM32; and `PWM_SET=0x0101` remains an
active command rather than a readiness signal.

`ATTITUDE=0x0201` accepts either the legacy 30-byte payload or the explicit
schema=2 80-byte DualAHRS payload. STM adds the transport timestamp and
source/status fields only for the legacy adapter. S3 validates the exact
length/schema pair, then rebuilds only the separate App BLE envelope with the
same payload bytes. It does not convert attitude fields, units, or lengths.
The App BLE envelope remains an independent contract.

## Legacy V1 Snapshot (Pre-SCBP-V3)

Both C endpoints implement the STM-S3 source envelope. The App implements the
BLE envelope. `smartcar_service/command_bridge.c` is the named translation
layer that validates STM telemetry, rebuilds the App envelope, and sends it
through the existing BLE notify path. This is source/build evidence, not
physical BLE/UART acceptance.

## Legacy ATTITUDE Payload (Pre-SCBP-V3)

The AHRS owns radian state internally. `ATTITUDE` is carried as type `0x21` on
the STM32-S3 frame and type `0x11` on the App BLE frame. Both payloads are 26
bytes and use explicit IEEE-754 float32 little-endian fields:

| Offset | Field | Unit |
| ---: | --- | --- |
| 0 | `roll_rad` | radian |
| 4 | `pitch_rad` | radian |
| 8 | `yaw_rad` | radian |
| 12 | `roll_deg` | degree |
| 16 | `pitch_deg` | degree |
| 20 | `yaw_deg` | degree |
| 24 | `valid` | u8, 0/1 |
| 25 | `source` | u8, source ID |

This 26-byte layout is retained only to explain the pre-V3 producer input.
The active UART payload is the 30-byte SCBP-V3 layout in
[SCBP_V3_REFERENCE.md](SCBP_V3_REFERENCE.md); S3 forwards its payload bytes
unchanged while it re-envelopes BLE.

## Boundary Notes

The STM-S3 SCBP-V3 and App BLE envelopes are deliberately distinct. The C
transport starts with `AA 55`; the App envelope starts with `AA`, version
`01`, and ends with `55`. The S3 bridge translates envelopes only. Its source
message mapping must not alter the SCBP ATTITUDE payload.

## Modification Notes

Never change a SCBP-V3 frame byte, `MSG_ID`, length, CRC range, or payload
offset in one UART endpoint only. New UART functions add a documented
`MSG_ID`; they do not change the V3 layout. Do not call a source-only check an
end-to-end acceptance.
