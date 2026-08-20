# App BLE Protocol V1

## Boundary

App BLE is not SCBP-CAN. Its envelope remains:

```text
AA | 01 | TYPE | LEN_LO | LEN_HI | PAYLOAD[LEN] | CRC_LO | CRC_HI | 55
```

CRC16-MODBUS covers the version through the payload. ESP32-S3 parses this
format for inbound App commands and constructs this format for selected UART
telemetry. It never relays raw SCBP-CAN bytes to BLE.

## Current S3 Outbound Mapping

| SCBP-CAN UART message | App BLE type | Exact payload |
| --- | ---: | --- |
| `ATTITUDE(0x201)` | `0x11` | 80-byte schema-2 DualAHRS, byte-preserving |
| `IMU_CAL_STATUS(0x202)` | `0x12` | 11 bytes |
| `RADAR_STATUS(0x301)` | `0x15` | 2 bytes |
| `IMU_TELEMETRY(0x207)` | `0x27` | 30 bytes |
| App command response | `0x06` | `acknowledged_type_u8, result_u8` |

S3 also emits radar status from its radar-control owner on the same 2-byte
App type `0x15` payload. Text logs remain on the separate FFE3 notification
path in the existing SmartCar log envelope.

## Compatibility Boundary

The UART migration removes V3 compatibility only from the STM32-S3 link. It
does not redefine the App BLE envelope or its independent parser. However,
the S3 bridge no longer emits legacy UART-derived IMU status, dual-status,
bias, calibration-result, or 30-byte attitude payloads. App consumers must
expect the 80-byte schema-2 attitude payload for new UART-originated data.

BLE delivery and App rendering are not demonstrated by firmware builds.
