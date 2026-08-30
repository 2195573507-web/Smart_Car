# App BLE Protocol V1

## Boundary

App BLE is not the STM32-S3 SRPv4 link. Its independent envelope remains:

```text
AA | 01 | TYPE | LEN_LO | LEN_HI | PAYLOAD[LEN] | CRC_LO | CRC_HI | 55
```

CRC16-MODBUS covers the version through the payload. ESP32-S3 parses this
format for inbound App commands and constructs this format for selected SRPv4
telemetry. It never relays raw SRPv4 bytes to BLE.

## Current S3 Outbound Mapping

| SRPv4 UART message | App BLE type | Exact payload |
| --- | ---: | --- |
| `ATTITUDE(0x11)` | `0x11` | 80-byte schema-2 DualAHRS, byte-preserving |
| `IMU_CAL_STATUS(0x12)` | `0x12` | 11 bytes |
| `WHEEL_SPEED_STATUS(0x14)` | `0x16` | 16 bytes, four float32 LE mm/s values |
| `RADAR_STATUS(0x20)` | `0x1A` | 2 bytes |
| `POWER_STATUS(0x13)` | `0x1C` | 4 bytes, float32 LE volts |
| `IMU_TELEMETRY(0x10)` | `0x27` | 30 bytes |
| App command response | `0x06` | `acknowledged_type_u8, result_u8` |

App commands use `0x15=WHEEL_SPEED_CMD` with a 16-byte payload and
`0x1B=RADAR_PWM_SET` with one speed-percent byte. PID tuning uses
`0x1D=PID_PARAMS_CMD` with four float32 little-endian values in the order
`kp, ki, kd, max_accel`; the S3 returns App ACK `0x06` only after STM ACK.
Text logs remain on the separate FFE3 notification path in the existing
SmartCar log envelope.

## Compatibility Boundary

The UART migration removes pre-SRP compatibility only from the STM32-S3 link. It
does not redefine the App BLE envelope or its independent parser. However,
the S3 bridge no longer emits legacy UART-derived IMU status, dual-status,
bias, calibration-result, or 30-byte attitude payloads. App consumers must
expect the 80-byte schema-2 attitude payload for new UART-originated data.

BLE delivery and App rendering are not demonstrated by firmware builds.
