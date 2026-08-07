# STM32-S3 Transport Frame

## Function

Record the C frame currently used by the STM32 UART link and S3 service.

## Source Location

- STM32: `STM32H757/Middleware/Communication/SmartCar_Frame/sc_frame.c`
- S3: `ESPS3/components/smartcar_protocol/frame.c`

## Frame

```text
AA | 55 | 01 | TYPE | LEN_LO | LEN_HI | PAYLOAD[LEN] | CRC_LO | CRC_HI
```

`SC_FRAME_MAX_PAYLOAD` is 128 and overhead is 8. CRC16-MODBUS covers the
version through payload bytes. The C parsers accept fragmented chunks and
report header/version/length/CRC errors through callbacks.

## Current Type Set

`PING 0x01`, `PONG 0x02`, `ACK 0x03`, `PWM_READY 0x10`,
`RADAR_PWM_READY 0x16`, `RADAR_PWM_ACK 0x17`, `CAL_EVENT 0x18`,
`CAL_EVENT_ACK 0x19`, `STM_BOOT_READY 0x1C`, `IMU_STATUS 0x20`,
`ATTITUDE 0x21`, `LOG 0x30`.

## Status

Source-established on both C endpoints. `smartcar_service` currently handles
PING/PONG, logs, boot/radar/calibration events; telemetry bridge completion is
not established.

## Modification Notes

Keep the C frame independent from App BLE framing unless a named translation
layer is added and tested. Preserve bounded buffers, parser resynchronization,
CRC diagnostics, and local safety ownership.
