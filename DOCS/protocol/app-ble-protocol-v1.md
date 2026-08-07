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
`ATTITUDE 0x11`, calibration types `0x12/0x13`, radar/PWM types `0x14..0x19`.

## Status

Implemented in the App model. Cross-boundary compatibility with current S3 C
parsers is not proven because S3 uses a different source frame layout/type set.

## Modification Notes

Keep explicit little-endian serialization and the 128-byte bound. Any approved
contract change must update the S3 bridge, STM endpoint, App decoder, and
canonical documentation together.
