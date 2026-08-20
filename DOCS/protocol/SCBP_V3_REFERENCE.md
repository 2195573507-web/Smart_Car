# SCBP-V3 Reference (Historical)

Status: deprecated. This document is retained only for historical lookup and
does not define an active firmware interface.

The STM32H757 <-> ESP32-S3 UART path was replaced by SCBP-CAN. Active code no
longer builds or dispatches V3 `AA 55` frames, `SC_TYPE_*` adapters, V3 PING/
PONG, 30-byte ATTITUDE, `0x0200`, `0x0208`, `0x0401`, or `0xF000` transport
IDs. See [protocol.md](protocol.md) for the active contract and
[stm32-s3-transport.md](stm32-s3-transport.md) for endpoint behavior.

The App BLE `AA 01 ... 55` envelope remains separate and is documented in
[app-ble-protocol-v1.md](app-ble-protocol-v1.md).
