# Smart_Car Communication Architecture

## Required Chain

```text
STM32H757 CM7
    |
    | USART2 PA2/PA3 <-> ESP32-S3 UART2 GPIO17/18, 921600 8N1
    | SCBP-CAN: shared parser, CRC, retry, REC/TEC health state
    v
ESP32-S3
    |
    | BLE GATT: FFE1 command write, FFE2 telemetry notify, FFE3 log notify
    | independent App BLE envelope
    v
macOS SmartCar Control App
```

This is a source-defined ownership chain, not physical-link acceptance.

## Transport Boundaries

| Boundary | Wire envelope | Owner |
| --- | --- | --- |
| STM32 <-> S3 | `5A A5 | CAN_ID_LE | FLAGS | LEN | HCS | SEQ | PAYLOAD | FCS_LE | 0D 0A` | `Common/SCBP_CAN` |
| App <-> S3 BLE | `AA | 01 | TYPE | LEN_LE | PAYLOAD | CRC16_LE | 55` | App parser/encoder |
| S3 log notify | Existing SmartCar FFE3 log envelope | `smartcar_log` |

The S3 is the translation boundary: it validates SCBP-CAN data and constructs
a distinct BLE envelope for selected telemetry. The two frames are not
byte-compatible and raw UART frames are never forwarded to BLE.

## Runtime Ownership

- STM32 owns sensor sampling, static calibration, attitude production, local
  safety decisions, and consumption of `RADAR_PWM_READY`.
- S3 owns UART2 queueing, radar calibration progression, radar status, BLE
  transport, and S3-side bus-off recovery.
- The App owns operator interaction; it does not receive a SCBP-CAN wire frame.

STM32 UART2 uses circular DMA plus IDLE receive handling and D-cache
invalidation. S3 uses ESP-IDF UART2 driver buffers plus a bounded software
queue. Both use the same non-blocking SCBP-CAN parser and link manager.

## Failure Semantics

- Invalid HCS is rejected before a declared payload is consumed.
- HCS/FCS failures and transaction timeouts feed REC/TEC health counters.
- Transactional calibration messages correlate fast responses by CAN ID and
  sequence, with 500 ms retry timing and up to three retransmissions.
- Bus-off triggers endpoint UART recovery and then a controlled link reset;
  it is not an assertion that a physical UART fault has been fixed.

Source builds do not establish electrical UART directionality, DMA behavior,
BLE delivery, radar/PWM behavior, sensor data, or vehicle safety.
