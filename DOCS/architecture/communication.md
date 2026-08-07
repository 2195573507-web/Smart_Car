# Smart_Car Communication Architecture

## Required Chain

```text
STM32H757
    |
    | USART2 PA2/PA3 <-> ESP32-S3 GPIO17/18, 115200 8N1
    v
ESP32-S3
    |
    | BLE GATT service FFE0: FFE1 write, FFE2 notify, FFE3 log notify
    v
macOS SmartCar Control App
```

The route above is the intended/source-defined ownership chain. It is not a
claim that the physical cable, BLE session, or end-to-end command/telemetry
path has passed acceptance.

## STM32 Side

- `uart_link.c` owns a manually initialized `USART2` HAL handle, a 512-byte RX
  ring, a 128-byte read chunk, and a 5 ms receive-to-idle timeout.
- `s3_service.c` feeds the STM-S3 parser and handles PONG, radar PWM ready, and
  calibration ACK events.
- `USART1` PA9/PA10 is a separate text/log path used by `bsp_uart` and the
  standalone SmartCar Logger.

## S3 Side

- `stm_uart.c` owns UART2 GPIO17/18, 115200 8N1, two 4096-byte driver buffers,
  and a 4096-byte newest-data storage ring.
- `smartcar_service` parses the STM-S3 source envelope and responds to PING;
  calibration/radar events are routed to `radar_calibration_manager`.
- `s3_ble.c` owns GATT transport and notification readiness. Its RX callback
  API exists, but current source search does not show an installed App command
  parser/bridge.

## Frame Boundaries

| Boundary | Current source envelope | Parser |
| --- | --- | --- |
| STM32 <-> S3 | `AA 55 01 TYPE LEN PAYLOAD CRC_LE` | C `sc_frame` |
| App <-> BLE GATT | `AA 01 TYPE LEN PAYLOAD CRC_LE 55` | Swift `SmartCarProtocol.Parser` |
| S3 log notify | `AA 55 01 SOURCE LEVEL TIMESTAMP LEN PAYLOAD CRC_LE` | Swift `SmartCarLogParser` |

The two control/telemetry envelopes share CRC16-MODBUS but are not byte
compatible. Do not describe them as one wire format until a synchronized,
tested bridge is implemented.

## Failure Semantics

- Parser errors must be counted and logged with reason, type, length, and CRC
  details where available.
- UART overflow/drop counters are diagnostic evidence, not an automatic claim
  of lost safety commands.
- A stale link must result in local policy handling at the owning endpoint;
  documentation must name the owner instead of assuming BLE or App recovery.
