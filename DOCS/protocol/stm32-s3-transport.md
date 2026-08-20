# STM32H757 To ESP32-S3 Transport

## Active Contract

The active UART2 transport is SCBP-CAN at 921600 baud, 8N1, with no hardware
flow control:

```text
STM32H757 USART2 PA2/PA3 <-> ESP32-S3 UART2 GPIO17/GPIO18
5A A5 | CAN_ID_LE | FLAGS | LEN | HCS | SEQ | PAYLOAD | FCS_LE | 0D 0A
```

The complete frame and payload definitions live in
`Common/SCBP_CAN/include/scbp_protocol_defs.h`. Neither endpoint owns a local
copy of SCBP-CAN wire structures or message IDs.

## Endpoint Ownership

| Endpoint | Responsibility |
| --- | --- |
| STM32H757 CM7 | Sensor/calibration producer, DualAHRS producer, log producer, and `RADAR_PWM_READY` consumer. |
| ESP32-S3 | UART gateway, radar calibration owner, App BLE re-enveloper, and link recovery owner on its UART side. |
| App BLE | Separate `AA 01 ... CRC16 ... 55` envelope; never a raw SCBP-CAN frame. |

STM32 uses a 512-byte circular RX DMA buffer with IDLE receive events,
cache invalidation, a software ring, and UART/DMA error recovery. ESP32-S3
uses the ESP-IDF UART2 driver plus a bounded newest-data queue. Both endpoints
feed arbitrary byte chunks into the shared stream parser.

## Calibration Transaction

1. STM emits `BOOT_READY(0x007)` with `ACK_REQUIRED` after it reaches the
   zero-radar wait state.
2. S3 admits the event into `radar_calibration_manager` and replies with a
   fast ACK or ERROR.
3. S3 sets radar calibration PWM to zero, then sends transactional
   `RADAR_PWM_READY(0x302, speed_percent=0)`.
4. STM admits the value only at the expected state and replies with fast ACK
   or ERROR.
5. After its static window completes, STM sends `CAL_EVENT(0x001, event=1)`
   transactionally. S3 releases the radar calibration lock only after it
   admits that event.

The event and response logic is idempotent for retries. `CAL_EVENT=0x001` is
the SCBP-CAN 10-bit message value; do not use historical V3 value `0x0401`.

## Telemetry And Logs

STM emits only these IMU-related UART payloads:

- `ATTITUDE(0x201)`: exactly 80-byte schema-2 DualAHRS.
- `IMU_CAL_STATUS(0x202)`: exactly 11 bytes.
- `IMU_TELEMETRY(0x207)`: exactly 30 bytes, with sensor ID 1 or 2.

S3 validates lengths and identifiers before re-enveloping selected payloads
for App BLE. `LOG(0x3F0)` is converted to the existing bounded FFE3 log
notification format. No `0x0200`, `0x0208`, 30-byte attitude, bias/result
transport, or `SC_TYPE_*` compatibility code participates in this link.

## Failure Handling

HCS failure never consumes the advertised length. ACK-required frames retry
every 500 ms up to three times. Error counters progress through active,
warning, passive, and bus-off states. A bus-off callback flushes or rebuilds
the endpoint UART receive path before link counters are reset.

Build and host-test results are source evidence only; device UART, DMA, radar,
BLE, sensor, and vehicle acceptance remain separate validation work.
