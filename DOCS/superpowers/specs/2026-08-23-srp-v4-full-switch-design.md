# SRP v4 Full-Switch Design

Status: CONFIRMED  
Date: 2026-08-23  
Scope: STM32H757 USART2 <-> ESP32-S3 UART2 gateway link

## Goal

Replace the active SCBP-CAN UART implementation with SRP v4 while keeping the
existing STM32 USART2 PA2/PA3 and ESP32 GPIO17/GPIO18 route, and preserving the
upper-layer motor, PID, IMU, calibration, radar, BLE, and logging behavior.

The migration is a hard source switch. SCBP-CAN executable source, tests, and
build entries are removed. Historical protocol documents remain only as
deprecated records so the repository does not lose engineering history.

## Wire Contract

Each frame is 12 bytes plus payload, little-endian:

```text
MAGIC u16 | LEN u16 | HEADER u32 | PAYLOAD[LEN] | CRC16 u16 | EOF u16
AA 55       N         4 bytes       N bytes          2 bytes     0D 0A
```

`MAGIC` is `0x55AA`, `LEN` is 0..500, `EOF` is `0x0A0D`, and CRC-16/CCITT-
FALSE covers `LEN + HEADER + PAYLOAD`. Header fields are accessed only through
macros:

```text
[31:24] priority, [23:16] type, [15:8] sequence, [7:0] flags
```

Bit 0 of flags selects TLV payload mode. All C protocol structures use
`#pragma pack(push,4)` and static alignment/size checks. Codec code performs
explicit byte-wise serialization and never dereferences an unaligned packed
wire member.

## Type Registry

| Type | Meaning |
| ---: | --- |
| 0x01 | CAL_EVENT |
| 0x02 | MOTOR_CMD (four-wheel command) |
| 0x03 | PID_PARAMS |
| 0x04 | SINGLE_WHEEL_CMD |
| 0x05 | MASTER_SPEED_CMD |
| 0x06 | CHASSIS_SPEED_CMD |
| 0x07 | BOOT_READY |
| 0x10 | IMU_TELEM |
| 0x11 | ATTITUDE |
| 0x12 | IMU_CAL_STATUS |
| 0x13 | POWER_STATUS |
| 0x14 | WHEEL_SPEED_STATUS |
| 0x15 | CHASSIS_STATE |
| 0x16 | WHEEL_CONTROL_STATUS |
| 0x20 | RADAR_STATUS |
| 0x21 | RADAR_PWM_READY |
| 0x30 | LOG |
| 0x70 | SYS_CONFIG |
| 0x7E | ACK |
| 0x7F | ERROR |

Existing active payload byte layouts remain unchanged unless a new SRP TLV is
explicitly specified. `SYS_CONFIG` uses TLV tag `0x01`, length 4, and a
little-endian u32 baud rate.

## Shared Library

`Common/SRP/` provides:

- `srp_def.h`: constants, packed frame/storage types, header macros, priority,
  flag, and error definitions.
- `srp_registry.h`: type IDs, payload length rules, and TLV tags.
- `srp_codec.c/h`: CRC-16/CCITT-FALSE table, SRP encode/decode, fragmented
  stream parser, and TLV iterator. A MODBUS CRC helper remains for the
  independent App BLE envelope.
- `srp_link.c/h`: four pending transaction slots, ACK/ERROR correlation by
  `(type, sequence)`, timeout/retry policy, REC/TEC state, and bus recovery.

## STM32 Transport

`uart_link.c` retains the public send/read/recovery surface but changes the
implementation to:

- USART2 921600-8N-1, unchanged pins.
- RX DMA1 Stream0 circular buffer with IDLE/half/full event handling, D2 SRAM
  placement, cache maintenance, and software ring delivery.
- Independent TX DMA stream and four statically bounded priority queues.
- A TX worker that starts `HAL_UART_Transmit_DMA` and never blocks RX.
- EMG priority over CMD/TELE/LOG. An active LOG transfer may be aborted and
  dropped to send EMG immediately; CMD and TELE transfers are frame-boundary
  protected.

The generated interrupt file receives only the required TX DMA IRQ dispatcher;
no GPIO, IOC, or peripheral route is changed.

## ESP32 Transport

`stm_uart.c` keeps UART2 GPIO17/18 and the existing receive API while adding:

- Independent 2 KB driver RX/TX buffers.
- Dedicated RX task for `uart_read_bytes` and bounded receive storage.
- Dedicated TX task for four priority queues and `uart_write_bytes`.
- Non-blocking `stm_uart_send`; no `uart_wait_tx_done` in business tasks.
- Recovery and baud-rate reconfiguration hooks that flush queues/storage before
  applying the new rate.

`command_bridge.c` remains the SRP endpoint dispatcher and BLE gateway. It
continues to call existing chassis/PID/calibration/radar services.

## Baud-Rate Change

The sender transmits `SYS_CONFIG` at the current rate. The receiver validates
the TLV, returns ACK at the current rate, drains its pending transport data,
then switches. The sender switches after the ACK and a bounded guard delay.
Both sides clear stale RX bytes and rearm their parser. The operation is
bounded to 500 ms in source logic; physical synchronization remains a bench
acceptance item.

## Verification

Host tests cover the CCITT-FALSE standard vector (`123456789` -> `0x29B1`),
exact magic/EOF bytes, fragmented and concatenated streams, CRC/EOF rejection,
unknown TLV skipping, sequence wrap, and 4-byte alignment. CM7/CM4 and ESP-IDF
builds provide source/build evidence. UART stress, EMG latency, dynamic baud
switching, and 24-hour CRC stability require matching flashed images and live
captures and are not inferred from builds.

