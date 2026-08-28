# SRP UART Configuration Guide

## Fixed Hardware Route

| Endpoint | Peripheral | TX | RX | Default |
| --- | --- | --- | --- | --- |
| STM32H757 CM7 | USART2 | PA2 | PA3 | 921600-8-N-1 |
| ESP32-S3 | UART2 | GPIO17 | GPIO18 | 921600-8-N-1 |

GPIO assignments are unchanged by SRP v4. No hardware flow control is used.

## STM32

`uart_link.c` configures independent DMA streams: DMA1 Stream0 circular RX
with USART IDLE/receive-to-idle events, and DMA1 Stream1 normal TX. RX bytes
are copied into a software ring and parsed by the service task. TX has four
static priority queues and a separate worker. DMA buffers are aligned and
placed in D2 SRAM so DMA1 can access them; cache invalidation/cleaning is
performed around DMA ownership changes.

An EMERGENCY frame can abort an active LOG DMA transfer. COMMAND and TELEMETRY
frames are completed at frame boundaries. RX parsing is never blocked by TX.

## ESP32-S3

`stm_uart.c` installs UART2 with independent 2 KiB driver RX and TX buffers,
then starts independent `srp_uart_rx` and `srp_uart_tx` FreeRTOS tasks. The RX
task only reads UART data into bounded storage. The TX task only drains the
four priority queues with non-blocking `uart_write_bytes` calls.

## Baud Change

Send `SYS_CONFIG` with `SRP_FLAG_TLV | SRP_FLAG_ACK_REQUIRED` and one baud TLV.
The receiver validates and queues an ACK on the current rate. After the ACK
has had time to leave the wire, it flushes RX/TX queues and applies the new
rate. The sender applies the same guard after its ACK callback. Both sides then
rearm their parsers. The source guard is 20 ms and the protocol operation is
designed to complete within 500 ms; confirm the actual timing with a logic
analyzer or UART capture.

## Diagnostics

Use `uart_link_get_stats()` and `stm_uart_get_stats()` for byte, overflow,
drop, DMA/HAL, and queue-drop counters. A nonzero CRC/EOF parser counter is a
transport fault, not proof of a business-layer rejection. Builds and host
tests do not establish electrical or long-duration stress acceptance.
