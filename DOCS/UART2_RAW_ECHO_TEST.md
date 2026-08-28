# UART2 Raw Echo Test (Archived)

Status: superseded by the forced GPIO18 level-test firmware. The active CMake
targets now build `DOCS/GPIO18_LEVEL_TEST.md`, not this UART Echo image. The
old task-specific Echo build caches are no longer in the workspace. This
document is retained only as historical behavior reference.

## Purpose

This forced image isolates the STM32H757 CM7 to ESP32-S3 electrical route:

| Endpoint | UART | Pins | Configuration |
| --- | --- | --- | --- |
| STM32H757 CM7 | USART2 | PA2 TX, PA3 RX | 115200-8-N-1, no flow control |
| ESP32-S3 | UART2 | GPIO17 TX, GPIO18 RX | 115200-8-N-1, no flow control |

Connect the link crossed and with a common ground: S3 GPIO17 to STM PA3, STM
PA2 to S3 GPIO18, and S3 GND to STM GND. Do not connect either UART TX to the
other UART TX.

The CM7 test target does not compile `uart_link`, USART2 DMA streams, SRP,
`s3_service`, motor-board transport, or the production BSP/UART SRP log bridge.
It only initializes USART1 and USART2. The S3 target does not link
`stm_uart`, `smartcar_service`, BLE, radar, or project protocol components.
Radar UART1/GPIO44 and motor behavior are outside this test image.

The S3 UART driver is the ESP-IDF FIFO/interrupt driver used by
`uart_read_bytes()` and `uart_write_bytes()`; the test application does not
link UART DMA or GDMA runtime services. The S3 linker script still exports an
absolute `GDMA` peripheral-address symbol, which is not executable DMA code.

## Behavior

- CM7 polls USART2 for one byte with a 10 ms timeout, immediately writes that
  byte back, and prints `[STM_RX_ECHO]` plus the hexadecimal byte on USART1.
- CM7 sends the exact bytes `STM_PING\n` once each second.
- S3 logs all UART2 bytes as text and hexadecimal bytes, then sends the exact
  bytes `S3_HELLO\n` once each second.

## Historical Build Note

The archived Echo configuration has no current firmware artifact. Do not use
its former task-specific build paths. The active PA2-to-GPIO18 test must be
built only according to `DOCS/GPIO18_LEVEL_TEST.md`; its STM32 flash image is
always `STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf`.

## Bench Acceptance

1. Flash the two test artifacts, then observe the STM USART1 debug console and
   the S3 USB Serial/JTAG console.
2. S3 must report `S3 Sent: S3_HELLO` and receive both `STM_PING` and echoed
   `S3_HELLO` bytes; inspect `S3_RX_HEX` rather than relying on text output.
3. STM USART1 must emit one `[STM_RX_ECHO]` entry per byte of `S3_HELLO\n`.
4. If either direction is absent, capture TX/RX at the pins with a logic
   analyzer before changing protocol, DMA, or application code.
