# STM32H757 To ESP32-S3 SRP Transport

Status: **Active**

The physical route is unchanged: STM32 USART2 PA2/PA3 to ESP32-S3 UART2
GPIO17/GPIO18, 921600-8-N-1 by default. SRP v4 uses `AA 55`, a two-byte length,
32-bit header, payload, CCITT-FALSE CRC16 and `0D 0A` EOF.

STM32 uses RX DMA1 Stream0 circular plus IDLE events and an independent TX
DMA1 Stream1 worker with four priority queues. ESP32 uses independent 2 KiB
UART driver buffers and dedicated RX/TX tasks. Neither TX path blocks RX.

`SYS_CONFIG` carries a baud TLV. The receiver ACKs at the old rate, waits the
guard window, flushes queues and switches; the sender switches after its ACK.
See [`../UART_Config_Guide.md`](../UART_Config_Guide.md).

Calibration, telemetry, chassis, PID, radar and logging payload layouts remain
the existing business contracts under `Common/SRP/include/srp_registry.h`.
Build and host-test evidence does not establish electrical, DMA runtime or
vehicle acceptance.
