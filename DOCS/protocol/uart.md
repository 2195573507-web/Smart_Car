# UART Module

## Function

Own raw byte transport and buffering between STM32H757, ESP32-S3, radar, and
the independent STM logger paths.

## Source Location

- STM transport: `STM32H757/Middleware/Communication/UART_Link/`
- STM log BSP: `STM32H757/BSP/UART/`
- S3 transport: `ESPS3/components/stm_uart/`
- S3 radar: `ESPS3/main/radar/radar_uart.c`
- macOS logger: `Tools/SmartCar_Logger_MAC/Sources/`

## Entry Files

`uart_link.c`, `stm_uart.c`, `radar_uart.c`, `SerialPortService.swift`.

## Inputs

Raw bytes, timeouts, and HAL/ESP-IDF statuses from the STM32-S3 link, radar,
and independent logger paths.

## Outputs

Bounded ring-buffer bytes, parser input, text logs, and transport statistics.

## Interfaces

STM `uart_link_init/send/read/get_stats/task_start`; S3
`stm_uart_init/send/receive_nonblock/get_stats`; radar `radar_uart_init`; Mac
`SerialPortService.open/read` and `LoggerSession.append`.

## Dependencies

STM HAL/FreeRTOS, ESP-IDF UART/GPIO/LEDC, parser/service consumers, and macOS
termios/DispatchSourceRead for the standalone logger.

## Current Status

Transport implementations exist. STM-S3 route is USART2/UART2 at 115200; radar
uses S3 UART1/GPIO44; logger uses STM USART1/PA9-PA10. Physical signals and
end-to-end delivery are unverified.

## Known Issues

The current IOC retains PD3/PD4 legacy labels while generated USART2 MSP and
transport source use PA2/PA3. Treat this as a confirmation item, not a reason
to alter GPIO or IOC in a documentation task.

## Modification Notes

Keep raw transport separate from parsing. Preserve overflow/drop counters and
timeouts; do not solve a wiring or admission problem with retries or larger
buffers without evidence.
