# Boot Module

## Function

Emit normalized early-boot events and retained-fault/RTOS-health diagnostics.

## Source Location

`STM32H757/Middleware/Boot/boot_log.c/.h`, `CM7/Core/Src/main.c`,
`System/Task/rtos_health.c`.

## Entry Functions

`boot_log_start`, `boot_log_uart_ready`, `boot_log`,
`report_retained_fault`, `report_retained_rtos_health`.

## Inputs

Startup stages, UART readiness, retained fault/health records.

## Outputs

Early buffered events, USART1 log lines, and boot state markers.

## Public Interfaces

`boot_log_start`, `boot_log_uart_ready`, and `boot_log`.

## Dependencies

BSP USART1, logger, timer, RTOS health hooks.

## Current Status

Source-established startup sequence: system, clock, GPIO, UART, RTOS, and ready
markers. Reset capture is not part of this audit.

## Known Issues

Early log buffering and logger connection ordering must remain intact.

## Modification Notes

Keep boot logging non-blocking where required and do not hide initialization
failures behind a generic ready event.
