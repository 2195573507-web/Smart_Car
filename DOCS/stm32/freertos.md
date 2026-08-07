# FreeRTOS Module

## Function

Schedule STM32 sampling, logging, UART link, S3 service, and health tasks under
the generated CM7 FreeRTOS port.

## Source Location

`STM32H757/Middleware/FreeRTOS/`, `Application/RTOS/imu_runtime.c`,
`Communication/UART_Link/uart_link.c`, `Communication/Services/s3_service.c`.

## Entry Functions

`vTaskStartScheduler`, `xTaskCreate` sites in `imu_runtime_start`,
`uart_link_task_start`, `s3_service_start`, and RTOS health hooks.

## Inputs

Task creation requests, timer ticks, queue/ring data, HAL callbacks, and health
events.

## Outputs

Periodic sampling, parser servicing, logs, stack/heap watermarks, and retained
fatal events.

## Public Interfaces

`xTaskCreate`, `vTaskStartScheduler`, task entry functions, and the
`rtos_health_*` query/record interfaces.

## Dependencies

FreeRTOS kernel/portable layer, HAL tick, mutexes, task notifications/critical
sections, logger.

## Current Status

Source/task scaffolding exists. Stack, latency, scheduler stability, and live
runtime evidence are unverified.

## Known Issues

Task stack changes must be evidence-driven. Do not solve ownership or blocking
bugs with blanket stack increases.

## Modification Notes

Keep ISR-safe boundaries, task context rules, timeouts, and health hooks intact.
