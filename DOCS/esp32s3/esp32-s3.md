# ESP32-S3 Module

## Function

Provide STM UART gateway transport, BLE GATT transport, radar UART/PWM, and
FreeRTOS service scheduling.

## Source Location

`ESPS3/main/`, `ESPS3/components/`

## Entry File

`ESPS3/main/main.c::app_main` initializes NVS, STM UART2, BLE, radar UART/GPIO4
PWM, and the SmartCar service.

## Inputs

STM UART2 bytes, BLE writes/CCC changes, radar UART1 bytes, timer/task events.

## Outputs

STM UART frames, BLE notifications/logs, radar state/logs, service statistics.

## Public Interfaces

`stm_uart_*`, `s3_ble_*`, `smartcar_service_init`, radar init/parser/control
interfaces, and shared `scbp_*` protocol interfaces.

## Dependencies

ESP-IDF, FreeRTOS, Bluedroid GATT, UART/LEDC/GPIO drivers, smartcar protocol,
radar components.

## Current Status

Source-established gateway/radar scaffold. BLE transport, App command callback
registration, SCBP-CAN parsing, and selected IMU telemetry relay are present.
Physical UART, BLE, radar, and sensor behavior remain unverified.

## Known Issues

The gateway must keep SCBP-CAN UART frames separate from the App BLE envelope;
it is not an end-to-end raw-frame relay. `radar_uart.c` remains the independent
radar UART1/GPIO44 path.

## Modification Notes

Keep radar UART1/GPIO44 and PWM GPIO4 separate from STM UART2 GPIO17/18. Keep
raw byte tasks, parsers, and service callbacks in separate ownership layers.
