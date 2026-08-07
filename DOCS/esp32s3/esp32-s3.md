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
interfaces, and `sc_frame_*`.

## Dependencies

ESP-IDF, FreeRTOS, Bluedroid GATT, UART/LEDC/GPIO drivers, smartcar protocol,
radar components.

## Current Status

Source-established gateway/radar scaffold. BLE transport and STM/radar raw
transport are present. App command callback registration and IMU telemetry
relay are incomplete in current source.

## Known Issues

`imu_bridge_handle()` is empty. `s3_ble_set_rx_callback()` has no visible
registration caller. Do not describe the gateway as an end-to-end relay.

## Modification Notes

Keep radar UART1/GPIO44 and PWM GPIO4 separate from STM UART2 GPIO17/18. Keep
raw byte tasks, parsers, and service callbacks in separate ownership layers.
