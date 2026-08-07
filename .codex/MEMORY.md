# Smart_Car Stable Memory

This file keeps slow-changing facts. Dated state, unresolved conflicts, and
temporary experiments belong in `PROJECT_STATUS.md`, `DECISION_LOG.md`, or
`docs/history/` instead.

## Project Position

- Phase 1 target: STM32H757 + ESP32-S3 + operator App.
- Phase 2 target: radar + ROS2 + SLAM/navigation; this remains planned.
- STM32H757 is the real-time terminal and final motion authority.
- ESP32-S3 is the gateway and radar-side compute/transport endpoint.
- The current App target is the macOS SwiftUI package at
  `IOS_APP/SmartCar_Control_MAC`; do not infer an iOS migration.

## Hardware Facts

| Item | State | Stable fact |
| --- | --- | --- |
| STM32H757XIH6 | Confirmed in project/IOC | CM7 source and CM4 project exist |
| LSM303 | Active source path | I2C4; accel and magnetometer feed the current IMU manager |
| BMI323 | Paused | SPI1 driver remains; runtime startup logs it as skipped |
| STM-S3 UART | Source-confirmed route | STM32 USART2 PA2/PA3 to S3 UART2 GPIO17/18; physical capture unverified |
| STM log UART | Source-confirmed route | STM32 USART1 PA9/PA10 to the CH340 logger path |
| Radar UART | Source-confirmed route | S3 UART1 receive on GPIO44, 115200 8N1 |
| Radar motor PWM | Source-confirmed route | S3 GPIO4, LEDC 10 kHz; current configured duty is 0% |
| Motor PWM | IOC/source allocation | STM32 TIM3 CH1..CH4 on PC6..PC9; behavior unverified |
| Encoder pairs | Static allocation | RF TIM1 PA8/PA9 and RB TIM2 PA15/PB3; LF/LB timer-mode support is unavailable on frozen nets |
| BLE UUIDs | Source-confirmed | FFE0 service; FFE1 write, FFE2 notify, FFE3 log notify |
| Battery/PCB details | Not established here | Treat as unverified until a current hardware record is named |

The IOC still contains legacy PD3/PD4 labels and an older description of the
STM connector. Current generated USART2 MSP code and `uart_link.h` use PA2/PA3;
this conflict is recorded, not silently resolved by documentation.

## Software Architecture

- STM32 layers: generated HAL/Core -> BSP -> drivers -> sensor/calibration,
  filter/attitude, communication services, RTOS tasks, and application hooks.
- S3 layers: ESP-IDF/FreeRTOS -> STM UART and radar transport -> internal frame
  parser/service -> BLE GATT/log transport.
- App layers: CoreBluetooth manager -> serial parser/decoded models ->
  MainActor telemetry stores/view models -> SwiftUI control/developer views.
- Logger is a separate receive-only macOS tool and is not the vehicle control
  App.

## Communication Facts

There are two distinct current frame envelopes:

1. App BLE model: `AA | 01 | TYPE | LEN_LE | PAYLOAD | CRC16-MODBUS_LE | 55`.
2. STM32-S3 source frame: `AA | 55 | 01 | TYPE | LEN_LE | PAYLOAD |
   CRC16-MODBUS_LE` (no trailing byte in `sc_frame.c`).

Both use CRC16-MODBUS over version through payload, but their byte offsets and
type tables are not interchangeable. The S3 source currently parses the
STM32-S3 frame. The App parser implements the trailing-`55` model.

## Confirmed Design Choices

- Keep STM32 safety and final motion authority local.
- Keep S3 as gateway/radar endpoint; do not move real-time control into BLE or
  ROS2.
- Keep LSM303 as the active IMU path and BMI323 as paused until explicitly
  reactivated.
- Keep BLE UUIDs and the 115200 transport rates as current source contracts.
- Keep calibration and attitude publication behind their readiness states.
- Preserve separate evidence labels for source, build, device, and integration.

## Reserved, Paused, and Unverified

- Reserved: ROS2 autonomy, SLAM, navigation, Wi-Fi gateway APIs, and future
  radar-to-LaserScan integration.
- Paused: BMI323 runtime use, full motor/encoder behavior, and automatic mode.
- Unverified: physical UART wiring, BLE delivery, App command admission,
  telemetry end-to-end flow, sensor electrical response, radar motion, and
  vehicle safety.
