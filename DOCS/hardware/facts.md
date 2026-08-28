# Smart_Car Hardware Facts

## Function

Record source/IOC-visible hardware facts without inventing PCB or electrical
behavior. Each row is explicitly marked as confirmed, reserved, paused, or
unverified.

## STM32H757

| Resource | Current assignment | State |
| --- | --- | --- |
| SPI1 | PA5/PA6/PA7; BMI323 CS PC4 | Source-confirmed, BMI323 paused |
| I2C4 | PD12 SCL, PD13 SDA; LSM303 | Source-confirmed, bus unverified |
| USART1 | PA9 TX, PA10 RX; CH340/debug log | Source-confirmed |
| USART2 | PA2 TX, PA3 RX; STM-S3 link | Generated MSP/source-confirmed |
| TIM3 PWM | PC6..PC9 CH1..CH4 | IOC/source allocation |
| RF encoder | TIM1 PA8/PA9 | Static timer allocation |
| RB encoder | TIM2 PA15/PB3 | Static timer allocation |
| LF/LB encoder | Frozen inputs do not form timer TI1/TI2 pairs | Unsupported for timer mode |
| PD3/PD4 | Legacy named connector GPIO in current IOC | Conflict; not current USART2 AF |

## ESP32-S3

| Resource | Current assignment | State |
| --- | --- | --- |
| STM link | UART2, TX GPIO17, RX GPIO18, 921600 8N1, SRP v4 | Source-confirmed, physical link unverified |
| Radar receive | UART1 RX GPIO44, TX disabled, 115200 | Source-confirmed, radar stream unverified |
| Radar motor | GPIO4 LEDC 10 kHz, 10-bit, configured 0% | Source-confirmed, rotation unverified |
| BLE | Device `SmartCar_S3`, FFE0-FFE3 | Source-confirmed, session unverified |

## Other Hardware

- LSM303, BMI323, X3/X3 Pro, motor drivers, battery, and PCB are named project
  components, but this document does not establish exact board revision,
  voltage, wiring quality, motor response, or safety behavior.
- ROS2 host hardware and USB/serial routing are reserved for a later phase.

## Verification Boundary

Only current source, generated MSP, IOC text, and existing project records were
read. A scope trace, logic analyzer capture, board reset log, BLE capture, or
vehicle test is required before changing an unverified row to hardware
accepted.
