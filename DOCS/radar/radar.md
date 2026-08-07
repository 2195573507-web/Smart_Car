# Radar Module

## Function

Acquire and parse YDLIDAR X3/X3 Pro bytes on the ESP32-S3 and control the radar
motor PWM used by the calibration sequence.

## Source Location

`ESPS3/main/radar/` and `ESPS3/components/radar_control/`.

## Entry Functions

`radar_uart_init`, `radar_gpio_monitor_init`, `radar_pwm_init`,
`radar_parser_init/feed/parse_measurement`, `radar_control_init`, and
`radar_control_set_calibration_pwm`.

## Inputs

UART1 RX GPIO44 at 115200 8N1, GPIO44 level, PWM requests, raw radar frames.

## Outputs

Raw bytes/HEX logs, parsed measurements, radar state, and GPIO4 LEDC PWM.

## Public Interfaces

`radar_uart_*`, `radar_gpio_monitor_init`, `radar_pwm_init`,
`radar_parser_*`, and `radar_control_set_calibration_pwm`.

## Dependencies

ESP-IDF UART/GPIO/LEDC, FreeRTOS, radar parser/control, calibration manager.

## Current Status

Raw receive, parser scaffolding, GPIO monitor, and PWM initialization are
source-established. The current configured duty is 0%; rotation and exact
firmware/checksum compatibility are unverified.

## Known Issues

Raw bytes do not prove a valid model frame or ROS2 LaserScan. Radar PWM ready
does not prove motor motion.

## Modification Notes

Keep radar UART1/GPIO44 isolated from STM UART2. Require model-specific capture
before changing parser/checksum assumptions.
