# Development Log

## 2026-07-30 - Architecture scaffold

- Added BSP templates for GPIO, UART, SPI, I2C, PWM, TIMER, and ADC.
- Added project-owned IMU, motor, and encoder driver templates.
- Added S3 link command, packet, protocol, and state interface placeholders.
- Added Middleware, Application, System, and Config module READMEs.
- Preserved existing HAL, CubeMX-generated files, IOC, CMake, CM4, and CM7
  sources.
- No lidar code, protocol implementation, flashing, or runtime hardware test.

Validation is limited to static structure, C syntax checks for new templates,
and the existing CM4/CM7 build baseline.
