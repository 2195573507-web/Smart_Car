# LSM303 Boot Flow Repair

## Goal

Make the STM32H757 LSM303 path deterministic from I2C-ready startup through
calibration, filtered publication, and diagnostics, while preserving the HAL,
USART logger, FreeRTOS architecture, and BMI323 boundary.

## Phases

- [x] Baseline the LSM303 driver, manager, calibration, filter, runtime, and CMake wiring
- [x] Repair initialization diagnostics and publication gating
- [x] Redesign the non-blocking calibration state machine and logging
- [x] Add synchronization and periodic status diagnostics
- [x] Configure and build the CM7 target
- [x] Record source/build evidence and remaining hardware-unverified claims

## Evidence boundary

Source inspection and a clean CM7 build can prove API integration and compile
coverage. They cannot prove live I2C identity, sensor output, timestamp cadence,
or long-duration board stability without hardware logs.
