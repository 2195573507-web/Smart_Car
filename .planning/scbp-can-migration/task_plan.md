# SCBP-CAN Migration Plan

## Goal

Replace the STM32H757 CM7 <-> ESP32-S3 UART SCBP-V3 path with the approved
SCBP-CAN protocol while preserving the independent App BLE envelope and the
user's in-progress IMU/calibration changes.

## Phases

- [x] Phase 1: Confirm the approved protocol contract and inspect repository state.
- [x] Phase 2: Map active STM32 and ESP32-S3 protocol/UART call sites and frozen payload contracts.
- [x] Phase 3: Implement and host-test the shared SCBP-CAN codec, parser, and link health manager.
- [x] Phase 4: Migrate STM32H757 service, runtime, logging, and UART2 transport.
- [x] Phase 5: Migrate ESP32-S3 protocol component, command bridge, calibration manager, and UART2 transport.
- [x] Phase 6: Replace current protocol documentation and run focused source/build verification.

## Evidence Boundary

Host tests and firmware builds demonstrate source integration only. UART signal
integrity, DMA behavior on target, BLE relaying, sensor data, radar/PWM control,
and vehicle behavior require separate device/integration validation.
