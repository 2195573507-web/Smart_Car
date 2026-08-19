# Task Plan: Dual Attitude Log Presentation

## Goal

Add bounded macOS DualAttitude text logs and 1 Hz STM32 DualAHRS summary logs
without changing protocol bytes or the high-rate acquisition path. Audit the
active BMI323 raw-to-physical-unit conversion chain and report static build
evidence separately from runtime evidence.

## Phases

- [x] Explore current source, rules, active CMake paths, and dirty-file scope
- [x] Approve design and unit policy with the user
- [x] Audit active BMI323 range, conversion, and bias units
- [x] Implement App bounded log presentation
- [x] Implement STM32 1 Hz READY/TRACKING log
- [x] Run SwiftPM and CM7 static builds
- [x] Record final findings and evidence limits

## Change Boundary

Allowed source files:

- `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/Stores/TelemetryStore.swift`
- `IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/UI/DeveloperModeView.swift`
- `STM32H757/Application/RTOS/imu_runtime.c`

The active BMI323 conversion source is inspected only; no conversion change is
planned because the current source is internally consistent.
