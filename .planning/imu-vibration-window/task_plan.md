# IMU Vibration Window and Handshake Plan

## Goal

Implement the requested 10-second vibration quality gate, state-machine timing
budgets, progress mappings, and S3-to-STM32 automatic PWM-level handoff.

## Progress

- [x] Read repository rules, architecture, calibration documentation, and prior context.
- [x] Audit current constants, binary payload builders, and CAL_EVENT/PWM_READY paths.
- [x] Apply scoped STM32 and S3 changes without changing packet layouts.
- [x] Build CM7 and S3 without flashing hardware.
- [x] Record diff and verification boundary.

## Verification

- `git diff --check`: PASS.
- CM7 `cmake --preset Debug && cmake --build build/Debug --target Smart_Car_H757_CM7 -j2`: PASS.
- S3 `source "$IDF_PATH/export.sh" && idf.py build` with ESP-IDF 5.5.4: PASS.
- No flash, monitor, or device runtime operation performed.
