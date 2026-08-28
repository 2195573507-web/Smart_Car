# CM7 Business Flow Stall Recovery

## Objective

Keep the CM7 SRP service, attitude gate, and chassis scheduler alive after an
IMU initialization or task-admission failure. Preserve zero-PWM safety until
the existing attitude lifecycle proves readiness, while keeping periodic SRP
status/state output available for S3 diagnostics.

## Phases

- [x] Confirm the current startup condition and protected safety boundaries.
- [x] Make attitude/chassis task admission independent from IMU runtime OK.
- [x] Add bounded startup failure and heap diagnostics.
- [x] Build CM7 and run scoped static checks.
- [x] Record build/test evidence and hardware/BLE acceptance criteria
  separately.

## Boundaries

- Preserve SRP v4 bytes, UART2 PA2/PA3 routing, BLE UUIDs, and motor safety
  gates.
- Do not claim live UART, BLE, sensor, or vehicle behavior from a build.

## Verification Evidence

- CM7 clean build: FLASH 199,212 B (19.00%), RAM 64,832 B (49.46%), RAM_D2
  512 B.
- ESP-IDF 5.5.4 isolated build: `build-codex-business-flow-20260825`, app
  partition free 90%, total image size 730,313 B.
- Host protocol test: `SRP_CODEC_TEST_PASS`.
- Static check: `git diff --check`.
- Pending bench evidence: flashed-image identity, STM UART RX/parser/TX
  counters, S3 `ESTABLISHED/READY`, BLE notification receipt, IMU lifecycle,
  and zero-PWM/vehicle behavior.
