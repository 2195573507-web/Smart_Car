# Task Plan: Four-Wheel Speed Closed Loop

## Goal

Implement the approved SCBP-CAN/App BLE wheel speed command, STM32 PID loop,
S3 bridge, radar coexistence, power telemetry, and macOS control card.

## Phases

- [x] Design and protocol decisions approved
- [x] Shared protocol and explicit wire helpers
  - Add SCBP IDs/lengths and shared `scbp_wire` float/u32 helpers.
  - Add host codec coverage for all new payload widths and finite-value rejection fixtures.
- [x] STM32 PID and MotorBoard/SCBP integration
  - Add `pid_controller.h/.c` to the active CM7 source/include lists.
  - Expose target/actual/power APIs from MotorBoard; replace the test loop with four PID instances.
  - Add 50 ms wheel status and 500 ms power streams; add 1000 ms command watchdog in `s3_service`.
- [x] ESP32-S3 BLE/SCBP bridge and radar remap
  - Remap App constants to 0x1A/0x1B and add 0x15/0x16/0x1C.
  - Forward wheel commands with SCBP ACK completion and relay wheel/power streams.
  - Keep radar PWM and periodic radar status paths independent.
- [x] macOS protocol/model/view integration
  - Add explicit f32 LE codec and new message cases/models/stores.
  - Add 50 ms wheel command coalescing, disconnect/background zeroing, BRAKE, and layout-B card.
  - Preserve radar controls on 0x1A/0x1B.
- [x] Host tests and target builds
  - Run SCBP C host test, Swift build/test, CM7 build, and ESP-IDF build.
  - Fix only task-scope failures; record unrelated baseline failures separately.
- [x] Final source/evidence audit
  - Run diff checks, inspect target-file diffs, and distinguish build evidence from hardware acceptance.

## Constraints

Preserve unrelated dirty changes. Do not change GPIO, IOC, UART ownership,
existing radar control APIs, or legacy message behavior except the explicitly
approved App type migration (`0x15/0x16` -> wheel messages and radar ->
`0x1A/0x1B`).

## Evidence Boundary

Builds and host tests are source evidence. Physical UART, BLE, motor, radar,
and vehicle acceptance are not claimed without captures and hardware runs.
