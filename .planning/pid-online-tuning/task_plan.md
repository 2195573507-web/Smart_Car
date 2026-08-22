# PID Online Tuning Task Plan

## Goal

Add runtime PID and acceleration tuning across the App BLE envelope, ESP32-S3
SCBP-CAN gateway, STM32H757 CM7 motor controller, and macOS four-wheel speed UI.

## Phases

- [x] Phase 1: Inspect current architecture, dirty worktree, protocol, and UI
- [x] Phase 2: Agree on visual interaction and ownership design
- [x] Phase 3: Write and review the design specification
- [x] Phase 4: Implement shared protocol and host tests
- [x] Phase 5: Implement CM7 runtime update ownership and SCBP dispatch
- [x] Phase 6: Implement S3 App-to-SCBP bridge
- [x] Phase 7: Implement App controls and four-lane chart
- [x] Phase 8: Run host, CM7, S3, and Swift verification

## Confirmed Decisions

- SCBP-CAN PID command ID is `0x111`, S3 to STM, ACK required, 16 bytes.
- App BLE PID command type is `0x1D`, with four float32 little-endian values.
- PID values are global and update all four wheels together.
- A master wheel-target slider sets all four targets; four individual sliders
  remain available for later per-wheel offsets.
- Wheel telemetry is shown as four synchronized lanes with fixed M1/M2/M3/M4
  colors and target-speed dashed baselines.
- PID values use numeric input plus stepper controls; Apply is the only write.
- MotorBoard retains ownership of the PID/Ramp arrays behind a bounded update API.

## Verification Boundary

Host tests and clean builds are source/build evidence. UART delivery, BLE
delivery, flashed runtime updates, motor response, and vehicle safety remain
unverified until matching hardware captures are available.

## Verification Evidence (2026-08-21)

- `cc -std=c11 -Wall -Wextra -Werror -pedantic ... && /tmp/test_scbp_can`:
  `SCBP-CAN host tests passed`
- `cmake --build --preset Debug --clean-first -j2` from `STM32H757/CM7`:
  CM7 ELF linked with 0 errors/warnings; FLASH 9.58%, RAM 34.18%.
- `ninja -C ESPS3/build-s3-bridge -j2`: gateway image and bootloader size
  checks passed; app image remains within its partition.
- `swift build` from `IOS_APP/SmartCar_Control_MAC`: executable linked with
  0 errors/warnings.
- `git diff --check`: passed.
