# Deliverable: App Speed Command Dispatch Repair

Status: complete for source and host-build scope.

## Changes

- Both app BLE managers now split frames at the negotiated
  `maximumWriteValueLength(for: .withResponse)` and preserve complete-frame
  ordering through coalescing and write errors.
- The macOS control view no longer exposes the retired percentage speed slider;
  speed input remains in the current wheel/chassis control card.
- Straight-line speed now starts and resets at `0.0`; active chassis and wheel
  targets are dispatched every 50 ms using latest-value coalescing.
- Normal stop, emergency stop, joystick release, and lifecycle stop use a
  zero-wheel command. S3 serializes ACK-required motion commands with one
  in-flight transaction plus newest pending target/scale replacements; a zero
  stop bypasses pending slots and clears both replacements.
- Motion cancellation releases SCBP pending slots without invoking the old
  completion callback, preventing a superseded target from being requeued ahead
  of the zero stop.

## Verification

- `swift build --package-path IOS-APP`: passed.
- `swift build --package-path SmartCar_Control_MAC`: passed.
- Protocol check: both implementations produced 24-byte wheel/chassis frames
  and parsed a 20+4 split frame successfully.
- `git diff --check`: passed.
- `idf.py -B build-speed-command-fix build` in `ESPS3`: passed.
- `cmake --build build/Debug --target Smart_Car_H757_CM7 -j2` in `STM32H757/CM7`:
  passed with `ninja: no work to do`.
- SCBP-CAN host tests: passed.
- Final regression after `scbp_link_cancel_message()` was added: SCBP-CAN host
  tests, both Swift package builds, ESP-IDF S3 build, CM7 build, and
  `git diff --check` all passed.
- macOS bundle refreshed and verified with `script/build_and_run.sh --verify`;
  bundle timestamp: `2026-08-23 17:38:57`.

Live BLE, S3, STM32, UART, ACK, and motor behavior still require hardware
validation with matching firmware and app builds.
