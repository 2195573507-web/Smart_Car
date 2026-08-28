# App-BLE V2 and Shared App Core Implementation Plan

## Goal

Implement the approved compatibility-first App-BLE V2 session layer, shared
Swift protocol/scheduling core, ESP32-S3 V2 admission bridge, app performance
improvements, and the canonical Bluetooth protocol documentation.

## Boundaries

- Preserve App-BLE V1 frame layout and command meanings for deployed clients.
- Preserve SCBP-CAN message IDs, STM32 motion authority, explicit zero stop,
  BUS_OFF recovery, and the STM32 command watchdog.
- Keep the iOS and macOS UIs platform-specific; share only Foundation-level
  protocol, parsing, session, and scheduling logic.
- Source/build verification does not establish BLE, UART, motor, or vehicle
  acceptance.

## Phases

- [x] Phase 1: Audit current App-BLE, S3 bridge, app state, and safety path
- [x] Phase 2: Freeze approved V2 compatibility design
- [x] Phase 3: Add shared Swift core and deterministic tests
- [x] Phase 4: Integrate V2 state/scheduler into macOS and iOS BLE managers
- [x] Phase 5: Add S3 V2 parser, admission, ACK mapping, and expiry stop
- [x] Phase 6: Publish canonical Bluetooth protocol document
- [x] Phase 7: Build/test/static-audit all changed target boundaries

## Acceptance Criteria

1. V2 HELLO, heartbeat, command ACK, session expiry, and duplicate handling
   are specified and implemented without changing V1 behavior.
2. Both apps use the same tested Swift protocol/scheduling module.
3. App outbound motion state is bounded and latest-state wins; zero/emergency
   commands preempt unsent nonzero motion commands.
4. S3 rejects invalid/expired/non-current-session V2 commands and maps valid
   commands to the existing SCBP-CAN transaction/stop paths.
5. Swift core tests and app builds pass; S3 build passes if the installed
   ESP-IDF environment is available.

## Evidence Log

- Implementation and source/build verification completed; see the evidence
  below for exact commands and the remaining hardware boundary.

## Completed Evidence

- `swift test --package-path Shared/SmartCarAppCore`: 8 tests passed.
- `swift build --package-path SmartCar_Control_MAC`: passed.
- `SmartCar_Control_MAC/script/build_and_run.sh --verify`: passed; refreshed
  `dist/SmartCar_Control_MAC.app` and confirmed the process started.
- `swift build --package-path IOS-APP`: passed.
- `idf.py -B build-app-ble-v2 build` with ESP-IDF 5.5.4: passed; app image
  linked and partition size check passed.
- Host SRP codec test compiled with `-Wall -Wextra -Werror` and passed.
- App parser C translation unit passed `-fsyntax-only -Wall -Wextra -Werror`.
- `git diff --check` passed for the changed App-BLE boundaries.

## Remaining Acceptance Gap

No BLE packet capture, matching flashed-image test, UART trace, STM32 ACK
trace, physical stop test, or vehicle acceptance was performed in this turn.
