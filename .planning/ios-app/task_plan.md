# Task Plan: Independent iOS SmartCar App

## Goal

Create a standalone iOS 17+ SwiftUI application under `IOS-APP/` with the
approved control/protocol parity for BLE, SCBP App-BLE framing, wheel speed
control, telemetry, health monitoring, attitude visualization, and logs.

## Scope Boundary

- Add files only under `IOS-APP/` for the deliverable.
- Remove the historical `IOS_APP/` directory as explicitly requested and
  preserve all unrelated dirty changes.
- Keep current command IDs and payload mappings; do not redesign firmware or
  the App-BLE contract.
- The approved control screen uses speed sliders only; the underlying wheel
  independent and master-speed commands remain available for parity.

## Phases

- [x] Phase 1: Explore current repository, docs, protocol, and macOS target.
- [x] Phase 2: Approve visual and architecture design.
- [x] Phase 3: Record implementation plan and source facts.
- [x] Phase 4: Create standalone SwiftPM package and migrate core modules.
- [x] Phase 5: Implement iOS TabView and adaptive control/telemetry UI.
- [x] Phase 6: Implement logs, haptics, permissions, and mobile export.
- [x] Phase 7: Build, inspect diffs, and report evidence boundaries.

## Verification

1. `swift build --package-path IOS-APP`
2. Confirm `IOS-APP/Resources/Info.plist` contains both Bluetooth usage keys.
3. Confirm the historical `IOS_APP/` tree is deleted and no directory remains
   at that path.
4. If the local developer directory lacks full Xcode, report iOS device/generic
   compilation as unverified rather than treating the host build as device
   acceptance.

## Status

Complete for the source/build scope. The package builds successfully with
SwiftPM on the host toolchain; device BLE, sensor, and generic iOS acceptance
remain runtime/Xcode validation items.
