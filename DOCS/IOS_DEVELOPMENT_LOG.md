# iOS Development Log

The early `IOS_APP/` scaffold entries below are historical. That source tree
was removed on 2026-08-23 after the replacement iOS target under `IOS-APP/`
was verified; current implementation evidence is tracked in
`.planning/ios-app/deliverable.md`.

## 2026-07-30: L1 App Architecture Scaffold

### Delivered

- Initialized `IOS_APP/Package.swift` as a Swift Package with SwiftUI source
  under `IOS_APP/SmartCarApp/`.
- Added CoreBluetooth transport types: `BLEManager`, `BLEDevice`, and
  `BLEState`.
- Added protocol types for the planned APP-to-S3 envelope: `Packet`, CRC16
  validation, command kinds, control intents, and intent-to-message mapping.
- Added a transport abstraction so the view model is not coupled to BLE APIs.
- Added vehicle link/status models with readiness and freshness predicates.
- Added SwiftUI surfaces for remote control, direction pad, bounded drag
  joystick with release stop, status, settings, and emergency stop.
- Added a MainActor `RemoteViewModel` that owns session/sequence values and
  dispatches typed control intents through the transport.

### Explicit Non-Claims

This entry records source initialization only. No real ESP32-S3 device was
paired, no BLE UUID contract was supplied, no command was accepted by a
gateway, and no motor, emergency-stop, telemetry, or vehicle behavior was
observed. The app does not implement automatic navigation, SLAM, radar display,
ROS2, or AI-agent control.

### Verification Notes

- The implementation tree was inspected under `IOS_APP/`.
- `swift build` passes for the package in the current workspace.
- The Swift test target was removed because XCTest is unavailable in the
  current command-line environment; no automated test result is claimed.
- Package/build results must be reported separately from iOS runtime
  acceptance; they do not establish BLE, S3, or vehicle behavior.
- Protocol field meanings and BLE characteristic details remain governed by
  `DOCS/SMART_CAR_PROTOCOL.md` and the ESP32-S3 implementation contract.

### Next Engineering Steps

1. Add unit tests for packet round-trips, malformed frames, CRC failures, and
   command payload bounds.
2. Add a deterministic test transport and wire incoming `STATUS` packets into
   `VehicleStatus` with stale/unknown handling.
3. Add lifecycle and UI tests for bounded joystick release, cancellation,
   backgrounding, and mode changes, once an XCTest-capable test environment is
   available.
4. Freeze the approved S3 BLE service/characteristic, pairing, MTU, heartbeat,
   acknowledgement, and timeout contract before hardware testing.
5. Validate the iOS target in Xcode, then run simulated transport, bench, and
   controlled vehicle tests independently.
