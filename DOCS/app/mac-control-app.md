# SmartCar Control Apps

## Function

Provide operator control, BLE connection management, telemetry presentation,
calibration/radar views, and a secondary developer/logging surface on iOS and
macOS. Both targets use the same App-BLE command behavior and telemetry model.

## Source Location

- iOS: `IOS-APP/Sources/SmartCarIOS/`
- macOS: `SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/`

## Entry Files

- `SmartCarIOSApp.swift` / `main.swift`: platform SwiftUI application entries.
- `Core/BLE/BLEManager.swift` / `BLE/BLEManager.swift`: CoreBluetooth central, GATT discovery, receive
  pipelines, and outbound frame calls.
- `Core/Model/SmartCarProtocol.swift` / `Model/SmartCarProtocol.swift`: App frame encoding/parsing.
- `Core/ViewModels/SmartCarViewModel.swift` / `ViewModels/SmartCarViewModel.swift`: UI-facing command/status state.
- `Core/Stores/TelemetryStore.swift` / `Stores/TelemetryStore.swift`: MainActor telemetry state.

## Inputs

BLE discovery/connection events, FFE1 writes/FFE2 notifications, FFE3 log
notifications, and user commands from control/developer views. Motion commands
include independent-wheel `0x15`/`0x2A`, MasterScale `0x2B`, and chassis-diff
`0x2D` with adjustable base speed.

## Outputs

BLE writes for ping, control, speed, radar speed, independent-wheel commands,
MasterScale, chassis speed, and emergency zeroing; published SwiftUI state;
bounded decoded-message and device-log stores.

## Public Interfaces

`BLEManager.startScanning/connectToDevice/disconnect`, `sendPing`,
`sendControl`, `sendSpeed`, `sendRadarSpeed`, `sendWheelSpeeds`,
`sendSingleWheelSpeed`, `sendMasterSpeedScale`, and `sendChassisSpeed`;
`SmartCarProtocol.encode` and `Parser.feed`; `TelemetryStore.ingest`.

## Dependencies

CoreBluetooth, SwiftUI/Combine, `SmartCarProtocol`, `BLEPacket` decoded models,
`TelemetryStore`, `DeviceLogStore`.

## Current Status

Source-established iOS and macOS UI/parser implementations. BLE UUIDs and
queue boundaries are defined. App-to-device behavior is unverified; host and
simulator builds do not prove BLE delivery or vehicle behavior.

## Known Issues

- App frame layout uses a trailing `0x55`, unlike the current STM-S3 C frame.
- `BLEReceivePipeline` performs parsing on a serial queue and delivers UI state
  through MainActor callbacks; preserve that separation.
- A successful `swift build` does not refresh the staged app bundle; use the
  package launcher/verification workflow when UI runtime is explicitly in scope.

## Modification Notes

Keep BLE/protocol/firmware boundaries unchanged for UI-only work. Do not infer
physical stop behavior from `emergencyStop()` sending a STOP frame.
