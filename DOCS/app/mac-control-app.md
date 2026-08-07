# macOS Control App Module

## Function

Provide operator control, BLE connection management, telemetry presentation,
calibration/radar views, and a secondary developer/logging surface.

## Source Location

`IOS_APP/SmartCar_Control_MAC/Sources/SmartCar_Control_MAC/`

## Entry Files

- `main.swift`: SwiftUI application entry.
- `BLE/BLEManager.swift`: CoreBluetooth central, GATT discovery, receive
  pipelines, and outbound frame calls.
- `Model/SmartCarProtocol.swift`: App frame encoding/parsing.
- `ViewModels/SmartCarViewModel.swift`: UI-facing command/status state.
- `Stores/TelemetryStore.swift`: MainActor telemetry state.

## Inputs

BLE discovery/connection events, FFE1 writes/FFE2 notifications, FFE3 log
notifications, and user commands from control/developer views.

## Outputs

BLE writes for ping, control, speed, radar speed, and radar calibration query;
published SwiftUI state; bounded decoded-message and device-log stores.

## Public Interfaces

`BLEManager.startScanning/connectToDevice/disconnect`, `sendPing`,
`sendControl`, `sendSpeed`, `sendRadarSpeed`; `SmartCarProtocol.encode` and
`Parser.feed`; `TelemetryStore.ingest`.

## Dependencies

CoreBluetooth, SwiftUI/Combine, `SmartCarProtocol`, `BLEPacket` decoded models,
`TelemetryStore`, `DeviceLogStore`.

## Current Status

Source-established macOS UI and parser. BLE UUIDs and queue boundaries are
defined. App-to-device behavior is unverified; S3 callback registration and
telemetry relay are not proven in the current S3 source.

## Known Issues

- App frame layout uses a trailing `0x55`, unlike the current STM-S3 C frame.
- `BLEReceivePipeline` performs parsing on a serial queue and delivers UI state
  through MainActor callbacks; preserve that separation.
- A successful `swift build` does not refresh the staged app bundle; use the
  package launcher/verification workflow when UI runtime is explicitly in scope.

## Modification Notes

Keep BLE/protocol/firmware boundaries unchanged for UI-only work. Do not infer
physical stop behavior from `emergencyStop()` sending a STOP frame.
