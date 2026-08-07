# SmartCar macOS TelemetryStore Architecture

## Scope

This change is limited to the macOS control app under
`IOS_APP/SmartCar_Control_MAC`. SmartCar AA/55 framing, BLE characteristics,
STM32 messages, S3 bridging, packet parsing, CRC handling, and command payloads
remain unchanged.

## Data flow

`BLEManager` owns CoreBluetooth callbacks, packet assembly, protocol parser
invocation, and the bounded decoded-message ring. Parsed messages are sent to
`TelemetryStore` for latest-value storage and to a small ViewModel callback for
debug counters. SwiftUI observes the relevant store independently.

`AttitudeState` publishes a coalesced snapshot at 20 Hz, `IMUState` at 5 Hz,
`VehicleStatusState` at 1 Hz, and `CalibrationState` immediately when a
calibration status or bias message arrives. No packet updates a SwiftUI-bound
log array.

## Log behavior

`decodedMessages` remains bounded to the existing maximum of 200 records. The
ViewModel exposes a separate `logSnapshot`; `refreshLogs()` copies the current
bounded array into that property. `DebugConsole` renders only this snapshot and
refreshes it from an explicit button.

## ViewModel behavior

`SmartCarViewModel` keeps control commands, page mode, connection/error state,
and 1 Hz debug metrics. The mode switch no longer starts a global timer or
sends `objectWillChange` manually. `CalibrationViewModel` subscribes directly
to `CalibrationState` and never scans decoded history.

## Verification

The package is verified with `swift build` from
`IOS_APP/SmartCar_Control_MAC`. This proves host compilation only; it does not
claim BLE hardware, STM32, S3, or runtime acceptance.
