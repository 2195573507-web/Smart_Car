# SmartCar_Control_MAC Restoration and iOS Parity Design

## Goal

Restore the original macOS control application as a standalone
`SmartCar_Control_MAC/` package, then add the control/protocol behavior already
present in `IOS-APP/`. Keep the existing macOS UI and package layout intact.

## Scope

- Restore only the historical `IOS_APP/SmartCar_Control_MAC` subtree into the
  new top-level `SmartCar_Control_MAC/` directory.
- Verify the restored baseline with the original macOS SwiftPM build.
- Port the iOS additions: App-BLE types `0x2A`, `0x2B`, `0x2C`, `0x2D`, exact
  payload validation, Float32 little-endian encoding/decoding, chassis-diff
  mode, adjustable base speed, mode feedback, serial `.withResponse` writes,
  motion coalescing/heartbeat, and disconnect/emergency-stop zeroing.
- Preserve macOS-specific Control/Developer/logging/calibration views and
  localization. Do not replace them with the iOS layout.

## Data Flow

macOS SwiftUI -> `SmartCarViewModel` -> macOS `BLEManager` -> App-BLE envelope
-> ESP32-S3 mapping -> SCBP-CAN. Telemetry follows the existing BLE notify path
back through `BLEManager` and `TelemetryStore`; `0x2C` updates the selected
control mode.

## Compatibility and Safety

- Preserve the existing App-BLE envelope and all historical command meanings.
- Preserve SCBP `0x110` as a 16-byte independent-wheel command; chassis diff
  uses App-BLE `0x2D` and SCBP `0x114`.
- Reject invalid command lengths/values before enqueueing.
- Keep one CoreBluetooth `.withResponse` write in flight and coalesce only
  superseded motion frames. Explicit zero and emergency stop remain ordered.
- No hardware BLE/UART/motor acceptance is inferred from host builds.

## Verification

1. Build restored macOS baseline before parity edits.
2. Build macOS after parity edits and run protocol/host tests where available.
3. Build `IOS-APP` to ensure the existing iOS target remains unchanged.
4. Run `git diff --check` and inspect the final file/status boundary.

