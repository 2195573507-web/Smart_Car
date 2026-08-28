# Independent iOS SmartCar App Design

**Status:** Approved by the user on 2026-08-23 for implementation.

## Goal

Add a standalone SwiftUI application under `IOS-APP/` for iOS 17+ while
preserving the active App-BLE semantics and telemetry meaning. The historical
`IOS_APP/` source tree is removed after the new target is verified.

## Architecture

The package contains one executable target, `SmartCarIOS`, with explicit
`Core/` and `UI/` source folders. Keeping the migrated core in the same app
module preserves the macOS source visibility and avoids introducing a second
public protocol/telemetry API solely for SwiftPM target boundaries. The core
folders contain the Foundation/CoreBluetooth/Combine models, App-BLE parser
and encoder, BLE lifecycle, telemetry stores, command view model, and log
parser; the UI folder contains the SwiftUI executable and platform adapters.

Core source is migrated from the current macOS target by semantic copy. The
mobile shell does not introduce a second protocol or a second telemetry model.
The UI subscribes to `SmartCarViewModel` and `TelemetryStore` snapshots on the
main actor.

## UI and Data Flow

`SmartCarIOSApp` creates one `SmartCarViewModel`, which owns `BLEManager` and
`TelemetryStore`. `BLEManager` scans for `SmartCar_S3`, records RSSI and
connection state, writes App-BLE frames to FFE1, and feeds FFE2/FFE3 into the
protocol and log parsers. Parsed messages update telemetry stores and log
stores; SwiftUI observes those stores.

The `TabView` has five tabs:

1. `操控`: SceneKit pose view beside the macOS-style `ALL WHEELS` slider, four
   independent sliders, and the RR/RF/LR/LF target-vs-actual panel.
2. `四轮调速`: expanded wheel controls, master scale, per-wheel step buttons,
   zero reset, and E-Stop.
3. `姿态/遥测`: SceneKit pose, compass/target heading, battery/power, IMU and
   dual-AHRS delta, calibration progress, and chassis state.
4. `健康`: BLE health counters, packet loss/error counters, task/stack fields
   when supplied by telemetry, and stale/fault indicators.
5. `日志`: STM32/S3 source selector, INFO/WARN/ERROR minimum level, text
   search, pause/resume, clear, copy, and `ShareLink` export.

The control screen intentionally exposes speed only. No new turn or joystick
command is introduced; existing wheel and master-speed commands remain
available for feature parity and future control modes.

## Active Protocol Mappings

- `0x15`: all-wheel speed command, four little-endian Float32 values.
- `0x16`: actual four-wheel speed status.
- `0x1C`: power voltage.
- `0x27`: IMU telemetry.
- `0x28`: dual-IMU lifecycle/status.
- `0x29`: chassis state.
- `0x2A`: one-wheel speed command, wheel id plus Float32.
- `0x2B`: master speed scale Float32.
- `0x2C`: wheel control status with mode, scale, targets, and actual speeds.

Wheel order remains `M1 RR`, `M2 RF`, `M3 LR`, `M4 LF`; slider range and step
remain `-800...800 mm/s` and `10 mm/s` as in the macOS implementation.

## Platform Adaptation

- Replace AppKit clipboard/save-panel use with `UIPasteboard` and `ShareLink`.
- Add `UIImpactFeedbackGenerator` feedback for slider commits, step buttons,
  mode changes, and E-Stop.
- Use SceneKit for the iOS 3D view and update Euler angles from the primary
  AHRS data, while showing validity/stale state when telemetry expires.
- Include `Resources/Info.plist` with both `NSBluetoothAlwaysUsageDescription`
  and `NSBluetoothPeripheralUsageDescription`.

## Error Handling and Safety

The port preserves the prior control behavior: all disconnect paths transmit a zero wheel
frame before cancellation when the write path is available; invalid or stale
telemetry is surfaced as stale rather than rendered as valid; BLE errors remain
user-visible; E-Stop resets targets and sends zero commands. No firmware,
GPIO, or transport ownership changes are in scope; the `0x114`/`0x2D` protocol
extension is implemented on the STM32, S3, and iOS paths.

## Verification

- Host syntax/build: `swift build --package-path IOS-APP`.
- Static checks: Info.plist keys, package target paths, and deletion of the
  historical `IOS_APP/` tree.
- iOS generic compilation is attempted with Xcode when the full Xcode toolchain
  is available. A host SwiftPM build is not treated as device or BLE
  acceptance.
