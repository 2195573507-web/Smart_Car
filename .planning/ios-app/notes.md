# Notes: Independent iOS SmartCar App

## Confirmed Existing Contracts

- Previous parity source: the historical macOS target
  `IOS_APP/SmartCar_Control_MAC`; it has been removed.
- Current App-BLE envelope is encoded by `SmartCarProtocol` with `0xAA 0x01`
  framing and Modbus CRC16; this is separate from the STM32-S3 SCBP-CAN UART
  frame.
- Active App message types include wheel speed `0x15`, wheel status `0x16`,
  power `0x1C`, PID `0x1D`, IMU telemetry `0x27`, dual IMU status `0x28`,
  chassis state `0x29`, single wheel `0x2A`, master speed `0x2B`, and wheel
  control status `0x2C`.
- Wheel order is M1/RR, M2/RF, M3/LR, M4/LF. Current macOS sliders use
  `-800...800` mm/s with 10 mm/s steps and a linked `ALL WHEELS` binding.
- Current macOS BLE UUIDs are service `FFE0`, write `FFE1`, notify `FFE2`,
  and logger `FFE3`; target device name is `SmartCar_S3`.
- Current telemetry store coalesces attitude, dual-AHRS, wheel, power, IMU,
  chassis, and calibration data on the main actor; wheel histories retain 48
  samples per wheel.
- Current log parser handles the separate `AA 55` log envelope and four levels
  (`DEBUG`, `INFO`, `WARN`, `ERROR`) from STM32/S3 sources.

## Existing iOS Prototype Risk

`IOS_APP/SmartCarApp` was a prior generic package using a different `Packet`,
`Command`, and joystick-oriented model. It is not the current parity source and
was removed with the historical `IOS_APP/` tree.

## Implementation Decisions

- Use a new SwiftPM package with a `SmartCarIOS` executable target and focused
  Core/UI source folders. The package is host-buildable with SwiftPM and
  Xcode-openable for iOS 17+; the existing source visibility keeps the
  migrated core semantics in one app module without duplicating public APIs.
- Copy current macOS semantic source into the new package where needed, then
  remove AppKit-only APIs. Core code stays Foundation/CoreBluetooth/Combine.
- Build the iOS shell with `TabView`, `SceneKit`, `ShareLink`, `UIPasteboard`,
  and `UIImpactFeedbackGenerator` guarded to iOS.
- Include `Resources/Info.plist` with both Bluetooth permission declarations.

## Validation Gaps

- This environment currently reports CommandLineTools as the active developer
  directory, so `xcodebuild` generic iOS compilation may be unavailable.
- BLE discovery, RSSI, reconnect behavior, sensor telemetry, and 3D motion
  remain hardware/runtime evidence, not build evidence.

## Implementation Evidence

- `0x15` is used for all-wheel slider updates and the non-zero wheel heartbeat.
- `0x2A` is used by the per-wheel +/- 10 mm/s nudge buttons.
- `0x2B` is used by the master speed scale slider.
- The control tab keeps SceneKit pose and speed controls in the same adaptive
  `ViewThatFits` layout; the pose card includes a heading compass and target.
- The log tab supports source/level filtering, regular-expression search,
  pause/resume, clear, filtered clipboard copy, and filtered `ShareLink` export.
- The iOS ViewModel requests scanning at launch; BLEManager defers it until
  CoreBluetooth is powered on and preserves reconnect backoff after drops.
