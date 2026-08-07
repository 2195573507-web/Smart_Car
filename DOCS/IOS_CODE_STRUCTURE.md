# iOS App Code Structure

## Scope

This document records the initialized Smart Car iOS application scaffold. The
app is the operator control entry point for L1 remote control. It provides the
SwiftUI/MVVM boundaries for BLE discovery, protocol serialization, manual input,
emergency-stop intent, and status presentation. It does not claim vehicle
connectivity, command delivery, motor movement, or hardware safety behavior.

## Source Tree

```text
IOS_APP/
  Package.swift
  SmartCarApp/
    SmartCarApp.swift
    Core/
      Bluetooth/
        BLEManager.swift       CoreBluetooth adapter and transport facade
        BLEDevice.swift         Discovered-device value type
        BLEState.swift          BLE lifecycle state
      Protocol/
        Packet.swift            Frame encoding, decoding, CRC validation
        Command.swift           L1 command and control-intent types
        Message.swift           Intent-to-packet mapping
      Control/
        Joystick/
          JoystickViewModel.swift  Normalized joystick intent state
        DirectionPad/
          DirectionPadState.swift  Press/release state for future hold control
        EmergencyStop/
          EmergencyStopState.swift Emergency-stop UI state vocabulary
      Model/
        VehicleState.swift      Link, readiness, freshness, and fault state
      Service/
        VehicleTransport.swift  Transport abstraction consumed by the view model
      UI/
        RemoteView/
          RemoteView.swift          L1 control surface composition
          RemoteViewModel.swift     MainActor MVVM state and command dispatch
          JoystickView.swift        Bounded drag joystick with release stop
          DirectionPadView.swift    Direction-pad controls
        StatusView/
          StatusView.swift           BLE and vehicle status presentation
        SettingsView/
          SettingsView.swift         BLE service UUID preference field
        EmergencyStopView.swift      Emergency-stop action surface
  Docs/
    README.md                         App-local documentation boundary
```

## Dependency Direction

`RemoteView` depends on `RemoteViewModel`; the view model owns observable UI
state and consumes `VehicleTransport`. `BLEManager` implements that transport
and owns CoreBluetooth access when CoreBluetooth is available. Protocol models
are Foundation-only and are passed to the transport as typed `Packet` values.
SwiftUI views do not construct frames or access CoreBluetooth directly.

The intended data flows are:

```text
Joystick / DirectionPad -> RemoteViewModel -> Message / Packet
                       -> VehicleTransport -> BLEManager -> ESP32-S3

ESP32-S3 notifications -> BLEManager.incomingPackets -> model/state layer
                        -> SwiftUI status surfaces
```

The current scaffold has the incoming packet stream and status model types in
place, but does not yet wire decoded `STATUS` messages into `VehicleStatus`.

## Protocol Boundary

`Packet` follows the planned Smart Car envelope shape and performs local header,
length, version, and CRC checks. `Command` includes L1 stop, emergency stop,
manual pad, manual joystick, and heartbeat intents, while retaining an
`autonomyIntent` command identifier for future L3 capability negotiation. The
BLE service and characteristic UUIDs remain runtime configuration values; no
hardware-specific UUID is assumed.

## Current Limitations

- The joystick view maps a bounded drag to normalized linear/turn intent and
  sends a stop when the drag ends. Backgrounding, cancellation, and richer
  gesture arbitration still need dedicated lifecycle tests.
- Direction-pad and emergency-stop actions create protocol packets but are not
  proven against an ESP32-S3 endpoint.
- BLE discovery/connect/write behavior is a CoreBluetooth adapter scaffold.
- Automatic navigation, SLAM, radar visualization, ROS2, and AI-agent control
  are intentionally absent from this L1 target.
- `Package.swift` declares iOS 17 and macOS 14 targets so the source can be
  checked in environments without an iOS SDK; macOS fallback code does not
  establish iOS runtime acceptance.

## Validation Boundary

The source tree and package manifest are static implementation evidence.
`swift build` passes in the current workspace. A Swift test target was removed
because XCTest is unavailable in the current command-line environment, so no
automated test result is claimed here. Build success cannot prove BLE
interoperability, protocol compatibility with S3, command admission,
emergency-stop timing, STM32 output, or vehicle behavior. Those require
simulated transport tests, bench tests, and controlled hardware tests under the
project verification plan.
