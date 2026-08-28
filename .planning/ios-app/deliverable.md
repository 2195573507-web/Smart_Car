# Deliverable: Independent iOS SmartCar App

# Independent iOS SmartCar App

## Created

- `IOS-APP/Package.swift`: standalone SwiftPM executable package, iOS 17+ and
  macOS host-build support.
- `IOS-APP/project.yml`: XcodeGen specification for the `SmartCarIOS` iOS
  application target (`com.smartcar.iosapp`, iOS 17.0+, iPhone/iPad).
- `IOS-APP/SmartCarIOS.xcodeproj/`: generated standard Xcode project.
- `IOS-APP/Resources/Info.plist`: app metadata plus both Bluetooth usage
  descriptions.
- `IOS-APP/Sources/SmartCarIOS/Core/`: migrated App-BLE protocol/framing,
  CoreBluetooth lifecycle, RSSI/reconnect health, vehicle models, telemetry
  stores, ViewModel, calibration state, and STM32/S3 log parser/store.
- `IOS-APP/Sources/SmartCarIOS/UI/`: five-tab SwiftUI shell, adaptive control
  and SceneKit pose view, wheel target/actual panels, heading compass,
  telemetry/health screens, haptics, and log console export controls.

## Feature Mapping

- Control and pose are co-located in the first tab for iPhone/iPad layouts.
- `ALL WHEELS` and RR/RF/LR/LF sliders retain the macOS `-800...800 mm/s`
  range and `10 mm/s` step.
- Slider/heartbeat uses `0x15`; per-wheel nudge uses `0x2A`; master scale uses
  `0x2B`; zero reset and E-Stop send zero wheel targets and reset scale.
- Telemetry renders four target-vs-actual wheel values, voltage/battery,
  calibration progress, dual-AHRS deltas, chassis safety/odometry, and BLE
  health counters.
- BLE discovery is requested automatically on app launch and resumes after
  Bluetooth power-on through the existing reconnect/backoff lifecycle.

## Verification

```text
swift build --package-path IOS-APP
Build complete! (1.52s)
```

`plutil -p IOS-APP/Resources/Info.plist` confirms
`NSBluetoothAlwaysUsageDescription` and `NSBluetoothPeripheralUsageDescription`.
`xcodegen generate --spec project.yml` created the Xcode project, and the
following simulator build passed with Xcode 26.6:

```text
xcodebuild -project SmartCarIOS.xcodeproj -scheme SmartCarIOS \
  -sdk iphonesimulator -destination 'generic/platform=iOS Simulator' \
  CODE_SIGNING_ALLOWED=NO build
BUILD SUCCEEDED
```

No runtime BLE, RSSI/reconnect, sensor, or SceneKit-on-device acceptance is
claimed from a simulator build. The historical `IOS_APP/` source tree was
removed as requested; unrelated repository changes were preserved.
