# Notes: SmartCar_Control_MAC Alignment

- Historical source is available in Git at `IOS_APP/SmartCar_Control_MAC`.
- Current iOS source contains the new App-BLE types `0x2A` through `0x2D`,
  chassis mode state, adjustable base speed, serial write queue, and motion
  heartbeat.
- The current macOS logger at `Tools/SmartCar_Logger_MAC` is unrelated and
  must remain separate.
- The user selected top-level restoration path `SmartCar_Control_MAC/` and
  requested original code first, then iOS additions.
- Historical baseline `swift build --package-path SmartCar_Control_MAC` passed
  before parity edits.
- Post-parity macOS build passed; core source files match the current iOS files
  except for the intentional AppKit lifecycle branch in the macOS ViewModel.
