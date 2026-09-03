# iPhone App Control Repair

## Goal

Restore the deleted `IOS-APP` iPhone target, apply the already-approved App BLE
control reliability repair there, and remove the same repair mistakenly made in
the retired `IOS_APP/SmartCarApp` target.

## Scope

- Include: `IOS-APP`, exact historical snapshot `bde0fc6`, iPhone App BLE
  control path, Xcode project integrity, host build/tests.
- Exclude: `IOS_APP/SmartCar_Control_MAC`, S3, STM32, SRP, MotorBoard, and
  physical vehicle acceptance claims.

## Phases

- [completed] Restore the tracked iPhone project snapshot and its required shared package, then inspect the current control path.
- [completed] Apply the minimal GATT readiness, ordered-write, and stop-safety repair in `IOS-APP`.
- [completed] Roll back only the mistakenly changed retired `IOS_APP/SmartCarApp` files.
- [completed] Run SwiftPM and Xcode-project validation, then report source/build versus device evidence.
- [completed] Add and verify the iPhone circular joystick control, preserving the approved App BLE V1 four-wheel command and safe-stop behavior.
- [completed] Diagnose the current iPhone command no-response from App write through S3 command admission, then remove the iPhone log feature and its avoidable receive/UI work without weakening control safety. Source/build evidence is complete; physical iPhone/S3/vehicle acceptance remains blocked by unavailable devices.

## Risks

- The worktree is dirty outside this scope. Never reset, clean, stash, or touch those files.
- Host validation cannot prove BLE delivery, S3 admission, UART transport, or vehicle stopping.
- The supplied device logs establish a healthy STM32-S3 link with zero wheel targets, but contain no contemporaneous App FFE1 write, S3 command-parser, or App command-ACK evidence.

## Errors

| Error | Resolution |
| --- | --- |
| `xcodebuild` looked for `IOS-APP/IOS-APP/SmartCarIOS.xcodeproj` | Run from `IOS-APP` with project path `SmartCarIOS.xcodeproj`. |
| SwiftPM could not access `../Shared/SmartCarAppCore/Package.swift` | Restore the required shared package from the same `bde0fc6` snapshot. |
| Shared-package restore command had an unterminated shell quote | No files were changed; rerun the restore without the malformed pipeline. |
| First joystick follow-up patch used the view-model hunk against the UI file | No source was changed because `apply_patch` is atomic; split the two file edits. |
