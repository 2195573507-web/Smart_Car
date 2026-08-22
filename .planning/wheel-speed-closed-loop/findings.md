# Findings: Four-Wheel Speed Closed Loop

- Existing SCBP-CAN frame and link APIs are shared by CM7 and S3.
- Existing App BLE parser uses `AA 01 TYPE LEN PAYLOAD CRC16-MODBUS 55`.
- Existing `TYPE=0x15` radar status and `TYPE=0x14` radar PWM will be remapped
  to the approved `0x1A` and `0x1B` values.
- CM7 already starts `s3_service` and `motor_board_task`; MotorBoard currently
  contains a 10-second open-loop test loop that must be removed.
- The existing MotorBoard parser exposes four `$MSPD` float speeds and battery
  voltage. Current S3 service has no wheel command branch.
- Existing macOS App is SwiftUI, not Flutter; implementation target is the
  macOS SwiftPM package.

## Verification

- SCBP host test: `SCBP-CAN host tests passed`.
- CM7: `cmake --preset Debug` and `cmake --build build/Debug --target Smart_Car_H757_CM7 -j2` passed.
- ESP32-S3: `idf.py -B build-wheel-speed build` passed with ESP-IDF 5.5.4.
- macOS Swift package: `swift build` passed.
- `git diff --check` passed.
- Physical UART, BLE, MotorBoard, radar, and vehicle behavior remain unverified.
