# Findings: Chassis Differential Command

## Confirmed Source Facts

- Active STM dispatch is `STM32H757/Middleware/Communication/Services/s3_service.c`.
- Active S3 bridge is `ESPS3/components/smartcar_service/command_bridge.c`.
- Active macOS App command sender is `IOS_APP/SmartCar_Control_MAC/.../BLE/BLEManager.swift`.
- The requested `RobotCommandService.swift` and `ControlPanelView.swift` do not
  exist in the active target; `ControlModeView.swift` and
  `WheelSpeedControlCard.swift` are the corresponding UI owners.
- `0x110` currently calls `chassis_runtime_set_wheel_speed_command()` and
  enters `MODE_WHEEL_INDEPENDENT`.
- `chassis_runtime_apply_wheel_speed_command()` is a legacy alias that already
  enters `MODE_CHASSIS_DIFF`, but no base-speed/yaw-rate state exists yet.
- `chassis_heading_control_step()` already accepts `target_vx` and
  `target_wz_rad_s`; the runtime currently reconstructs these from four raw
  wheel targets in diff mode.
- App BLE and STM-S3 SCBP frames are distinct envelopes and must remain so.

## Design Decisions

- Store differential command fields in the chassis runtime and derive left/right
  raw targets using `base_speed +/- 0.5 * yaw_rate * track_width`.
- Keep `MODE_CHASSIS_DIFF` as the reported command mode when the IMU is invalid;
  bypass heading correction for that cycle and emit the exact warning requested.
- Use `scbp_wire_read_f32_le` / `scbp_wire_write_f32_le` and explicit length checks
  at both endpoints.
- Keep BLE disconnect stop behavior on existing `0x110` zero frame unless the
  protocol task explicitly requires a new chassis-stop transaction.

## Evidence Limits

No physical UART/BLE/motor/vehicle acceptance is available from source or host
builds alone.

## Implementation Evidence

- `Common/SCBP_CAN/include/scbp_protocol_defs.h` registers
  `SCBP_MSG_ID_CHASSIS_SPEED_CMD = 0x114` and the 16-byte payload size without
  changing the existing `0x110` size or field order.
- `STM32H757/Middleware/Communication/Services/s3_service.c` rejects `0x114`
  unless the payload is exactly 16 bytes, both f32 values are finite, and all
  eight reserved bytes are zero; accepted frames select `MODE_CHASSIS_DIFF` and
  apply the differential command.
- `STM32H757/Application/Chassis/chassis_runtime.c` stores base speed/yaw rate,
  derives right/left targets from track width, invokes heading control in diff
  mode when the IMU snapshot is valid, and emits the required warning while
  bypassing heading correction when it is invalid.
- `ESPS3/components/smartcar_service/command_bridge.c` validates App type
  `0x2D`, forwards the same 16-byte payload as SCBP `0x114`, and maps the STM ACK
  back to App ACK type `0x2D`.
- The active command/UI target is
  `IOS_APP/SmartCar_Control_MAC`; the requested `RobotCommandService.swift` and
  `ControlPanelView.swift` are absent. `BLEManager.swift`,
  `SmartCarViewModel.swift`, `WheelSpeedControlCard.swift`, and the telemetry
  model are the corresponding owners.
- `SmartCarViewModel.driveStraight()` invalidates the pending single-wheel
  command and both wheel timers before sending App BLE `0x2D`, so an earlier
  `0x15 -> 0x110` heartbeat cannot switch STM32 back to independent mode.

## Verification Evidence

- Host command: `cc -std=c11 -Wall -Wextra -Werror -pedantic .../test_scbp_can.c`
  followed by `/tmp/test_scbp_can`; result: `SCBP-CAN host tests passed`.
- macOS App command: `swift build` in `IOS_APP/SmartCar_Control_MAC`; result:
  `Build complete!`.
- CM7 clean build: `cmake --build build/Debug --clean-first --target
  Smart_Car_H757_CM7 -j2`; result previously recorded as 0 errors/warnings,
  FLASH 18.11%, RAM 45.82%.
- ESP-IDF build: `idf.py -B build-s3-bridge build`; result previously recorded
  as successful with ESP-IDF 5.5.4.
- `git diff --check`; result: no output.
