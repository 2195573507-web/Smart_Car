# PID Online Tuning Findings

## Current Facts

- `Common/SCBP_CAN/include/scbp_protocol_defs.h` currently defines `0x110`
  wheel speed command and has no `0x111` PID command.
- `Common/SCBP_CAN/include/scbp_wire.h` and `scbp_wire.c` already provide
  explicit binary32 little-endian scalar and array helpers with finite-value
  rejection on array decode.
- `STM32H757/Middleware/MotorBoard/motor_board_task.c` owns static arrays
  `s_wheel_pid[4]` and `s_wheel_ramp[4]`; its public header exposes target,
  stop, actual-speed, and battery APIs but no tuning API.
- `STM32H757/CM7/Core/Inc/pid_controller.h` currently uses the lowercase
  `pid_controller_t` type and has no `PID_Update_Gains` or
  `Ramp_Update_Max_Accel` declarations.
- `STM32H757/Middleware/Communication/Services/s3_service.c` validates and
  handles `WHEEL_SPEED_CMD`, then responds with the existing SCBP fast ACK.
- `ESPS3/components/smartcar_service/command_bridge.c` parses App BLE frames,
  sends `WHEEL_SPEED_CMD` with an ACK callback, and returns App type `0x06`
  acknowledgements.
- `IOS_APP/SmartCar_Control_MAC` has App BLE type `0x15` for wheel commands,
  `BLEManager.sendWheelSpeeds`, `SmartCarViewModel.wheelTargets`, and a
  `WheelSpeedControlCard` with one linked slider, four per-wheel sliders, and
  four short sparkline plots.
- `TelemetryStore.WheelSpeedState` retains 48 samples per wheel and receives
  actual speeds in M1=RR, M2=RF, M3=LR, M4=LF order.
- Default wheel parameters are `Kp=1.10`, `Ki=0.06`, `Kd=0.00`, and
  `WHEEL_RAMP_MAX_ACCEL=800.0f`; target speed firmware limit is 1000 mm/s.

## Required Contract

SCBP PID payload offsets are:

```text
0..3   kp        f32 LE
4..7   ki        f32 LE
8..11  kd        f32 LE
12..15 max_accel f32 LE, mm/s^2
```

The App envelope remains separate:

```text
AA | 01 | 1D | 10 00 | payload[16] | CRC16-MODBUS-LE | 55
```

The S3 validates the App payload, creates a new SCBP-CAN transaction with
source S3, destination STM, message `0x111`, and `ACK_REQUIRED`, then maps
the result to the existing App ACK payload `{type, result}`.

## Validation Ranges

The UI and firmware use the same admission ranges:

| Parameter | Range | Step | Default |
| --- | ---: | ---: | ---: |
| Kp | 0.0..4.0 | 0.05 | 1.10 |
| Ki | 0.0..0.3 | 0.005 | 0.06 |
| Kd | 0.0..0.1 | 0.002 | 0.00 |
| Accel | 200..2000 mm/s^2 | 50 | 800 |

The firmware rejects non-finite values and values outside these ranges before
changing any wheel. A failed validation leaves all four wheels unchanged.
