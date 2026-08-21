# Four-Wheel Speed Closed-Loop Design

## Scope

Implement the approved minimal extension across the existing SCBP-CAN,
STM32H757 MotorBoard, ESP32-S3 gateway, and macOS SwiftUI control app. The
existing App BLE envelope remains `AA 01 TYPE LEN PAYLOAD CRC16-MODBUS 55`.
The existing SCBP-CAN envelope remains unchanged.

## Message Contracts

| Direction | Message | ID/type | Payload |
| --- | --- | --- | --- |
| S3 -> STM | wheel command | SCBP `0x110`, ACK_REQUIRED | 4 x float32 LE, 16 B |
| STM -> S3 | wheel status | SCBP `0x210`, STREAM_DATA; App `0x16` | 4 x float32 LE, 16 B |
| STM -> S3 | power status | SCBP `0x209`, STREAM_DATA; App `0x1C` | float32 LE, 4 B |
| App -> S3 | wheel command | BLE `0x15` | 4 x float32 LE, 16 B |
| S3 -> App | radar status | BLE `0x1A` | online u8, speed percent u8 |
| App -> S3 | radar PWM | BLE `0x1B` | speed percent u8 |

All new multi-byte fields use explicit byte-wise little-endian helpers. C
packed structures and Swift `MemoryLayout` are not used for these payloads.

## Control Flow

The macOS App keeps four target values and receives four actual values. Slider
changes are coalesced to at most one BLE command every 50 ms. Disconnect,
scene/background termination, and BRAKE send an all-zero wheel command. S3
validates the BLE frame and forwards the payload as an ACK-required SCBP
command. STM validates length and finite float values, updates targets, and
returns an SCBP OK response. The STM MotorBoard task applies one position PID
per wheel whenever a valid `$MSPD` frame arrives, with M1=RR(+1), M2=RF(-1),
M3=LR(+1), M4=LF(+1), then sends `$pwm:m1,m2,m3,m4#`.

The STM command watchdog clears targets after 1000 ms without a valid wheel
command. The MotorBoard task streams calibrated actual wheel speed every 50 ms,
and the latest finite battery voltage every 500 ms.

## PID Behavior

The controller is position-form PID with configurable `Kp`, `Ki`, `Kd`,
`max_out`, `max_iout`, and deadband. Integral output is clamped. When the
unclamped output would saturate, integration is inhibited in the direction of
further saturation; the integral is allowed to unwind in the opposing
direction. A zero target resets the integral smoothly and produces a bounded
output toward zero.

## Failure Handling

- Invalid BLE lengths, non-finite floats, and out-of-range radar PWM receive a
  BLE ACK rejection and do not reach hardware.
- SCBP invalid length or non-finite wheel payload receives
  `SCBP_FAST_RESP_INVALID_PARAM`.
- SCBP transaction failure is reported to the App through the existing ACK type.
- Missing wheel commands trigger STM target zeroing and zero PWM.
- Missing wheel telemetry is shown as stale in the App; it is never treated as
  a healthy zero-speed report.

## Verification

Run the Common SCBP host suite, a strict Swift package build/test, a CM7
clean-ish build in an isolated build directory, and an ESP-IDF build in the
existing project. These prove source and codec integration only. UART/BLE
captures, motor response, PID tuning, emergency-stop timing, and vehicle safety
remain hardware acceptance work.
