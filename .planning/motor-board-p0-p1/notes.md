# MotorBoard P0/P1 Findings

## Confirmed Current Behavior

- `main.c` initializes USART6/transport/protocol and queues zero PWM before RTOS tasks.
- Normal mode delays MotorBoard task creation until the attitude startup coordinator
  observes IMU/AHRS readiness and five advancing update checks.
- `motor_board_set_target_wheel_speeds()` validates finite/range values but does not
  require initialization sequence completion or a fresh MSPD stream.
- `motor_board_update_pid()` rejects invalid MSPD timing and resets PID history, but
  there is no separate runtime no-feedback watchdog.
- The current sequence writes mtype/mline/mphase/wdiameter on each task startup.
- The current protocol treats text containing `OK`, `ACK`, or `Set ` as a generic ACK and
  clears read_flash mode at that point.
- Battery cache has no age/freshness check.

## User Hardware Capture

```text
read_flash:OK!
Motor_Version:1.7.3
Motor_type:1
Dead_Zone:1600
Pulse_Line:11
Pulse_Phase:30
wheel_diameter:65.000
P:0.800    I:0.060    D:0.500
```

Required matching fields are `Motor_type`, `Dead_Zone`, `Pulse_Line`,
`Pulse_Phase`, and `wheel_diameter`. Board version and board-side P/I/D are
diagnostic only because CM7 owns the local speed loop.

## Evidence Limits

- Source and build can prove integration and parser behavior only.
- UART capture is required to prove command order, no-write behavior, feedback period,
  and the 200 ms timeout response.
- Vehicle acceptance remains separate from firmware build success.

## Implementation Notes

- `MB_Protocol_SendReadFlash()` resets a typed snapshot and keeps multiline
  capture active through `read_flash:OK!`; only the five required field bits
  form a complete snapshot.
- The task now reads first, skips matching fields, writes only observed
  mismatches, and performs a second readback after any write. A generic ACK is
  ignored for persistent configuration because it cannot be associated with a
  current command.
- Motion stays locked until two MSPD frames establish feedback. A valid-frame
  gap beyond 200 ms queues zero PWM, clears retained targets/PID/ramp, and
  restarts initialization. Battery cache freshness is two seconds.
- Host test command passed on 2026-08-28:
  `clang ... motor_board_protocol.c tests/test_motor_board_protocol.c -lm`.
- The canonical Debug CM7 build passed on 2026-08-28. No UART capture, flashing,
  or vehicle motion acceptance was run in this implementation pass.
