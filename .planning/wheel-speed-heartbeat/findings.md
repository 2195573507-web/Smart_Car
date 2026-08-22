# Findings

- `motor_board_protocol.c` already applies `{1,-1,1,1}` to parsed `MSPD`
  feedback.
- `motor_board_task.c` applies a second sign to feedback before PID and a sign
  to the output, which reverses M2 feedback normalization.
- `SmartCarViewModel` currently has only a one-shot 50 ms wheel command timer.
- `s3_service.c` already clears targets after 1000 ms without a valid wheel
  command; this task must keep that behavior unchanged.
- `WheelSpeedControlCard` already routes BRAKE to
  `emergencyWheelBrake()`.
- The prior implementation applied the encoder sign in the parser and again
  in the control task. The parser now preserves raw `MSPD`; the task owns the
  only encoder calibration.
- Timeout stop now resets all four PID states, latches motion off, and queues
  `$pwm:0,0,0,0#`; a nonzero command releases the latch.
- App command and heartbeat timers use the common RunLoop mode so UI tracking
  does not pause the 100 ms watchdog feed.
