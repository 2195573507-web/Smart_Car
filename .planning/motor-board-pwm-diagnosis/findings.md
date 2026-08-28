# Findings

## Confirmed

- `motor_board_update_pid()` loops over four indexes and uses the same index for
  target, feedback, PID state, trim, and PWM output.
- `MB_Protocol_SendPwm()` preserves its four arguments as `$pwm:m1,m2,m3,m4#`.
- The board guide defines physical channels as M1 LF, M2 LR, M3 RF, M4 RR.
- Chassis output is logical order RR, RF, LR, LF, so unchanged send order puts
  the high right target onto the two physical left channels.
- With `KV=1.05`, friction compensation 140 PWM, and a 170 mm/s steady target,
  the feedforward alone is `170*1.05+140=318.5` PWM. Ramp and error terms can
  make the first output larger; this is not enough evidence to retune yet.

## Verification Gap

The source/build cannot prove the physical harness or board channel labeling.
The next vehicle capture must correlate logical target, raw MSPD M1..M4,
logical feedback, PID PWM, and `tx_raw` M1..M4.

