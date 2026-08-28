# Audit Notes

## Evidence Rules

- Current source is authoritative for implementation and parameters.
- Documentation and historical memory are used only as cross-checks.
- Build-only evidence is not presented as hardware behavior evidence.

## Current Source Findings

- `STM32H757/Application/Chassis/chassis_task.c` runs at 10 ms. Current heading
  gains are `0.28/0.085/0.006`; error is shortest-angle `current-target` in
  degrees; the three terms are proportional error, bounded integral in deg*s,
  and filtered/bias-corrected body gyro Z in rad/s. Correction is limited to
  `+/-2.0 rad/s` and remaining wheel headroom.
- `chassis_kinematics.c` uses `left=v-w*193/2`, `right=v+w*193/2`, with order
  `RR, RF, LR, LF`; current source maps this to `M1..M4` in the same order.
- Chassis linear ramp is `400 mm/s^2`. MotorBoard has a second per-wheel target
  ramp at `800 mm/s^2` default, evaluated with fixed `0.05 s` PID dt on MSPD
  feedback events. No angular-acceleration limiter was found.
- Wheel control config is `Kp=1.10`, `Ki=0.060`, `Kd=0.00`, I limit `700`, output
  limit `2500`, error deadband `6 mm/s`, target floor `30 mm/s`, feedforward
  `1.40 PWM/(mm/s)`, friction compensation `260 PWM` over `80 mm/s`, and trim
  `M1=1.08`, `M2..M4=1.00`. The implementation does not use `kd` in the PID
  calculation, so this layer is PI plus feedforward in practice.
- MotorBoard command output is event-driven by valid `$MSPD` frames. It sends
  `$pwm:m1,m2,m3,m4#` through a 512-byte TX ring and USART6 TXE interrupt;
  no DMA or blocking HAL transmit is used. `$spd` is only an unused protocol
  wrapper in the current source.
