# Progress: Dynamic Yaw Error Suppression

## 2026-08-27

- Confirmed the user's local contracts remain authoritative: M1=RR, M2=RF,
  M3=LR, M4=LF; track width 193.0 mm; existing trims and safety gates stay
  unchanged.
- Implemented `$MSPD` timestamp-derived `dt` with monotonic DWT-backed time,
  2 ms to 100 ms bounds, baseline reset, and PID/ramp/feedback-LPF use.
- Implemented heading `w_prev` slew limiting at 5 rad/s^2, zero reset on gate,
  stop, inactive mode, and invalid attitude, plus zero-default `w_ff` API.
- Kept the local heading correction polarity and expressed `Kd` as negative
  damping for positive local body yaw rate.
- Added LPF alpha finite-value and [0,1] guards; protected the MSPD timestamp
  state with a FreeRTOS critical section.
- CM7 Debug target built successfully; hardware, flashed-image, UART, and
  vehicle behavior remain unverified.
