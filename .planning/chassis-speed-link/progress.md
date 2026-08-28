# Progress: S3 Chassis Speed Link Repair

## 2026-08-26

- Completed source inspection and confirmed the missing SRP `0x06` dispatch.
- Confirmed the user-authorized track width of 193.0 mm.
- Implementation is beginning; no flashed-device or vehicle evidence exists.
- Added `chassis_kinematics` with `CHASSIS_TRACK_WIDTH_MM = 193.0f` and
  RR/RF/LR/LF mapping.
- Added the 10 ms `chassis_task`, SRP/IMU/attitude/MotorBoard admission gate,
  timeout/BUS_OFF stop-and-clear behavior, and diagnostics.
- Added SRP `0x06` decode/ACK/logging and MotorBoard nonzero PWM diagnostics.
- CM7 Clean Build passed: FLASH 17.89%, RAM 61.41%; host kinematics and SRP
  codec tests passed. Hardware/runtime/vehicle evidence remains pending.
