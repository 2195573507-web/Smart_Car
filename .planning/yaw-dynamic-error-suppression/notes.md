# Research Notes: Dynamic Yaw Error Suppression

## Evidence Labels

- **Current source**: directly inspected in the active checkout.
- **User-provided**: supplied in the task statement; not yet confirmed in source.
- **Upstream source**: linked to a specific GitHub revision/path.
- **Proposal**: recommended design, not an implemented behavior.

## Initial Current-Source Facts

- `STM32H757/Application/Chassis/chassis_kinematics.h` defines a four-wheel
  order of RR, RF, LR, LF and `CHASSIS_TRACK_WIDTH_MM = 193.0f`.
- The repository README assigns final low-level motion authority to STM32H757
  and requires all CM7 builds to use `STM32H757/CM7/build/Debug`.
- The checkout is dirty, including active chassis, motor-board, and DualAHRS
  files. This task will add documents only.

## Open Questions to Resolve from Current Source

- Exact heading-controller gains, period, saturation, and angle convention.
- MotorBoard `$MSPD` receive timestamp and wheel PI update cadence.
- Whether wheel-speed PI uses a fixed 50 ms interval in active source.
- Available IMU gyro-Z filter timing, validity flags, and odometry data shape.
- Existing safety reset paths that a future controller state must obey.

## Resolved Current-Source Facts

- `chassis_task.c` defines a 10 ms task period and uses `CHASSIS_TASK_PERIOD_S`
  for heading integration. It has output gates for S3 sync, IMU boot readiness,
  attitude readiness, and MotorBoard task existence.
- The active heading gains are `0.28`, `0.085`, and `0.006`; heading error is
  `current_yaw - target_yaw`, with positive correction mapped by kinematics to
  faster RR/RF and slower LR/LF.
- `motor_board_task.c` invokes wheel ramp and PI only after an `MB_FRAME_MSPD`
  frame is polled, but both operations use the fixed `MB_PID_DT_SECONDS=0.05f`.
- `pid_controller.c` already includes dynamic-API-shaped `dt_seconds` inputs,
  conditional integration, feedback LPF, linear feedforward, smooth Coulomb
  compensation, and output saturation. The missing part is sourcing a real
  feedback interval at the MotorBoard call site.
- DualAHRS filters BMI323 gyro samples and exposes a validity-gated heading and
  filtered body-Z rate. The current odometry README is a boundary placeholder;
  no active wheel-odometry estimator was found in that module.

## Upstream Verification

- ROS 2 `diff_drive_controller` revision
  `94e74de35f9d04f313aca8f29df66c3a76004aa7` exposes `SpeedLimiter` with
  velocity/acceleration/jerk limits and passes `period.seconds()`; its
  controller also updates odometry from wheel feedback.
- PX4 Autopilot revision `64cbe71af74ddf87b4209c1aedba587a3f345c43` contains
  the `src/modules/rover_differential` directory with separate differential
  speed/rate control modules. Exact parameters are version-dependent.
- `robot_localization` revision `7dfb6aa97b2082185d2fac3420888ae8474bfc1a`
  contains EKF prediction/correction using measured durations, angle wrapping,
  covariance checks, and Mahalanobis gating.

## User-Confirmed Boundary

- The user explicitly confirmed that the local low-level configuration,
  wheel mapping, encoder direction, and related polarity are correct.
- Do not let PX4/ROS/official documentation override that contract or frame it
  as the cause of the yaw-drift issue. Future work should target dynamic
  actuator asymmetry, timing mismatch, filtering phase, slew limits, and
  feedforward/synchronization behavior.
