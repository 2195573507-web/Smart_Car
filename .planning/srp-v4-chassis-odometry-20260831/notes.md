# Findings: SRP v4 Chassis Odometry

## Confirmed Current Facts

- `Common/SRP/include/srp_registry.h` already declares message `0x15`, schema
  1, the four flag bits, the exact 24-byte payload, and the packed host view.
- `s3_service_send_chassis_state()` exists but has no producer.
- MotorBoard corrects RF feedback polarity once and stores actual MSPD values
  in `[RR, RF, LR, LF]` order, but its public getter has no timestamp or
  atomic four-value snapshot.
- MotorBoard already emits legacy 16-byte wheel status every 50 ms. That
  payload has no source freshness and must not be used as ROS odometry.
- The current `chassis_task` has no startup caller. Starting it for telemetry
  would also activate a control path and is outside this task.
- The attitude coordinator starts MotorBoard only after stable IMU/AHRS
  readiness and continuously revokes `g_attitude_is_ready` on freshness loss.
- S3 validates `CHASSIS_STATE` for BLE display but excludes it from the TCP
  telemetry sink and bounded queue.
- The S3 telemetry queue uses a wheel FIFO plus latest-only attitude/IMU
  observations with bounded fairness.

## Adopted Design

- Add a timestamped, sequenced MotorBoard actual-speed snapshot.
- Add a pure `Middleware/Odometry` integrator.
- Add a separate low-priority 50 ms chassis-state task; do not start the
  dormant chassis control task.
- Preserve pose across stale gaps but clear integration history and
  `ODOMETRY_VALID`; never integrate a missing interval on recovery.
- Add chassis state as another latest-only S3 observation.

## Hardware Gates

- MSPD scale and sign require straight/reverse/turn vehicle checks.
- Primary-yaw coordinate convention and map rotation require physical checks.
- Matching CM7/S3 images and a captured SRP/S3RD `0x15` frame are required.

