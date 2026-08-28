# Findings

## Confirmed Source Facts

- `imu_boot_manager.c` enters `IMU_PHASE_STATIC_CALIBRATION` only after both
  self-test streams are observed, then waits for zero-PWM sync and a static
  window. Raw-precheck and static-motion failures now share the bounded retry
  helper; the third failure sets the LSM-only degraded path instead of entering
  a terminal error state.
- BMI323 raw samples exist in `imu_manager.c::imu_bmi323_task`; the calibration
  API receives converted SI units, so the requested `300 LSB` precheck belongs
  at the raw producer boundary.
- `imu_manager_finalize_dual_initialization()` requires both sensors to init and
  starts the BMI producer. An LSM-only finalizer is needed for a disconnected
  BMI323 path.
- `chassis_runtime_get_primary_snapshot()` rejects all samples while
  `g_attitude_is_ready == 0`, and the runtime resets heading state on stop,
  startup gating, and invalid IMU data. The controller currently anchors yaw
  only when the first straight command locks.
- MotorBoard currently uses direct logical-to-board mapping and
  `ENCODER_DIR_SIGN={1,-1,1,1}` exactly once at the PID feedback input. PWM is
  sent without an extra M2 sign multiplication.

## Constraints

- Keep SCBP/BLE schemas and IDs unchanged.
- Do not change wheel PID gains or the encoder sign convention.
- Build/runtime proof is distinct from physical sensor, UART, motor, and vehicle
  acceptance.
