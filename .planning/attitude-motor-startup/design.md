# Design: Attitude-Gated Motor Startup

## Architecture

Add `Application/Safety/attitude_startup_coordinator.[ch]`. The coordinator is
an RTOS task with a small periodic loop. It starts in LOCKED, calls
`motor_board_force_stop()` while IMU is not ready or is failed, and transitions
to READY only when the dual boot manager, filter, attitude zero gate, and
`attitude_get_status() == AHRS_READY` all report success. It also requires both
sensor update counters to advance for five 20 ms cycles before publishing
`g_attitude_is_ready` and calling the existing `motor_board_task_start()` once.

The CM7 normal build becomes the default (`SMARTCAR_MOTOR_BOARD_ONLY=OFF`),
while the existing option remains available for isolated motor-board bring-up.
`main()` initializes the motor-board transport/protocol, immediately requests
the first forced stop, starts the IMU/S3/coordinator tasks, and deliberately
does not start the motor-board task directly.

## Safety Behavior

- Sensor/calibration failure leaves the flag false and repeats forced stop with
  a bounded diagnostic log.
- Static calibration accepts exactly 1000 BMI323 gyro samples. Motion during a
  window resets the accumulator and window; after three restarts it enters the
  existing static-window failure state and remains locked.
- No PWM/PID algorithm changes are made.
- The 100 Hz attitude loop remains owned by the existing `imu_task`; no second
  estimator is introduced.

## Verification

Use `git diff --check`, inspect the protected paths for no task-local diff,
scan the source for the state transitions and call ordering, configure CM7
with `-DSMARTCAR_MOTOR_BOARD_ONLY=OFF`, build the canonical target with
`--clean-first`, run the host calibration test, and check the ELF with
`arm-none-eabi-nm -u`. Report errors/warnings separately from hardware
acceptance.
