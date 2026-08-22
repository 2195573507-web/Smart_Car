# Findings: Attitude-Gated Motor Startup

## Confirmed Source Facts

- `imu_runtime_start()` creates the existing `imu_task`, whose manager loop is
  10 ms (100 Hz), plus the telemetry/debug task.
- `imu_boot_manager` owns `INIT -> SELF_TEST -> STATIC_CALIBRATION -> READY /
  FAILED`; current static calibration computes BMI gyro bias and validates
  sample quality before committing leveling.
- `attitude_zero_is_ready()` is the existing final alignment/zeroing gate.
- `motor_board_force_stop()` clears targets, resets all PID instances, clears
  ramp targets, sets the motor stop latch, and queues zero PWM.
- `motor_board_task()` currently resets its stop latch to false during task
  startup. Therefore delaying task creation is required to make the startup
  gate effective without editing that protected file.
- The final coordinator gate additionally requires `attitude_get_status()` to
  equal `AHRS_READY` and five consecutive 20 ms cycles with both LSM303 and
  BMI323 update counters advancing.
- BMI323 gyro bias collection is capped at exactly 1000 accepted samples. The
  static window is 6000 ms to accommodate the normal 200 Hz source while
  retaining the existing 30 s zero-PWM synchronization wait before sampling.
- Static motion is detected from gyro norm, acceleration norm, and static
  variance/RMS. Each reset clears calibration and keeps zero PWM; the fourth
  motion reset transitions to `IMU_ERROR_STATIC_WINDOW`.

## Risks / Limits

- The current dual lifecycle requires both LSM303 and BMI323 initialization and
  static quality. Device WHO_AM_I, motion stability, UART, PWM, and motor
  behavior remain unverified until a matching image is flashed and captured.
- The coordinator can expose readiness and start the motor task only after the
  existing attitude zero gate; it does not alter App/S3 command parsing.
- A normal IMU image requires `SMARTCAR_MOTOR_BOARD_ONLY=OFF`; a motor-board-only
  bring-up image intentionally has no attitude gate and remains a separate
  configuration.
- The four protected paths were not edited by this task. `s3_service.c` still
  has an unrelated pre-existing worktree diff and was preserved.

## Verification Evidence (2026-08-22)

- Clean CM7 build: `cmake --preset Debug -DSMARTCAR_MOTOR_BOARD_ONLY=OFF` then
  `cmake --build build/Debug --target Smart_Car_H757_CM7 --clean-first -j2`;
  0 warnings and 0 errors; FLASH 176228 B, RAM 59920 B, D2 RAM 512 B.
- Host calibration test compiled with `-Wall -Wextra -Werror` and passed the
  1000-sample cap, bias, motion, and reset cases.
- `arm-none-eabi-nm -u` reported no undefined symbols and `git diff --check`
  was clean.
