# Task Plan: Attitude-Gated Motor Startup

## Goal

Open the completed STM32H757 CM7 attitude lifecycle and add a strict boot gate
that keeps the motor board stopped until sensor initialization, static
calibration, leveling/yaw zeroing, and attitude readiness have completed.

## Scope

- Modify only CM7 startup coordination, the existing IMU calibration/boot
  manager hooks needed for the 1000-sample motion gate, CM7 build default, and
  task-local planning/design records.
- Keep these files byte-for-byte unchanged by this task:
  `STM32H757/CM7/Core/Inc/wheel_control_params.h`,
  `STM32H757/CM7/Core/Src/pid_controller.c`,
  `STM32H757/Middleware/MotorBoard/motor_board_task.c`,
  `STM32H757/Middleware/Communication/Services/s3_service.c`.
- Do not claim hardware acceptance from source or build evidence.

## Phases

- [x] Analyze current startup, IMU lifecycle, motor stop API, and dirty scope.
- [x] Implement startup coordinator and open normal CM7 attitude image.
- [x] Verify state-gate source behavior and protected-file integrity.
- [x] Reconfigure and build CM7 with captured errors/warnings.
- [x] Record evidence, risks, and remaining hardware-validation gaps.

## Current Design

`main()` initializes and immediately requests a motor-board forced stop after
the transport/protocol are ready. The IMU and S3 tasks run while the motor-board
task is not created. A coordinator task maintains the stop request, waits for
the full IMU lifecycle, `AHRS_READY`, and five consecutive advancing LSM303 /
BMI323 update cycles, then publishes `g_attitude_is_ready` and starts the
motor-board task. Failed IMU lifecycle states never release the gate. Static
calibration requires exactly 1000 BMI323 gyro samples and resets on motion,
with a bounded three-restart limit. The normal image is also the `main.c`
fallback when the build definition is omitted; the motor-board-only image
remains an explicit `SMARTCAR_MOTOR_BOARD_ONLY=ON` bring-up configuration.

## Verification Evidence

```text
cmake --preset Debug -DSMARTCAR_MOTOR_BOARD_ONLY=OFF
cmake --build build/Debug --target Smart_Car_H757_CM7 --clean-first -j2
```

The final `Smart_Car_H757_CM7.elf` build completed with 0 warnings and 0
errors (FLASH 176228 B, RAM 59920 B, D2 RAM 512 B). The host calibration test
passed, `arm-none-eabi-nm -u` reported no undefined symbols, and
`git diff --check` was clean. Hardware acceptance remains open for WHO_AM_I,
static-motion behavior, UART, PWM, motor, and vehicle captures.
