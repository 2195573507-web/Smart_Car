# Phase 1-2 Firmware Recovery

## Objective

Restore the requested SRP safety timeout observability and verify the existing
BMI323 Primary / LSM303 Redundant / DualAHRS and calibration paths without
changing protocol bytes, hardware routing, or the radar/motor/BLE feature scope.

## Phases

| Phase | Status | Deliverable |
| --- | --- | --- |
| Scope and source audit | Complete | Existing CM7/S3/SRP/IMU paths inspected; protected boundaries recorded |
| CM7 safety and diagnostics | Complete | 200 ms timeout and SRP_S3 counters/snapshot |
| Static IMU/DualAHRS audit | Complete | Source evidence for Primary/Redundant and calibration/attitude schema |
| CM7 clean build | Complete | `STM32H757/CM7/build/Debug` build evidence |
| ESP32-S3 clean build | Complete | ESP-IDF 5.5.4 isolated clean build evidence |
| Post-reset SRP recovery and log bridge fix | Complete | 100 ms sync heartbeat, 1500 ms S3 liveness recovery, DEBUG log acceptance |
| Continuous STM stream and symmetric startup recovery | Complete | 50 ms STM fallback telemetry, idempotent sync, 500 ms S3 probing, 1500 ms S3 watchdog |
| Final evidence report | Complete | Modified files, diagnostics, build artifacts, hardware gaps |

## Scope Boundaries

- Protected: GPIO/CubeMX, SRP wire format, BLE UUIDs, USART6 motor-board work,
  radar point-cloud parsing and avoidance, unrelated dirty-worktree changes.
- Source/build evidence does not prove flashed UART, BLE, sensor identity, or
  vehicle stopping behavior.

## Errors

| Error | Attempts | Resolution |
| --- | --- | --- |
