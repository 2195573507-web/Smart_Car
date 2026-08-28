# Phase 1-2 Recovery Progress

## 2026-08-25

- Confirmed user approval for the 200 ms safety timeout and bounded diagnostic
  additions.
- Recorded current dirty-worktree state and protected boundaries.
- Baseline CM7 and ESP32-S3 builds were reported by the prior agent as passing;
  this turn will rerun the requested clean builds after the edit.
- Changed `S3_SERVICE_S3_FRAME_TIMEOUT_MS` from 3000 ms to 200 ms.
- Added cumulative S3 frame-timeout, BUS_OFF, force-stop success/failure
  counters and a last-timeout snapshot containing elapsed time, REC, TEC, and
  pre-demotion sync state. The existing SRP wire format and DualAHRS paths were
  unchanged.
- CM7 clean build passed: `STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf`
  (FLASH 18.80%, RAM 48.25%).
- ESP-IDF 5.5.4 isolated build passed:
  `ESPS3/build-codex-phase12-20260825/smartcar_s3_gateway.bin` (0xB22F0 bytes,
  90% free in the smallest app partition).
- Investigated the post-reset failure: STM's 200 ms receive watchdog was not
  matched by S3's 1000 ms `RADAR_STATUS` cadence, and S3 stayed in the synced
  state after STM reset. Reused existing `CMD_SYNC_REQ`/`RSP_BOOT_INFO` IDs to
  add a 100 ms synced heartbeat, track the last valid STM SRP frame, and
  restart UART/parser/link synchronization after 500 ms without a valid frame.
  The follow-up symmetric-startup pass widened only the S3-side receive
  watchdog to 1500 ms; STM's 200 ms safety timeout remains protected by the
  100 ms synchronized heartbeat.
- Fixed the S3 log bridge envelope check so valid `DEBUG` (level 0) STM logs
  are accepted; malformed envelopes now report source/level/text/frame sizes.
- CM7 clean build rerun passed on 2026-08-25:
  `STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf` (FLASH 18.80%, RAM
  48.25%). `git diff --check` passed.
- S3 build verification passed with ESP-IDF 5.5.4 in the isolated build
  directory; current binary is
  `ESPS3/build-codex-phase12-20260825/smartcar_s3_gateway.bin` (730416 bytes).
- Hardware acceptance remains pending: flash matching STM/S3 images and
  capture `STM_UART_DIAG`, sync recovery after an STM reset, 100 ms S3 TX
  cadence, `RADAR_PWM_READY TX/ACK`, `STATIC_CAL_DONE`, and absence of valid
  DEBUG `LOG_DROP` messages.
- Follow-up repair for the two-sided startup/streaming failure:
  `STM32H757/Middleware/Communication/Services/s3_service.c` now sends a
  synchronization-gated `IMU_CAL_STATUS` fallback every 50 ms from the
  independent `s3_service_task`, using `vTaskDelayUntil`. The stream does not
  depend on IMU runtime task creation, calibration completion, radar state, or
  motor enablement; sent/drop counters are included in `SRP_S3` diagnostics.
- STM `CMD_SYNC_REQ` handling remains idempotent in every host state and now
  suppresses repetitive successful-heartbeat log lines while still refreshing
  the session timestamp and queue counters on every request.
- S3 probing is now 500 ms while unsynchronized, keeps the existing 100 ms
  synchronized heartbeat to satisfy STM's 200 ms safety watchdog, and uses a
  1500 ms received-frame watchdog before returning to re-probe mode.
- UART2 `ReceiveToIdle_DMA` immediate rearm and UART error rearm paths were
  source-verified in `uart_link.c`; no additional ISR rewrite was needed.
- Follow-up verification passed: CM7 clean build (FLASH 18.84%, RAM 48.25%),
  ESP-IDF 5.5.4 build (`smartcar_s3_gateway.bin`, 0xB2540 bytes), SRP codec
  host test, and `git diff --check`.
- Physical acceptance is still pending. After flashing matching images,
  verify both power-up orders and hot reset with `STM_UART_DIAG`, `SRP_S3`,
  continuous `IMU_CAL_STATUS` reception, and reappearance of
  `SRP sync state=ESTABLISHED` within the requested window.
- Final task-scheduling/TX pass completed: `main.c` starts `uart_link_task` and
  `s3_service` before optional IMU/calibration task creation, preserving heap
  for the SRP recovery path. `s3_service` creates a 512-word task, runs a
  bounded 1 ms service loop, emits a synchronization-gated 50 ms fallback
  heartbeat without taking the calibration mutex, and logs task create/alive
  counters. `uart_link_send` uses blocking `HAL_UART_Transmit` with a 50 ms
  timeout and releases the TX mutex on every return path.
- Corrected the task-start diagnostic to use the project's single-argument
  `LOG_INFO` macro via `snprintf`; CM7 clean build then passed with
  `Smart_Car_H757_CM7.elf` at FLASH 18.86% and RAM 48.25%.
- Final verification passed: ESP-IDF 5.5.4 build produced
  `ESPS3/build-codex-phase12-20260825/smartcar_s3_gateway.bin` (0xB2540
  bytes), SRP codec host test printed `SRP_CODEC_TEST_PASS`, and scoped
  `git diff --check` passed.
