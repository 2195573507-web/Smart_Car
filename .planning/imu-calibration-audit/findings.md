# IMU/Radar Calibration Audit Findings

This file is append-only for this task. Agent reports will be recorded here
after each independent handoff, with source evidence and explicit uncertainty.

## Agent 1: STM32 calibration state machine

The independent handoff identified stale `imu_vibration.complete` as the P0
cause. `imu_boot_manager_update()` accepted `VIBRATION_SAMPLE` during the
2-second settle window, before `imu_vibration_start()` cleared the prior
window's completion flag. It also identified the `RADAR_PWM_READY` branches
that could clear `event_waiting` and advance the index before `CAL_ACK_RX`.
Evidence: `STM32H757/Middleware/Calibration/imu_boot_manager.c:365-380,
512-533,554-597,639-673` and `imu_vibration.c:61-110`.

## Delegation status

Agent 2 was created but its first turn failed at the model-capacity boundary.
The remaining requested roles were not fabricated; the owner performed
read-only source cross-checks for sampling, RMS, STM/S3 protocol, S3
idempotence, UART/parser, RTOS, logging, attitude, architecture, regression,
and final timing. Those checks are recorded in the task summary and remain
static evidence only. No hardware or runtime claim is made.

## Owner cross-checks

- Sampling: `imu_task` runs at 10 ms (`imu_manager.c:509-543`) and publishes
  one complete accel+mag snapshot only when both reads succeed
  (`imu_manager.c:259-314`); calibration counters increment inside the
  calibration modules, not in a loop over cached data.
- RMS: `imu_vibration_start()` clears `sum`, `sum_square`, `sample_count`, and
  `complete` (`imu_vibration.c:61-69`); one valid snapshot increments once and
  computes RMS at exactly 1000 samples (`:72-110`). The stale completion was
  an upper-layer admission bug, not a double accumulator.
- Protocol/S3: STM ACK matching already requires `event_waiting` and the
  current `pending_event_id` (`imu_boot_manager.c:650-673`); S3 repeats an ACK
  without advancing after its first accepted event (`radar_calibration_manager.c:264-320`).
- UART/RTOS/logging/attitude: AA55 parser and CRC paths are unchanged; UART
  uses a 512-byte ring with a 5 ms receive timeout and 256-word task stack;
  attitude intentionally collects 500 zero samples after IMU_READY, so
  transient `WAIT_CAL` is expected. `%f` RMS formatting was unsafe for the
  embedded logger and is replaced with fixed-point text.

All owner cross-checks were read-only before the scoped implementation patch;
build and hardware evidence are still pending.

## 2026-08-07 Full Rerun

The earlier partial delegation above is retained as historical context only.
It is not accepted as completion evidence for the current request. Agents 1-11
will be rerun against current source, followed by a separate Agent 12 review.

## 2026-08-07 Current-Source Primary Audit

- The user waived the multi-Agent requirement after dispatcher failures. These
  findings are primary-agent source analysis, not independent specialist reports.
- STM admission now requires `VIBRATION_SAMPLE && vibration_started`, and each
  `imu_vibration_start()` clears sums, squares, count, result, and completion.
  That repairs stale completion at the accumulation boundary, but an explicit
  timestamp gate is still warranted so a cached snapshot cannot increment a
  window twice if call topology changes.
- The current static sample constant is 1000 at 100 Hz, which is about 10 seconds
  and does not satisfy the requested approximately 50-second static window. The
  minimum compatible correction is 5000 accelerometer-only samples. Magnetometer
  input remains excluded from static bias accumulation.
- STM ACK handling requires `event_waiting`, matching event ID, and matching
  state before it changes stage. Duplicate ACKs are therefore inert. READY frames
  are validated against expected speed; early next-level READY is cached and
  cannot clear `event_waiting`.
- S3 accepts vibration completion in `RADAR_WAIT_EVENT`. Because all five levels
  reuse event ID 2, an old id=2 frame arriving after the next PWM ACK is
  indistinguishable by ID alone and can advance the new level. Protocol-compatible
  mitigation is a per-level not-before deadline based on 2 seconds settling plus
  1000 samples at 100 Hz. A transaction token would be stronger but is excluded
  because payload/protocol compatibility must be preserved.
- Current STM source no longer emits periodic PING. It retains PONG parsing and
  uses UART `last_rx_time` for rate-limited stale-link diagnostics. S3 retains a
  response to an explicitly received PING, preserving parser compatibility without
  periodic heartbeat traffic.
- Attitude intentionally collects 500 zero-reference samples after filter/IMU
  readiness. Several seconds of `AHRS_WAIT_CAL` are therefore expected design
  behavior; no attitude algorithm change is indicated.
- `uart_link_task` high-water output of 58 words is low but nonzero. The calibration
  repair does not add UART task local buffers or blocking work; build evidence cannot
  prove absence of a runtime HardFault, so this remains a hardware-validation risk.

## 2026-08-07 Final Review

- The final concurrency pass moved static/vibration accumulator reset ahead of
  publishing the sampling state. A future second caller can no longer observe
  `vibration_started=1` while the previous level's `complete/sample_count/RMS`
  is still present.
- The S3 prior-stage `id=2` guard is 11 seconds after matching PWM ACK. This is
  intentionally shorter than the real 12-second STM window so the 1000th sample
  at the 100 Hz boundary is not misclassified. It does not shorten calibration:
  STM still requires a 2-second settle and 1000 unique timestamps.
- Theoretical minimum after radar-zero confirmation is 52 seconds static plus
  60 seconds vibration, or 112 seconds plus UART/protocol scheduling overhead.
- Host source tests passed for 5000 unique static timestamps, duplicate timestamp
  rejection, accelerometer-only bias, and independent 20/40 PWM RMS windows.
- STM32 CM7 clean build rebuilt 58 steps and linked the final ELF: FLASH 79,480
  bytes (7.58%), RAM 39,352 bytes (30.02%).
- ESP32-S3 build completed 9 steps after the final change; app image is 0xb1800
  bytes and the smallest app partition has 90% free.
- Static search found no periodic PING encoder/sender. PING/PONG enum parsing and
  the S3 response to an externally received PING remain compatible.
- The LSM303 driver is configured for 100 Hz and the IMU task for 10 ms, but no
  data-ready bit is checked. Freshness is therefore source-enforced by timestamp
  admission, not hardware-proven in this task.
- Final exactly-once hardening requires a matching pending event ID *and* its
  corresponding state before STM clears `event_waiting`. Vibration completion
  also rechecks `VIBRATION_SAMPLE && vibration_started` while holding the boot
  lock, so re-entry cannot transmit a second id=2 event for one 1000-sample
  window. The final clean CM7 ELF is 118,784 bytes (`text=79,440`, `data=108`,
  `bss=39,236`).
