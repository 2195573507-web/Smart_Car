# IMU/Radar Calibration Audit Progress

## 2026-08-07

- Loaded mandatory workflow skills and confirmed the workspace is dirty.
- Preserved unrelated user changes; no firmware edits have been made.
- Started the twelve-role read-only audit workflow.
- Restarted the audit because the prior attempt did not produce twelve
  independent reports. The current run will not substitute owner analysis for
  a requested specialist handoff.
- Confirmed the current worktree contains unrelated user changes and extensive
  untracked firmware trees. Target-file snapshots and diffs will be used to
  preserve that baseline.
- Confirmed the user pre-authorized implementation after all reports and final
  review, so the post-audit design gate is report completion rather than an
  additional design-choice question.
- The collaboration dispatcher repeatedly rejected empty payloads before any
  specialist Agent was created. The user then explicitly waived the multi-Agent
  requirement and instructed the primary agent to finish the task directly.
- Resumed with a current-source primary-agent audit. No flash, monitor, serial,
  BLE, hardware, UUID, GPIO, frame-format, CRC, or LSM303 calibration-design
  changes are authorized.
- Completed current-source tracing of STM state/ACK/READY admission, LSM303 sample
  publication, vibration accumulation/reset, S3 calibration event handling, UART
  parser/heartbeat behavior, and attitude readiness gating.
- Selected a protocol-compatible repair direction: 5000 accelerometer-only static
  samples, fresh-timestamp admission, explicit per-window resets, and an S3
  vibration-event not-before guard.
- Implemented the scoped STM/S3 state, sampling, RMS reset, idempotence,
  last-RX freshness, stack-margin, log, and calibration-description changes.
- A combined read-only driver inspection command failed at JavaScript parse
  time due to quoting. It made no filesystem or process changes and will be
  replaced by smaller independent checks.
- Completed a final state-machine concurrency review and moved each accumulator
  reset before the corresponding sampling state becomes visible.
- Adjusted the S3 duplicate-event guard to 11 seconds to retain a one-second
  boundary margin while STM remains authoritative for the full 12-second level.
- Re-ran STM32 CM7 clean build: 57 files cleaned, 58 build steps completed,
  final ELF linked with FLASH 7.58% and RAM 30.02%.
- Rebuilt ESP32-S3 in `build-codex-imu-calibration`: 9 steps completed, final
  image size 0xb1800, smallest app partition 90% free.
- Confirmed no periodic PING sender and no changes to protocol format, CRC, BLE
  UUIDs, UART GPIO, IOC, or LSM303 driver/calibration design.
- No flash, monitor, serial, BLE, or hardware connection was performed.
- Added the final STM exact-once state-match and in-lock completion guards, then
  repeated the CM7 clean build. The final ELF linked at 11:38 and a subsequent
  incremental build reported no work remaining.
