# Progress

## 2026-08-25

- Restored the approved scope from the interrupted thread: unconditional
  business-task creation with existing attitude/PWM safety admission.
- Source audit complete; implementation and build remain pending.
- `main.c` now always starts the attitude coordinator and chassis runtime;
  IMU admission failures keep the existing motor stop gate closed.
- Added `imu_boot_manager_mark_startup_failure()` plus heap/stage diagnostics
  for IMU, attitude-gate, and chassis task creation failures.
- The 50 ms SRP calibration heartbeat now carries the lifecycle stage,
  degraded bit, sample counters, and error byte.
- Added a bounded pre-sync LOG backlog; the logger retries those records over
  SRP/FFE3 after `CMD_SYNC_REQ` establishes the session.
- CM7 clean build passed: FLASH 199,212 B (19.00%), RAM 64,832 B (49.46%),
  RAM_D2 512 B.
- ESP-IDF 5.5.4 isolated build passed in
  `ESPS3/build-codex-business-flow-20260825`.
  `smartcar_s3_gateway.bin` is 0xB2540 (90% free in the 0x700000 app
  partition); `idf.py size` reports 730,313 bytes total image size.
- Incremental CM7 build completed without errors; the existing CM7 ELF/BIN
  remain present in `STM32H757/CM7/build/Debug`.
- SRP host codec test passed with `SRP_CODEC_TEST_PASS`; `git diff --check`
  passed.
- Hardware acceptance remains pending: no matching images were flashed and no
  live UART2/USART2, BLE FFE3, sensor, or vehicle-motion capture was taken.
