# Progress

## 2026-08-26

- Restored context from the prior scheduler/business-flow investigation.
- Audited `main.c`, UART RX/TX, SRP service, FreeRTOS configuration, and S3 log
  relay boundaries.
- Confirmed the reported ISR response is not represented by the current source;
  matching flashed-image provenance remains an open requirement.
- Design selected: default-off highest-priority 50 ms SRP LOG probe plus startup
  admission and handshake context markers.
- Completed source audit: no ISR TX mutex ownership was found; `s3_service` parses
  and responds from its task, while the RX ISR only re-arms DMA and notifies the
  UART worker. The startup path has no sensor wait loop before scheduler start,
  but retains the clock-ready loop and fail-stop HAL error paths.
- Existing worktree probe code is being treated as an opt-in diagnostic artifact;
  default production behavior remains SRP-gated and motor-stop safe.
- Reconfigured the canonical `STM32H757/CM7/build/Debug` directory with
  `SMARTCAR_SCHEDULER_PROBE=ON` and built the probe image successfully:
  `Smart_Car_H757_CM7.elf` (FLASH 185,576 B, RAM 64,112 B), plus refreshed
  `.bin` and `.hex` artifacts. ELF strings contain `RTOS_PROBE`,
  `RTOS_ADMISSION`, and `SCHEDULER_RETURNED` markers.
- Persisted the same setting in `STM32H757/CM7/CMakePresets.json` and verified
  `cmake --preset Debug` keeps `SMARTCAR_SCHEDULER_PROBE=ON` in the Debug
  cache; the Debug target is already up to date.
- Re-ran the Common/SRP host codec test with strict warnings; it passed.
- Built S3 from a clean isolated directory using ESP-IDF 5.5.4 and the
  installed `idf5.5_py3.14_env`; `smartcar_s3_gateway.bin` is 0xB2540 with
  90% smallest-app-partition space remaining.
- `git diff --check` passed. No flash, UART capture, BLE notification capture,
  sensor/PWM test, or vehicle acceptance was performed.

## 2026-08-26 continuation

- Collected the production-style `OFF` regression result and restored the
  canonical `Debug` preset with both `SMARTCAR_SCHEDULER_PROBE=ON` and
  `SMARTCAR_RAW_DIAGNOSTICS=ON`.
- Rebuilt `Smart_Car_H757_CM7.elf` successfully: `FLASH=188,616 B`,
  `RAM=64,320 B`; regenerated `.bin` and `.hex` from the same ELF.
- Verified the vector table address and FreeRTOS exception symbols, confirmed
  no undefined ELF symbols, and reran `git diff --check`.
- Reran the strict Common/SRP host test successfully.
- Reran an ESP-IDF 5.5.4 isolated `fullclean + build`; the S3 image is
  `0xb2540` bytes with 90% smallest app partition space remaining.
- No matching-image flash or live USART1/UART2/S3/BLE/vehicle capture was
  performed; runtime root-cause classification remains pending that evidence.

## 2026-08-26 handshake repair continuation

- The implementation boundary is now limited to a direct, pre-sync
  `RSP_BOOT_INFO` SRP send, UART2 TX state/error snapshots and bounded stale
  `gState` recovery, plus a pre-scheduler UART2-ready raw marker.
- Implemented the direct response via `srp_link_send()` to its physical
  `uart_link_send()` transport, before `HOST_SYNCED` is published. A valid
  `CMD_SYNC_REQ` now reaches that branch before the BUS_OFF early return.
- Preserved DMA/IDLE receive ownership and added explicit post-arm raw
  `PRIMASK`/`BASEPRI` snapshots. Every TX clears ORE/NE/PE/FE, records prior
  `gState`/`ErrorCode`, and uses the TX-owner mutex before a stale-state abort.
- The pre-sync probe now emits `RTOS_PROBE_PRE_SYNC` on raw USART1 and does not
  increment its SRP failure counter; it uses the normal SRP LOG route only
  after synchronization.
- Canonical Debug CM7 build passed: FLASH `189,232 B`, RAM `63,808 B`, D2 DMA
  buffer `s_dma_rx` remains at `0x30000000`. Strict SRP codec test, no-undefined
  symbol check, ELF marker check, and `git diff --check` passed. No device was
  flashed and no live UART capture was made.

## 2026-08-26 final continuation

- Audited the probe transport path and found it bypassed the SRP session gate
  by calling `uart_link_send()` directly.
- Changed `scheduler_probe_send_once()` to call `s3_service_send_log()`, so
  pre-sync CM7 output remains silent on USART2 while raw USART1 markers still
  identify scheduler/task progress.
- Rebuilt canonical Debug successfully: `FLASH=188,472 B`, `RAM=63,800 B`;
  regenerated `.bin` and `.hex` from the same ELF.
- Rechecked `SMARTCAR_SCHEDULER_PROBE=ON` and
  `SMARTCAR_RAW_DIAGNOSTICS=ON`, the FreeRTOS exception symbols/vector
  entries, `arm-none-eabi-nm -u`, and `git diff --check`.
- Re-ran the strict Common/SRP host codec test successfully.
- Hardware capture remains the required next evidence layer: flash matching
  CM7/S3 images and capture USART1 markers plus UART2/S3 traffic before
  classifying the runtime stall.

## 2026-08-26 current continuation

- Reconfigured the canonical `STM32H757/CM7/build/Debug` preset and completed a
  `--clean-first` CM7 build with `SMARTCAR_SCHEDULER_PROBE=ON` and
  `SMARTCAR_RAW_DIAGNOSTICS=ON`; the link reports FLASH `188,472 B` and RAM
  `63,800 B`.
- Regenerated `.bin` and `.hex` directly from the resulting ELF. The strict
  Common/SRP host codec test passed, `arm-none-eabi-nm -u` reported no
  undefined symbols, and `git diff --check` passed.
- Verified the linked vector table at `0x08000000`: SVC/PendSV/SysTick entries
  resolve to the project handlers, which branch to the FreeRTOS port handlers;
  the diagnostic `Default_Handler` branch resolves to the raw USART1 marker.
- Rebuilt the isolated ESP-IDF 5.5.4 S3 directory successfully; the gateway
  image is `0xb2540` bytes with 90% smallest-app-partition space remaining.
- Re-enumerated STM32CubeProgrammer before any write operation. No DFU,
  ST-Link, J-Link, or target serial endpoint is currently visible, so no flash,
  reset, UART capture, BLE capture, sensor test, or vehicle test was executed.

## 2026-08-26 physical UART2 probe

- Added an opt-in `SMARTCAR_UART2_PHYSICAL_PROBE` build option and enabled it
  in the canonical Debug preset. The probe emits a one-time raw `AA 55 DE AD
  BE EF` frame immediately after the private USART2 handle completes
  `HAL_UART_Init`; it clears UART line-error flags before that send.
- Moved `uart_link_init()` ahead of remaining peripheral initialization, kept
  RX DMA/IDLE arming before `vTaskStartScheduler()`, and preserved USART2/DMA1
  Stream0 NVIC priority 5. The diagnostic USART2 IRQ writes `0x5A` to TDR only
  when TXE is available, then executes the normal HAL handler.
- Verified the pre-sync `RSP_BOOT_INFO` path still bypasses the ordinary sender
  gate and that `uart_link_send()` clears ORE/NE/PE/FE, records `gState`, and
  recovers a stale non-ready TX state while owning its mutex.
- Reconfigured and clean-built `STM32H757/CM7/build/Debug` successfully:
  FLASH `189,336 B`, RAM `63,816 B`, RAM_D2 `512 B`. Strict Common/SRP host
  codec test, `arm-none-eabi-nm -u`, and `git diff --check` passed. ELF
  disassembly confirms the startup `HAL_UART_Transmit(..., 6, 100)` call and
  the IRQ `0x5A` TDR write. No device was flashed and no logic-analyzer capture
  was made.

## 2026-08-26 UART2/RTOS isolation continuation

- Confirmed ReceiveToIdle DMA is re-armed from every RX event callback and both
  USART2 and DMA1 Stream0 have NVIC priority 5, matching the configured
  FreeRTOS syscall-safe threshold.
- Enabled the isolated `uart2_iso` task in the canonical Debug preset. It sends
  `C3 3C A5 5A` through the existing TX mutex and then delays 100 ms; raw UART2
  fault markers are `E1`-`E7`.
- Reconfigured and clean-built the canonical CM7 Debug target successfully:
  FLASH `189,876 B`, RAM `63,824 B`, RAM_D2 `512 B`. All four diagnostic cache
  switches are `ON`; `arm-none-eabi-nm -u` and `git diff --check` passed.
- Disassembly confirms `USART2_IRQHandler` writes `0x5A` to USART2 TDR when a
  receive/IDLE/error event and TXE are set; it also confirms the isolation
  task calls `HAL_UART_Transmit(..., 4, 50)` and `vTaskDelay(100)`.
- The canonical current artifact is
  `STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf`. Existing `.bin` and
  `.hex` files were not rebuilt by this target and have older timestamps, so
  they must not be flashed for this test. No target flash or live capture was
  performed.

## 2026-08-26 E5 stack-overflow repair

- Field capture reported `0x5A 0xE5`, proving the raw USART2 RX marker and the
  FreeRTOS stack-overflow hook. The source cannot identify the overflowing
  task from that byte alone, so the repair covered every active receive,
  telemetry, attitude, chassis-control, logging, and diagnostic task with a
  sub-1024-word allocation.
- Raised the FreeRTOS heap from 32 KiB to 48 KiB and increased the active task
  stack budgets to the values recorded in `findings.md`. No source-level
  `chassis_task` exists; `motor_board` is the relevant control-task expansion.
- Replaced two automatic `SRP_MAX_FRAME_SIZE` (512-byte) buffers in
  `srp_link_send()` with `srp_link_t::tx_scratch`; protected direct BOOT_INFO
  sends with the existing S3 link mutex while retaining their pre-sync gate
  bypass.
- Clean CM7 Debug build passed: FLASH `189,620 B`, RAM `80,720 B` (61.58%),
  RAM_D2 `512 B`. `arm-none-eabi-nm -u` and `git diff --check` passed. The
  strict existing SRP codec test and a focused host test of two non-ACK
  scratch-buffer sends passed with `-Wall -Wextra -Werror`.
- No target flash or new logic-analyzer capture was performed. The only valid
  artifact remains `STM32H757/CM7/build/Debug/Smart_Car_H757_CM7.elf`; stale
  `.bin` and `.hex` side files remain unsuitable for this test.

## 2026-08-26 UART2 traffic convergence

- Removed all UART2 raw diagnostic injection and the temporary `uart2_iso`
  heartbeat task; standard SRP remains the only CM7 USART2 producer.
- Fixed the `RSP_BOOT_INFO` parser-context recursive mutex acquisition by using
  the already-held S3 link lock around the direct `srp_link_send()` call.
- Reconfigured Debug with scheduler/raw diagnostics OFF and completed a clean
  build: `Smart_Car_H757_CM7.elf`, FLASH `185,448 B`, RAM `80,464 B`,
  RAM_D2 `512 B`.
- `git diff --check` passed and no source references to the removed physical or
  isolation diagnostics remain. No target flash or live S3/BLE capture was
  performed; field sync/telemetry acceptance is still pending.
