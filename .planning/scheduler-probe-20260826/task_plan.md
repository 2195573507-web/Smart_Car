# CM7 Scheduler Probe and SRP Context Diagnosis

## Objective

Restore the STM32-first SRP startup response path when the CM7 image produces no
UART2 data, while preserving the production SRP gate and zero-PWM safety.

## Phases

- [completed] Audit current startup, handshake, and TX lock boundaries.
- [completed] Separate the pre-sync handshake response from normal SRP gating and add UART state diagnostics.
- [completed] Clean-build the canonical CM7 Debug image and run SRP/static checks.
- [completed] Record hardware-only acceptance criteria and remaining evidence.
- [completed] Add a logic-analyzer-only USART2 physical probe and rebuild CM7.
- [completed] Add a separate UART2/RTOS isolation heartbeat and raw fault
  markers, then rebuild the canonical CM7 Debug image.
- [completed] Mitigate the captured `E5` stack overflow by expanding the
  active CM7 task budgets, removing SRP TX stack buffers, and rebuilding.
- [completed] Remove all UART2 raw diagnostic traffic, restore SRP-only Debug
  configuration, fix the parser-context handshake lock, and rebuild CM7.

## Boundaries

- Do not revert unrelated dirty worktree changes.
- Do not change SRP v4 framing, UART2 routing, BLE UUIDs, or motor safety gates.
- Debug diagnostics and probes are disabled by default; USART2 must carry only
  standard SRP traffic during handshake and telemetry.
- Builds and source inspection do not prove flashed-image, UART, BLE, or vehicle behavior.

## Verification Evidence

- Source audit: current RX callback only re-arms DMA, pushes the ring, and notifies
  `uart_link_task`; SRP handshake response is produced in `s3_service_task`.
- Source audit: `main()` still has an unbounded `PWR_FLAG_VOSRDY` wait and
  fail-stop `Error_Handler()` paths; no sensor scan loop runs before the scheduler.
- Source audit: pre-scheduler dynamic allocations include the log queue, UART/S3
  mutexes, and all application task stacks; task-create return values are not
  uniformly surfaced, so admission telemetry is required.
- CM7 canonical Debug configure: `STM32H757/CM7/build/Debug/CMakeCache.txt`
  contains `CMAKE_BUILD_TYPE=Debug`, `SMARTCAR_SCHEDULER_PROBE=ON`, and
  `SMARTCAR_RAW_DIAGNOSTICS=ON`.
- `STM32H757/CM7/CMakePresets.json` now persists
  `SMARTCAR_SCHEDULER_PROBE=ON` in the `Debug` preset, so the setting survives
  a fresh configure; an explicit `-DSMARTCAR_SCHEDULER_PROBE=OFF` remains
  available for a production-style build.
- CM7 Debug probe build passed on 2026-08-26; the current ELF reports
  `FLASH=188,616 B`, `RAM=64,320 B`, and has matching `.bin`/`.hex` exports in
  `STM32H757/CM7/build/Debug`.
- The linked vector table is at `0x08000000`; its SVC, PendSV, and SysTick
  entries resolve to the project handlers, which branch to
  `vPortSVCHandler`, `xPortPendSVHandler`, and `xPortSysTickHandler`.
- FreeRTOS configuration uses a 32 KiB dynamic heap and
  `configUSE_TIMERS=0`; scheduler startup allocates Idle only, so no Timer task
  allocation is involved.
- Host SRP codec test passed with `-Wall -Wextra -Werror`.
- S3 ESP-IDF 5.5.4 `fullclean + build` passed in isolated
  `ESPS3/build-srp-debug-20260826`; `smartcar_s3_gateway.bin` linked with 90%
  smallest-app-partition space remaining.
- `git diff --check` passed; `arm-none-eabi-nm -u` reported no undefined
  symbols. The Common/SRP host test passed with `-Wall -Wextra -Werror`, and
  the isolated ESP-IDF 5.5.4 `fullclean + build` passed with
  `smartcar_s3_gateway.bin` at `0xb2540` bytes and 90% smallest-partition
  space remaining.
- Matching-image flash, live USART1/UART2/S3 counters, BLE delivery, sensor
  behavior, and vehicle acceptance remain unverified.
- The 2026-08-26 physical-probe Debug build enables
  `SMARTCAR_UART2_PHYSICAL_PROBE=ON`. It sends `AA 55 DE AD BE EF` once after
  `HAL_UART_Init`, clears RX line errors before that send and before every SRP
  TX, and writes `0x5A` to USART2 TDR at IRQ entry when TXE is available.
  This image intentionally injects raw UART2 bytes and is diagnostic-only.

## Errors

None yet.
