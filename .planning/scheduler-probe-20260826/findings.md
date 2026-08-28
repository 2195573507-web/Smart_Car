# Findings

- `HAL_UARTEx_RxEventCallback()` in `uart_link.c` does not parse SRP or send a
  response. It re-arms DMA, pushes bytes into the ring, and notifies the UART
  worker. The `RSP_BOOT_INFO` path is `s3_service_task` -> parser callback ->
  `s3_service_handle_sync_req()` -> `uart_link_send()`.
- `main.c` contains one unbounded clock-ready loop (`PWR_FLAG_VOSRDY`) and
  permanent `Error_Handler()` loops for HAL failures. No sensor scan or external
  signal wait was found before `vTaskStartScheduler()`.
- Existing business tasks use priorities 1-3; FreeRTOS `configMAX_PRIORITIES` is
  7, so a probe at priority 6 can expose a lower-priority task stall.
- `uart_link_send()` takes its TX mutex for at most 50 ms and uses blocking HAL
  TX. It is a task-context API; the current ISR callback does not call it.
- S3 accepts a valid SRP LOG envelope and forwards it through the existing log
  bridge to the BLE log characteristic, making `0x30` a low-risk probe payload.
- The existing 32 KiB FreeRTOS heap and dynamic task creation make pre-scheduler
  task-admission/heap markers necessary; a failed task create alone does not prove
  the scheduler failed.
- The current startup path calls `imu_runtime_start()` before
  `vTaskStartScheduler()`, but its normal IMU lifecycle only prepares state and
  creates two deferred initialization workers; sensor I/O occurs after the
  scheduler starts. The only visible pre-scheduler unbounded wait is the
  `PWR_FLAG_VOSRDY` loop in `SystemClock_Config()`; HAL failures enter the
  permanent `Error_Handler()` loop.
- `main.c` already contains a default-off SRP LOG probe and task-admission marker
  in the dirty worktree. The probe must remain opt-in and its 50 ms cadence must
  not be treated as production behavior until a probe image is explicitly built
  and flashed.
- The canonical CM7 Debug preset now explicitly enables the probe while the
  CMake option default remains OFF. A fresh `cmake --preset Debug` writes
  `SMARTCAR_SCHEDULER_PROBE=ON` to `build/Debug/CMakeCache.txt`.
- The resulting Debug probe image links successfully: FLASH 185,576 B (17.70%),
  RAM 64,112 B (48.91%), with ELF/BIN/HEX artifacts under `STM32H757/CM7/build/Debug`.
- The Common/SRP host test passed with strict compiler warnings; the S3 clean
  build passed under ESP-IDF 5.5.4 and produced `smartcar_s3_gateway.bin`
  (0xB2540, 90% smallest-app-partition space remaining).
- `git diff --check` passed and `arm-none-eabi-nm -u` reported no CM7 undefined
  symbols. These remain source/build proofs only; no matching-image flash or
  live UART/BLE/vehicle acceptance was performed.
- The 2026-08-26 canonical Debug rebuild has both diagnostic options enabled;
  its linked vector table is at `0x08000000` and resolves the three RTOS
  exception entries to the project handlers. The handlers preserve the
  FreeRTOS branches and only add bounded counters in the diagnostic image.
- `configTOTAL_HEAP_SIZE` is `32 KiB`, dynamic allocation is enabled, and
  `configUSE_TIMERS=0`; therefore the scheduler-start failure branch can be
  distinguished from a Timer task allocation failure. The added
  `SCHEDULER_RETURNED_HEAP` marker is only reached if `vTaskStartScheduler()`
  returns in the flashed diagnostic image.
- The current CM7 link reports `FLASH=188,616 B` and `RAM=64,320 B`. The
  `.bin` and `.hex` files were regenerated from that ELF after the rebuild;
  they are ignored build artifacts, while the repository's canonical flash
  input remains the ELF.
- A standalone SRP codec test passed with strict warnings, and the current
  isolated ESP-IDF 5.5.4 S3 build passed (`smartcar_s3_gateway.bin`:
  `0xb2540` bytes, 90% smallest app partition free). Neither result proves a
  flashed endpoint or a physical UART/S3 response.
- The scheduler probe initially encoded and sent its LOG frame directly through
  `uart_link_send()`, which bypassed the SRP pre-sync gate. It now uses
  `s3_service_send_log()`, preserving the `CMD_SYNC_REQ` admission boundary
  while retaining independent raw USART1 scheduler markers.
- The current clean CM7 link is reproducible with both diagnostic options on:
  FLASH `188,472 B`, RAM `63,800 B`; the generated `.bin`/`.hex` were refreshed
  from that ELF. The vector section is at `0x08000000`, and the three RTOS
  exception vectors point at the project wrappers and then the FreeRTOS port.
- The strict SRP host test and isolated ESP-IDF 5.5.4 build pass again. The S3
  gateway image is `0xb2540` bytes with 90% smallest-partition space free.
- A current `STM32_Programmer_CLI -l` probe enumeration found no DFU,
  ST-Link, J-Link, or board serial endpoint. Therefore the scheduler markers
  and S3 handshake remain unobserved runtime evidence; the next valid step is
  matching-image flash followed by reset-time USART1 and UART2/S3 capture.

## 2026-08-26 handshake stall audit

- `CMD_SYNC_REQ` was decoded in `s3_service_task`, but `RSP_BOOT_INFO` first
  changed `s_host_state` to `HOST_SYNCED` and then reused the ordinary gated
  sender. The response must use a dedicated direct SRP transport path and only
  promote the host state after a successful physical send.
- The UART2 RX DMA/IDLE arm already occurs before `vTaskStartScheduler()` and
  both `DMA1_Stream0_IRQHandler` and `USART2_IRQHandler` are wired. The source
  also clears ORE/NE/PE/FE before each arm; the remaining useful repair is to
  make TX `gState`/error state observable and recover a stale TX-busy state in
  the single-owner TX critical section.
- The physical probe must live in `uart_link_configure_uart()`, not `main()` as
  a global `huart2` call: USART2 uses the private `s_handle` and has no CubeMX
  `MX_USART2_UART_Init()` entry. `main()` calls `uart_link_init()` immediately
  after USART1 setup, before I2C, timers, USART6, and the scheduler.
- The completed Debug ELF contains `AA55DEADBEEF`; its USART2 IRQ first tests
  TXE and writes `0x5A` to TDR before invoking `uart_link_handle_usart_irq()`.
  The direct boot-info sender bypasses only the ordinary pre-sync business gate
  and promotes `HOST_SYNCED` only after `srp_link_send()` succeeds.
- `HAL_UARTEx_RxEventCallback()` calls `uart_link_start_dma_receive()` after
  every RX event, including a zero-length IDLE event and full-buffer transfer
  completion. DMA1 Stream0 and USART2 both use NVIC priority 5, equal to
  `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`, so their ISR notification
  calls are valid under FreeRTOS.
- The initial diagnostic-only `uart2_iso` task was priority 1 with a 256-word
  stack. It takes the existing UART2 TX mutex opportunistically, emits
  `C3 3C A5 5A`, and waits 100 ms. This makes an absent heartbeat evidence about
  scheduler/Tick or UART2 TX, independent of the SRP parser and service task.
  The `E1`-`E7` markers identify core faults, RTOS stack/malloc hooks, or
  Default_Handler.
- The CM7 CMake target produces only the ELF. The `.bin` and `.hex` files in
  the build directory are not build dependencies and were older than the
  freshly linked ELF; they must not be selected for this diagnostic flash.

## 2026-08-26 captured stack overflow repair

- Logic-analyzer capture `0x5A 0xE5` proves USART2 received an event and then
  FreeRTOS executed `vApplicationStackOverflowHook`; it does not identify the
  overflowing task by itself.
- The active CM7 task map has no `chassis_task`: `motor_board` is the chassis
  control executor, while `attitude_gate`, `imu_task`, and `imu_data_logger`
  make up the attitude/telemetry path.
- `s3_service` had a 512-word stack. Its direct BOOT_INFO path calls
  `srp_link_send()`, which previously held two automatic 512-byte SRP frame
  buffers in the non-ACK path. `srp_link_t::tx_scratch` now owns the one
  reusable non-ACK buffer; all S3 senders, including the direct handshake
  response, serialize access through the existing link mutex.
- Stack budgets are now: `s3_service` 1024 words, `srp_uart` 1024,
  `uart2_iso` 512, `scheduler_probe` 512, `attitude_gate` 1024,
  `imu_task`/`imu_data_logger` 1024 each, `motor_board` 1024, and logger 512.
  The dynamic FreeRTOS heap is 48 KiB, increased from 32 KiB to admit these
  stacks without turning the prior E5 failure into malloc-hook E6.

## 2026-08-26 UART2 traffic convergence

- Removed `SMARTCAR_UART2_PHYSICAL_PROBE` and
  `SMARTCAR_UART2_ISOLATION_TEST` options and compile definitions.
  `USART2_IRQHandler()` now dispatches only to
  `uart_link_handle_usart_irq()`; no executable UART2 raw TDR marker or
  startup frame remains.
- Removed the `uart2_iso` task and its `C3 3C A5 5A` heartbeat, plus UART2
  E1-E7 fault-marker injection. Debug CMake sets
  `SMARTCAR_SCHEDULER_PROBE=OFF` and `SMARTCAR_RAW_DIAGNOSTICS=OFF`.
- `s3_service_send_boot_info()` now calls `srp_link_send()` directly while the
  parser callback's outer `s_link_mutex` is held. This preserves serialization
  without recursively taking the non-recursive mutex, while still bypassing
  only the normal pre-sync business gate.
- Final clean CM7 build passed with FLASH `185,448 B`, RAM `80,464 B`, and
  RAM_D2 `512 B`. The CMake cache contains only the two expected OFF switches;
  source searches found no raw UART2 probe/isolator remnants.
