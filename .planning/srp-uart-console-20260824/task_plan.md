# Task Plan: S3 Console and SRP UART2 Handshake

## Goal

Release GPIO43/44 from the ESP32-S3 UART0 console, make USB Serial/JTAG the
primary console, and add bounded diagnostics that distinguish S3 UART2 TX/RX,
STM32 USART2 DMA reception, and SRP parsing/response failures.

## Phases

- [x] Phase 1: Inspect repository rules, active source, configuration, and dirty-file boundaries.
- [x] Phase 2: Record confirmed findings and implement the minimal configuration/diagnostic patch.
- [x] Phase 3: Regenerate/check ESP-IDF configuration and build S3 plus STM32 CM7 targets.
- [x] Phase 4: Capture the required flashed UART/GPIO evidence and close the hardware boundary.
- [x] Phase 5: Record the USB-console port-selection failure and close the diagnosis.
- [x] Phase 6: Close the CM7 RX arming race, wake error recovery, and add a
  combined STM SRP/UART boundary diagnostic line.
- [x] Phase 7: Close the CM7 RX arming race, verify ISR routing/recovery, and
  complete the clean CM7/host verification pass.
- [x] Phase 8: Make the D2 DMA region explicitly non-cacheable and add compact
  runtime proof for USART/DMA IRQ entry, Rx callback rejects, DMAMUX requests,
  and RX NDTR.

## Scope

- `ESPS3/sdkconfig.defaults`, generated `ESPS3/sdkconfig` state if needed,
  `ESPS3/components/stm_uart/*`, and the S3 service diagnostics.
- `STM32H757/Middleware/Communication/UART_Link/*` and STM32 SRP service
  diagnostics only.

## Protected Boundaries

- Preserve UART2 GPIO17/18 and USART2 PA2/PA3 routing.
- Preserve radar UART1 RX GPIO44 and PWM GPIO4 ownership.
- Do not claim physical handshake, GPIO release, or wire integrity from a build.

## Verification Boundary

- Source and isolated-build checks are complete.
- Flashing completed for the current image and the native USB Serial/JTAG
  monitor captured the live bootloader/app/UART2 logs. This proves the S3
  console path and UART2 initialization on hardware; STM32-side RX and GPIO
  electrical handshake remain separate acceptance boundaries.
- The CM7 Debug build now proves the RX stream is marked active before HAL
  enables IDLE/DMA interrupts, and UART errors notify the recovery worker.
  Runtime acceptance still requires a matching flashed CM7 image and STM-side
  `SRP_S3`/`SRP_UART2_DIAG` capture.

## Phase 8 decision

- Keep the frozen PA2/PA3 route and manual UART-link DMA ownership. The HAL
  computes `DMAmuxChannel` and programs `Init.Request` in `HAL_DMA_Init()`, so
  linking `hdmarx`/`hdmatx` after each init is valid.
- Overlay Region 1 on the existing 4 GB default-deny MPU region for the first
  32 KB of D2 SRAM, covering all current `.dma_buffer` objects. Keep explicit
  cache maintenance as a defensive measure.
- Record only scalar register snapshots in interrupt context; full recovery
  remains in the UART task. Logs stay compact because the transport log
  payload is 96 bytes.

## Phase 8 result

- Region 1 now covers `0x30000000..0x30007FFF` as full-access,
  execute-never, shareable, non-cacheable, non-bufferable memory.
- DMAMUX request IDs are checked against 43/44 during every DMA configuration;
  a mismatch leaves the transport unready and emits an error.
- `SRP_UART2_HW` records DMA RX/TX IRQs, USART IRQs, callback rejects, NDTR,
  request IDs, DMA error code, and a packed PA3 GPIO state. No ISR logging or
  blocking recovery was added.

## Phase 7 result

- RX is marked active before `HAL_UARTEx_ReceiveToIdle_DMA()` enables UART/DMA
  interrupt sources, and the callback immediately re-arms the next normal-mode
  transaction after copying bytes to the SRP ring.
- USART2/DMA error flags are cleared in the callback; full HAL abort/deinit and
  restart remain owned by the UART worker task.
- Clean CM7 Debug build, host SRP codec tests, `git diff --check`, ELF ISR
  symbols, and the D2 `.dma_buffer` placement all passed. Hardware acceptance
  still requires flashing this matching CM7 image and capturing live counters.
