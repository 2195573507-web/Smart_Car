# Progress

## 2026-08-25

- Design approved by user.
- Context audit complete; implementation in progress.

## 2026-08-26 completion

- Added `cm7_raw_diag.c/.h` with direct USART1 markers, bounded counters,
  FreeRTOS assert/fault markers, and a diagnostic `Default_Handler` path.
- Instrumented scheduler admission/start/return, probe task delay boundaries,
  SVC/PendSV/SysTick counters, RTOS failure hooks, and UART2 TX mutex/HAL
  phases.
- Built the canonical CM7 Debug probe with raw diagnostics enabled and ran
  strict static checks; the current ELF links without undefined symbols.
- Hardware capture, matching image provenance, and live UART/S3 evidence are
  still required before using any marker to classify the runtime stall.

## 2026-08-26 current continuation

- Rebuilt the canonical CM7 Debug image from a clean target and refreshed the
  ELF/BIN/HEX artifacts; the raw diagnostic module and startup assembly both
  compiled successfully with no undefined ELF symbols.
- Confirmed that the current machine has no STM32 probe or target serial port,
  so raw USART1 markers, Default_Handler output, and UART2 TX phases remain
  pending live capture.
