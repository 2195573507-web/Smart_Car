# CM7 Raw USART1 Stall Diagnosis

## Goal

Produce a CM7 debug image that exposes scheduler, task-entry/loop, FreeRTOS
fault, and UART2 TX-lock progress through a dependency-free USART1 path.

## Phases

- [x] Audit current UART, task, and fault-record ownership.
- [x] Add bounded raw USART1 diagnostic module.
- [x] Instrument startup, task boundaries, fault hooks, and UART2 TX.
- [x] Build CM7 and run static checks.
- [x] Record hardware capture procedure and evidence limits.

## Safety Boundaries

- Preserve SRP v4 framing and UART2 PA2/PA3 routing.
- Preserve existing motor force-stop and retained fault capture.
- Keep raw output bounded so diagnostics do not monopolize USART1.

## Verification

- `SMARTCAR_RAW_DIAGNOSTICS=ON` is compiled into the canonical Debug probe
  image; the default CMake option remains OFF outside that preset.
- The current CM7 ELF links the raw module and has no undefined symbols.
- Raw output remains unverified until the matching image is flashed and
  USART1 is captured. A build or a string table does not prove a task entered,
  UART2 TX completed, or an exception reached `Default_Handler`.
