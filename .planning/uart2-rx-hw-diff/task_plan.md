# USART2 RX Hardware Diff and Repair

## Objective

Compare CM7 USART2 receive hardware and lifecycle configuration between commit
`a250dec` (branch `1`) and the current worktree. Repair only a source-proven
current configuration defect that can prevent RX interrupts or DMA transfers.

## Scope

- In scope: CM7 USART2 MSP, DMA/NVIC, clock/MPU/cache setup, receive startup,
  IRQ handlers, and UART link callback path.
- Protected: pin/IOC allocations, SRP wire framing, ESP32 firmware, unrelated
  dirty worktree changes, and physical-wire conclusions.

## Phases

| Phase | Status | Evidence |
| --- | --- | --- |
| Baseline and repository rules | complete | Current and historical refs resolved; docs read |
| Extract line-level old/current configuration | in_progress | Pending source and preprocessed/build evidence |
| Diagnose RX/DMA blocker and apply minimal repair | pending | Must be source-proven |
| Build and static verification | pending | CM7 Debug build plus targeted checks |
| Report layered evidence and residual hardware checks | pending | Final review |

## Errors

| Error | Attempts | Resolution |
| --- | --- | --- |
| None | 0 | N/A |
