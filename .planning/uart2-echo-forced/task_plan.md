# UART2 Forced Echo Firmware

## Goal

Produce STM32H757 CM7 and ESP32-S3 firmware images that force a 115200-8-N-1
UART2 physical-link Echo test and exclude SRP, DMA transport, and application
services from the test targets.

## Phases

- [x] Audit the existing Echo branch, pin ownership, and target dependencies.
- [x] Force test configuration and reduce both target source/dependency graphs.
- [x] Build both images in new isolated directories.
- [x] Audit final symbols and document flash and bench acceptance.

## Boundaries

- Do not flash hardware in this task.
- Preserve production protocol sources and unrelated dirty-worktree changes.
- Source/build evidence does not establish physical wiring or signal integrity.
