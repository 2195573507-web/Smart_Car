# Codex Rules

## Change Sequence

1. Read `.codex/BOOT.md`, `MEMORY.md`, `RULES.md`, and `INDEX.md`.
2. Read `PROJECT_STATUS.md` and the affected module document.
3. Inspect current source and the exact ownership boundary.
4. State allowed files, protected files, evidence level, and exclusions.
5. Make the smallest authorized change.
6. Run proportionate static/build tests without upgrading their meaning.
7. Report modified files, evidence, risks, and unverified items.

## General Code Rules

- Preserve public interfaces and behavior unless the task explicitly changes
  the contract.
- Do not perform unrelated refactors, retries, watchdog masking, or blanket
  stack/heap changes.
- Keep ownership in the existing layer; repair lifecycle/admission causes.
- Treat source as current implementation truth and old plans as history.

## STM32 Rules

Check RTOS task context, stack, interrupt/DMA ownership, UART, memory, and
generated-code boundaries. Never block an ISR or hide a fatal initialization
failure. Keep BSP/HAL handles private to their owning layer. Do not repurpose
SWD or frozen nets.

## ESP32-S3 Rules

Keep raw transport tasks separate from parsers and gateway services. Check
FreeRTOS task lifetime, BLE readiness/CCC state, UART buffers, queue ownership,
and memory. Radar UART1/GPIO44 and PWM GPIO4 are separate from STM UART2.

## Protocol Rules

- Treat App BLE and STM32-S3 envelopes as separate contracts until an explicit
  bridge is implemented and verified.
- Any protocol change must update all participating implementations and the
  canonical protocol documents together.
- Use explicit field serialization; never rely on C/Swift structure padding.

## Logging Rules

Every new event must identify module, state, and error reason. Keep logs
diagnostic and bounded. A log line, host parser, or build symbol does not prove
physical transport.

## Documentation Rules

Mark claims `CONFIRMED`, `PLANNED`, `RESERVED`, `PAUSED`, `DEPRECATED`, or
`UNVERIFIED`. Do not erase contradictory history; link it and explain the
precedence decision.

## Git Modification Rules

1. Check `git status` before editing and before staging.
2. Use one coherent feature, repair, or documentation change per commit.
3. Do not commit generated output, dependency caches, transient logs, or local
   IDE state; verify uncertain files with `git check-ignore`.
4. Create a branch before a substantial or risky change.
5. Do not create a nested Git repository, add a submodule, or rewrite history
   without explicit authorization.
