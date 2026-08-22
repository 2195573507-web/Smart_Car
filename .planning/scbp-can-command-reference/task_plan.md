# Task Plan: STM32-S3 SCBP-CAN Command Reference

## Goal

Create a source-based reference document that is the required starting point
for future STM32-S3 UART command additions.

## Phases

- [x] Read repository rules, canonical protocol docs, shared SCBP-CAN code, and active endpoint call sites.
- [x] Write the command registry, frame/payload contracts, calibration flow, extension rules, and evidence boundaries.
- [x] Run document and source consistency checks.
- [x] Deliver the reference path and verification result.

## Scope

Documentation-only change. Preserve all existing firmware and unrelated dirty
worktree changes. Modified documentation paths:

- `DOCS/protocol/stm32-s3-command-reference.md`
- `DOCS/protocol/protocol.md`
- `DOCUMENT_INDEX.md`

## Evidence Boundary

The document records source-confirmed contracts. No firmware build, flash,
UART capture, BLE capture, sensor test, radar/PWM test, or vehicle acceptance
is implied by this task.

## Verification Result

- `git diff --check`: passed for the documentation change.
- Relative Markdown links in the changed documents: passed.
- Active message IDs and source constants cross-checked: passed.
- Shared SCBP-CAN host test: `SCBP-CAN host tests passed`.
- Existing dirty firmware/motor-board files were preserved and not edited.
