# AI Context Refactor Report

## 1. Added Files

The new authoritative context files are listed in
[DOCUMENT_INDEX.md](DOCUMENT_INDEX.md), [MODULE_INDEX.md](MODULE_INDEX.md), and
the detailed audit. They include `.codex/*`, `PROJECT_STATUS.md`,
`DECISION_LOG.md`, architecture/protocol/module pages, indexes, changelog, and
this report.

## 2. Moved Files

The A5/5A protocol plan, the former unified V1 table, and Protocol V2 planning
table were moved to `docs/history/protocol/`; short compatibility pages remain
at their prior paths to preserve old links. The prior STM32 root README is also
retained as a dated baseline under `docs/history/`, while
`STM32H757/README.md` is now the current module entry. Other nested,
module-local, extracted, vendor, and third-party documents are intentionally
not moved.

## 3. Merged or Superseded Content

- Current architecture facts are consolidated in `docs/architecture/`.
- App BLE and STM32-S3 frame contracts are separated in `docs/protocol/`.
- Old A5/5A and future V2 proposals are historical/planned, not current wire
  truth.
- Existing module README files remain local references and are indexed with
  trust/status labels.

## 4. Core Knowledge Structure

`BOOT -> MEMORY/RULES/INDEX -> PROJECT_STATUS -> affected module -> source ->
verification` is the recommended Codex path. Stable facts, dated status,
decisions, canonical engineering pages, local references, and history now have
different authority levels.

## 5. Recommended Codex Startup

Read `.codex/BOOT.md`, `.codex/MEMORY.md`, `.codex/RULES.md`, and
`.codex/INDEX.md`; then read `PROJECT_STATUS.md`, the affected module page, and
the exact code-map entry. State evidence level and protected boundaries before
editing.

## 6. Findings

- The workspace contains multiple nested repositories and more than one
  documentation ownership boundary.
- The App BLE frame and STM32-S3 source frame are not byte-identical.
- S3 BLE RX callback registration and telemetry bridging are not proven in the
  current source; `imu_bridge_handle()` is empty.
- Current generated STM USART2 uses PA2/PA3 while legacy IOC labels PD3/PD4;
  this remains a human-confirmation item.
- Hardware, runtime, and end-to-end acceptance remain unverified.

## 7. Human Confirmation Needed

See [PROJECT_STATUS.md](PROJECT_STATUS.md) for the complete list. The highest
priority decisions are the approved cross-boundary protocol, physical UART
route, App command admission, telemetry relay, and hardware acceptance plan.

## Scope and Evidence

This report describes Markdown-only work. No build, flash, monitor, server,
BLE session, UART capture, sensor capture, radar test, or vehicle test was run.

## Final Documentation Checkpoint

The final audit inventory contains 1,184 Markdown/README-like files, including
nested, extracted, vendor, and third-party material. Every row carries a path,
name, purpose, owner, status, trust level, and handling recommendation. The
additional historical STM32 baseline and four nested `.markdown` files are
explicitly classified rather than silently omitted.

## Verification Evidence

| Check | Result |
| --- | --- |
| Audit inventory versus audit rows | 1,184 paths each; 0 missing and 0 stale rows |
| Required knowledge-base files | 0 missing |
| Core-module documentation template | 19 modules checked; 0 missing required section groups |
| Local Markdown links | 93 knowledge-base documents and 222 local links checked; 0 broken targets |
| Mermaid architecture diagrams | 7 diagrams; 16 paired fence lines; 0 unbalanced files |
| Canonical Markdown hygiene | 0 trailing-whitespace and 0 unresolved-placeholder matches |
| Executable validation | Not run by design; no executable source or configuration changed |

The root worktree still contains unrelated pre-existing source, generated, and
non-Markdown changes. They were neither reverted nor modified by this
documentation task.

## Requirement Coverage

| Requested phase | Delivered evidence |
| --- | --- |
| 1. Workspace document scan | `DOCUMENT_AUDIT_REPORT.md` classifies every inventory path with the required fields. |
| 2. AI context system | `.codex/BOOT.md`, `MEMORY.md`, `RULES.md`, `WORKFLOW.md`, and `INDEX.md`. |
| 3. Project status database | `PROJECT_STATUS.md` separates source-established, in-progress, planned, paused, and deprecated work. |
| 4. Decision database | `DECISION_LOG.md` records hardware, UART, BLE, protocol, IMU, radar, ROS2, and architecture choices. |
| 5. Canonical documentation structure | `docs/` contains architecture, hardware, module, protocol, history, and navigation layers. |
| 6. Architecture documentation | `docs/architecture/system.md`, `communication.md`, and `data_flow.md`, including four system diagrams. |
| 7. Core module documentation | Nineteen consistently structured pages plus the STM32 local entry README. |
| 8. Code map | `docs/code_map.md` maps source roots, entries, interfaces, tasks, and outputs. |
| 9. Documentation normalization | Canonical links resolve; conflicting protocol plans are historical; current contracts remain separated. |
| 10. Final indexes | `DOCUMENT_INDEX.md`, `MODULE_INDEX.md`, and the complete audit inventory. |
| 11. Changelog | `CHANGELOG.md` records this knowledge-base refactor. |
