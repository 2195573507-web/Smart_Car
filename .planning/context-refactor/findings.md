# Smart_Car AI Context Refactor Findings

## Workspace Facts

- The root worktree is already heavily dirty and contains tracked changes,
  untracked source trees, generated output, planning files, and `.DS_Store`
  artifacts that predate this task.
- The workspace contains multiple nested Git repositories, including
  `S3-radartest` and nested copies below it.
- Initial inventory found 1,131 Markdown/README-like paths when build, cache,
  extracted, nested, and third-party trees were included.
- Of those, 855 are under `S3-radartest`, 119 under `.analysis_extract`, 46
  under `DOCS`, 40 under `资料`, 40 under `STM32H757`, and 16 under `ESPS3`.
- `DOCS/` and `docs/` resolve to the same directory on this case-insensitive
  filesystem. Canonical links can use `docs/` without creating a duplicate.
- `DOCS/.DS_Store` is non-Markdown and cannot be changed or removed in this
  task, so the directory itself will not be case-renamed.

## Documentation Risks

- `DOCS/SMARTCAR_PROTOCOL_V1.md` describes the implemented AA/01/.../55
  CRC16-MODBUS contract.
- `DOCS/SMART_CAR_PROTOCOL.md` is explicitly deprecated but still contains a
  competing A5/5A and CRC-CCITT proposal.
- `DOCS/SMARTCAR_PROTOCOL_V2.md` is a future placeholder and must not be read as
  implemented behavior.
- Several current-looking architecture and planning documents date from before
  the active STM32-S3-BLE integration and need historical or planned labels.
- Root planning files belong to an earlier architecture task and must not be
  overwritten; this task uses `.planning/context-refactor/`.

## Current Architecture Evidence

- STM32H757 owns deterministic sensor, actuator, safety, and final motion
  authority.
- ESP32-S3 owns the STM32 UART gateway, BLE endpoint, and radar integration.
- The current macOS SwiftUI app is `IOS_APP/SmartCar_Control_MAC`; it is not an
  iOS migration target.
- ROS2, SLAM, and autonomous navigation remain planned future work.
- Current protocol framing is `AA | 01 | TYPE | LEN_LE | PAYLOAD | CRC16-MODBUS_LE | 55`.
- LSM303 is the active IMU path; BMI323 is retained but paused.
- Hardware/runtime/integration status must not be inferred from source or build
  evidence alone.

## Audit Classification Policy

- `ACTIVE/HIGH`: current canonical Smart_Car source-of-truth documents.
- `ACTIVE/MEDIUM`: current module-local guides and tool documentation.
- `MISSING_UPDATE/MEDIUM`: relevant project-owned material needing current
  status or link updates.
- `OUTDATED/LOW`: superseded plans or contracts that conflict with current
  implementation.
- `DUPLICATE/LOW`: copied or generated duplicate material.
- `ARCHIVE/LOW`: historical records, extracted snapshots, vendor, third-party,
  or nested-project material that is not Smart_Car authority.

## Evidence Boundary

This task performs static source and documentation inspection only. It does not
compile, flash, monitor, connect BLE/UART, run the app, operate a vehicle, or
validate physical hardware.
