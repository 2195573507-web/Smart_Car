# Findings: Smart_Car IOS_APP and ESP32-S3 Planning

- `DOCS/CODEX_WORKFLOW.md` requires independent implementation, audit, and
  documentation roles with explicit validation boundaries.
- `DOCS/` is the existing documentation root. The requested `IOS_APP/`,
  `ESPS3/`, and lowercase `docs/` paths do not currently exist.
- `DOCS/STM32H757/STM32_ARCHITECTURE.md` records an unresolved physical UART
  route: `PD3`/`PD4` are not a valid STM32 UART TX/RX pair. The new protocol
  therefore cannot claim that link is electrically ready.
- Existing system documentation assigns low-level real-time control to
  STM32H757, gateway responsibilities to ESP32-S3, and future autonomy to
  ROS2_WIN. The new material must preserve those ownership boundaries.
- Multi-agent architecture drafting has been started with independent System
  Architecture and iOS Architecture roles. The remaining roles are sequenced
  because the workspace permits three concurrent delegated agents plus the
  task owner.
- The completed Architecture Agent produced a static-only L1/L3 system plan
  with five resolving local links and no placeholder markers. It preserves the
  unresolved `PD3`/`PD4` STM32 UART route as a hard hardware prerequisite.
- The iOS design draft is present under the existing `DOCS/IOS_APP/` hierarchy.
  It uses SwiftUI/domain/data layering, a single command-bearing transport,
  named L1 manual modes, and L3 supervision only; it does not assume that a
  stop acknowledgement establishes physical braking.
- Protocol, performance, and test role calls were blocked at the agent service
  boundary with HTTP 502 before execution. This is an orchestration failure,
  not a project or document validation result.
- `DOCS` and `docs` resolve to the same directory inode on this macOS
  workspace, so the requested lowercase document root is already represented
  without a second tree. `IOS_APP/` and `ESPS3/` remain empty directories.
- The independent safety/performance/test delegated calls were attempted but
  unavailable at the agent service boundary. Their owner-authored fallback
  documents are explicitly static and should receive a fresh independent audit
  before implementation is authorized.
