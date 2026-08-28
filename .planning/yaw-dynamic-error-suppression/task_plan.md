# Task Plan: Dynamic Yaw Error Suppression Study

## Goal

Implement and document a source-grounded, tightly scoped reduction of dynamic
yaw error in the existing four-wheel differential chassis. Preserve the local
wheel, kinematics, protocol, and safety contracts while adapting MotorBoard
feedback timing and heading correction dynamics.

## Phases

- [x] Phase 1: Inspect repository rules, architecture, worktree state, and relevant memory.
- [x] Phase 2: Trace current heading, wheel-speed, feedback, and safety ownership.
- [x] Phase 3: Collect and verify GitHub upstream reference implementations.
- [x] Phase 4: Specify compatible control, timestamp, fusion, and compensation models.
- [x] Phase 5: Draft the technical guide and validate its internal references.

## Constraints

- Preserve the existing protocol, GPIO, wheel mapping, kinematics, and safety
  boundaries. Firmware edits are limited to the requested control modules.
- Treat a firmware build as source integration evidence only; do not claim
  vehicle acceptance.
- Use the active CM7 concepts: `err = cur_yaw - target_yaw`, right side is
  `RR/RF`, left side is `LR/LF`, and track width is `193.0 mm`.
- Do not claim the reported gains or 50 ms loop are current source facts until
  independently observed in the active checkout.

## Status

**In progress**: the guide is complete and the requested local control changes
are implemented. Build, static contract, and boundary checks remain part of
the final verification; no hardware or vehicle acceptance is claimed.
