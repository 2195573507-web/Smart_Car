# Task Plan: Restore and Align SmartCar_Control_MAC

## Goal

Restore the original macOS control app at `SmartCar_Control_MAC/`, then port
the current iOS control/protocol additions without restructuring either UI.

## Phases

- [x] Restore historical macOS package into the new top-level directory.
- [x] Build restored baseline and record any environment limitation.
- [x] Port iOS protocol, BLE queue, mode, speed, and telemetry additions.
- [ ] Build both packages and run static/protocol checks.
- [ ] Review diff boundary and report hardware-validation gaps.

## Constraints

- Preserve unrelated dirty worktree changes.
- Keep `0x110` and `0x114` contracts unchanged.
- Use safe little-endian Float32 conversion and exact payload lengths.
- Do not restore the deleted generic `IOS_APP/` tree.

## Status

Final build and static verification are the current phase.
