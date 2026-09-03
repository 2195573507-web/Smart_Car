# ROS2 Rotation Direction Diagnosis

## Goal

Identify the first evidence-backed source of reversed physical rotation in the ROS2-to-MotorBoard chain, apply a minimal ROS2-only correction only when that source is on the host, and distinguish host proof from hardware proof.

## Current Phase

Phase 4 - verification of the source-only conclusion.

## Phases

### Phase 1: Scope and Chain Audit

- [x] Read workspace and ROS2 ownership documentation.
- [x] Trace `/cmd_vel` through ROS2 control, kinematics, wheel order, and gateway interfaces.
- [x] Record source-only evidence and protected boundaries.
- **Status:** complete

### Phase 2: Root-Cause Decision

- [x] Compare ROS positive yaw and REP-103 expectations against current transformations.
- [x] Examine MotorBoard channel and physical-direction definitions without modifying them.
- [x] Choose the nearest confirmed faulty boundary, or stop with firmware/wiring evidence.
- **Status:** complete

### Phase 3: Minimal Host Correction

- [x] Determine that no ROS2 host correction is justified because this workspace has no command-output implementation.
- [x] Add focused read-only kinematic regression coverage for pure rotation, translation, and combined forward/reverse turns.
- **Status:** complete

### Phase 4: Verification

- [x] Run formatting/static checks, fresh colcon build, and all relevant unit tests.
- [x] Inspect running `/cmd_vel`, `/odom`, and `/tf` direction consistency without altering safety gates.
- [x] State remaining physical-wheel validation explicitly.
- **Status:** complete

## Decisions

| Decision | Rationale |
| --- | --- |
| Preserve wheel order `[M1: RR, M2: RF, M3: LR, M4: LF]`, `193.0 mm`, and `WHEEL_TRIM` unless source evidence proves their use is wrong. | They are explicit physical/calibration contracts, not a generic sign correction. |
| Do not modify STM32, ESP32, SRP, or MotorBoard code during host diagnosis. | STM32 owns final actuation and safety; a host compensation would mask a downstream fault. |
| Treat source, build, running ROS graph, and physical wheel behavior as separate evidence levels. | Topic or test results cannot prove wiring or motor polarity. |

## Errors

| Error | Attempt | Resolution |
| --- | --- | --- |
| Tool invocation initially reported an invalid working directory while `D:\Smart_Car` was readable. | 1 | Continued with explicit absolute paths; no repository operation failed. |
