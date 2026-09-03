# Rotation Direction Progress

## 2026-09-03

### Phase 1 - Scope and Chain Audit

- **Status:** in_progress
- Read root, Codex, ROS2, architecture, and prior ROS2 work records.
- Confirmed source/build/runtime/physical evidence must remain distinct.
- Confirmed this task must not alter STM32, ESP32, SRP, MotorBoard, or Git history while host-layer diagnosis is underway.
- Next: identify the actual `/cmd_vel` and wheel-control implementation, tests, runtime launch path, and downstream command definitions.
- Enumerated ROS2 packages and searched source-only control terms. The initially broad search included bags/evidence and was not treated as source evidence; the narrowed package list and direct source paths show no ROS2 control output path.
- Next: inspect the state/TF conventions and the actual downstream MotorBoard/STM32 command mapping with focused source searches.
- Resumed with the existing isolated plan. Verified the user-owned `smartcar-mapping-session` is still running and recorded the current dirty-worktree state; no existing files were reverted or altered.
- Runtime snapshot: the launch has live telemetry/odom/TF parameters enabled, but `/cmd_vel` is absent and therefore has no publisher or subscriber. `/odom` has exactly one publisher (`s3_ydlidar_bridge`) but no sample was received during an 8-second bounded inspection. `odom -> base_link` is absent for the same reason. `base_link -> laser_frame` resolves to translation `(0.200, 0.000, 0.155)` and zero rotation.
- Source conclusion: this workspace's state bridge uses frozen `RR, RF, LR, LF` ordering and `angular = (right - left) / track_width`; its default track width remains 0.193 m. There is no ROS2 command-to-wheel-output code to repair. The STM32 source checkout contains only motor/control scaffolds, so the MotorBoard channel and physical polarity cannot be source-audited here.
- Added a source-only `WheelKinematics` regression test covering stationary positive/negative yaw, forward straight, and forward/reverse left/right arcs. It verifies the configured 0.193 m feedback kinematics only; it does not imply a motor command path exists.
- Replaced the table-loop test with seven named parameterized cases so test output directly identifies positive yaw/left, negative yaw/right, forward straight, forward left/right, and reverse left/right.
- Verification complete: a fresh 4-package `colcon build --cmake-force-configure --symlink-install` passed. Final `colcon test` and `colcon test-result --verbose` reported `117 tests, 0 errors, 0 failures, 0 skipped`; `RosYawSign/RosYawMotion` ran all seven named cases successfully. `git diff --check` passed for tracked work. The image lacks `clang-format` and any discovered ament formatter/lint package, so no mutating tool installation was attempted.
- Final read-only diagnostics: `/cmd_vel` remains absent. `/odom` has one bridge publisher but no current sample; `/tf_static` has the expected robot_state_publisher source and the static `base_link -> laser_frame` transform resolves with zero RPY. The bridge reports accepted historic chassis frames but current `session_closed`/`odom_invalid`, so no live odom/TF direction comparison can be asserted. No container, protocol, firmware, safety gate, calibration value, or physical wiring was changed.
