# Rotation Direction Findings

## Constraints

- Diagnose `/cmd_vel -> ROS2 control -> kinematics -> wheel targets -> S3/STM32/MotorBoard -> wheels` in order.
- Positive ROS `angular.z` must be checked against REP-103 left/CCW convention.
- Maintain wheel order `[M1: RR, M2: RF, M3: LR, M4: LF]`, `193.0 mm`, and `WHEEL_TRIM` unless evidence proves otherwise.
- Keep existing protocol, safety, emergency-stop, and command-admission boundaries.
- No Git commit and no firmware/protocol modification without evidence that the problem is not host-side.

## Initial Evidence

- No `AGENTS.md` is present below `D:\Smart_Car`; root `README.md` instead requires `.codex/BOOT.md`, `MEMORY.md`, `RULES.md`, module docs, source inspection, and evidence separation before a change.
- `.codex` documentation assigns final motion authority and safety to STM32, with ESP32-S3 as gateway.
- The ROS2 workspace has substantial pre-existing dirty work that must remain intact.
- Source enumeration found four ROS2 packages: `s3_ydlidar_bridge`, `smartcar_state_bridge`, `smartcar_description`, and `smartcar_bringup`. No source in these packages subscribes to `/cmd_vel`, transforms `geometry_msgs/msg/Twist` into wheel targets, or serializes a motor command toward S3/STM32/MotorBoard. The bridge is telemetry/scan/odom-only.
- The current ROS2 state path publishes received/integrated odometry; it is not a command/control path. Its `odom_message.cpp` copies the already-computed `angular_z_rps` to `nav_msgs/msg/Odometry` without sign inversion.
- The active `smartcar-mapping-session` container is an existing user runtime (`srp_interleave_0831-ros2-dev`, TCP 8765) and must be inspected without restart, stop, parameter changes, or command publication. `git status --short` confirms many pre-existing workspace changes; they are out of scope and remain untouched.

## Open Questions

## Verified Conclusion

- **Classification:** other/downstream unresolved; it is not a confirmed ROS2 command-sign, TF, or host-kinematics defect in this checkout. The actual command source and the target conversion path are absent from the ROS2 source tree and from the inspected runtime graph.
- **ROS convention:** positive `angular.z` is positive yaw about `+Z`, hence counter-clockwise/left when viewed from above under REP-103. The host feedback kinematics preserve this convention: right-side speed minus left-side speed divided by positive track width. Positive yaw therefore requires the right side to advance faster than the left side.
- **Wheel contract:** `[M1: RR, M2: RF, M3: LR, M4: LF]` remains unchanged. `track_width_m` remains `0.193` (193 mm); no `WHEEL_TRIM` definition occurs in the inspected ROS2/STM32 source, and none was introduced or changed.
- **TF/odom source:** `odom_message.cpp` and `state_node.cpp` pass yaw and `angular_z_rps` through directly, and use the same orientation for `odom -> base_link`. URDF and live `tf2_echo` resolve `base_link -> laser_frame` as `(0.200, 0.000, 0.155)`, zero RPY. The origin is explicitly provisional and must be physically measured before declaring sensor-frame acceptance.
- **Runtime limit:** the active launch has no `/cmd_vel`; `/odom` is published by `s3_ydlidar_bridge`, but no bounded `/odom` sample arrived during inspection and `odom -> base_link` was unavailable. The final diagnostic sample showed a closed chassis session (`odom_invalid=true`), so runtime ROS data cannot establish physical turning direction.
- **Downstream limit:** STM32 has placeholder motor/control boundaries and no checked-in MotorBoard implementation/decoder/channel mapping. Therefore no source evidence can distinguish firmware sign/mapping from motor-polarity/wiring error. A host-side sign flip would conceal rather than repair either fault.

## Verification Evidence

- `colcon build --cmake-force-configure --symlink-install`: 4 packages passed.
- Final `colcon test` and `colcon test-result --verbose`: 117 tests, 0 errors, 0 failures, 0 skipped.
- New parameterized `RosYawSign` test prints and passes: stationary positive yaw left (`+1.0 rad/s`), stationary negative yaw right (`-1.0 rad/s`), forward straight, forward left/right, and reverse left/right.
- Static formatting: repository has no configured formatter/lint target and the active image has neither `clang-format` nor an ament formatting package. `git diff --check` passed for tracked work; the new test source was also compiled and exercised successfully. No formatter was installed because that would mutate the user-owned runtime environment.

## Required Physical Follow-up

1. Identify and snapshot the external `/cmd_vel` publisher and the code that turns it into the frozen `M1..M4` targets.
2. With wheels safely off the ground and emergency-stop supervision, capture target values and actual wheel rotation for pure `angular.z = +w`, pure `angular.z = -w`, forward/reverse left, forward/reverse right, and straight translation.
3. At STM32/MotorBoard, correlate each target channel to `RR/RF/LR/LF`, then verify each forward command maps to forward chassis motion. Repair the first failing mapping or physical polarity there; do not add a ROS2 compensation.
