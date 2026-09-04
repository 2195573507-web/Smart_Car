# Mapping Console Design

## Goal

Replace the desktop scan-viewer shortcut with a one-click mapping console for
the ROS2_WIN workspace. The console starts the real mapping stack, opens RViz
in `map`, saves maps, resets the active SLAM session, and returns the bridge to
its false-gated safe state when mapping stops.

## Scope And Boundaries

- Only `ROS2_WIN` files and the existing desktop shortcut are changed.
- The mapping stack uses the current TCP bridge, `robot_state_publisher`,
  `slam_toolbox`, and the existing `p1_mapping.rviz` configuration.
- The console does not publish `/cmd_vel`, open a serial port, change STM/S3
  firmware, or alter S3RD/SRP protocol parameters.
- Map clearing never touches `bags/`, source code, bridge configuration, or
  files outside `ROS2_WIN/maps/`.

## Operator Flow

1. Double-click **Smart Car Mapping** on the desktop.
2. Enter measured laser XYZ/RPY values and select **Measured laser extrinsics
   confirmed**. The checkbox is clear by default; without it, **Start
   Mapping** refuses to launch. The console then stops only the current TCP-8765
   bridge after confirmation, builds the mounted ROS workspace if needed, and
   starts one named `smartcar-mapping-session` container with live odom/TF
   gates enabled only for the mapping session.
3. RViz starts with Fixed Frame `map` and its `/scan`, `/map`, TF, and model
   displays enabled. The console validates the expected ROS nodes/topics.
   The default desktop flow waits up to 60 seconds for live diagnostics,
   `/scan`, `/odom`, and `odom -> base_link` TF. If they are incomplete, it
   shows `实时里程计/TF未就绪，暂不能建图` and leaves the full mapping
   container running so RViz and SLAM remain available for diagnosis.
4. Select **Save Map + Pose Graph** to save a timestamped `.pgm`/`.yaml` map
   pair and the `.posegraph`/`.data` `slam_toolbox` graph under `maps/`.
   The mapping launch records `/scan`, `/odom`, TF, `/map`, and diagnostics to
   a timestamped rosbag under `bags/`; stopping the session finalizes it.
5. Select **Clear Saved Maps And Reset SLAM** to confirm, stop the active
   mapping session, move map artifacts (`.pgm`, `.yaml`, `.posegraph`,
   `.data`) from `maps/` to the Windows Recycle Bin, then start a fresh SLAM
   session with no serialized posegraph.
6. Select **Stop And Restore Safe Mode** to stop the mapping session and
   start exactly one false-gated bridge container on TCP 8765.

## Failure Handling

- Docker, VcXsrv, build, start, and ROS graph failures are shown in the
  console status area and leave no second TCP-8765 owner running.
- A live-data timeout is a non-fatal warning by default. It does not stop the
  TCP bridge or remove `smartcar-mapping-session`. Pass `-StrictLiveGate` to
  the console or worker only when the original fail-closed timeout behavior is
  explicitly required.
- Explicit **Stop And Restore Safe Mode**, **Clear Saved Maps And Reset SLAM**,
  Ctrl+C, or a genuine startup/container failure may perform cleanup.
- The console checks container ownership before stopping any port-8765 bridge
  and asks for confirmation before the replacement.
- Map clearing is recoverable through the Windows Recycle Bin. If an artifact
  cannot be moved, the reset aborts and reports the path.
- Closing the console calls the safe-stop path when the console owns the
  mapping session.

## Verification

- PowerShell syntax parsing must pass for the console launcher.
- The desktop shortcut must target the mapping-console script.
- The safe mode must show only `/scan` and `/diagnostics`, with all four live
  telemetry/odom/TF gates false.
- Live mapping remains evidence-gated: actual `/scan`, `/odom`, TF, and map
  messages are required independently before declaring a physical mapping run
  healthy.
