# Mapping Console Verification

Date: 2026-09-01

## Passed

- `docker/open_mapping_console.ps1` and `docker/mapping_session.ps1` pass the
  PowerShell parser.
- `C:\Users\至亲\Desktop\Smart Car Mapping.lnk` and the retained
  `C:\Users\至亲\Desktop\Smart Car RViz2.lnk` both target
  `D:\Smart_Car\ROS2_WIN\docker\open_mapping_console.ps1` with
  `ExecutionPolicy Bypass`.
- A GUI smoke test opened a responsive `Smart Car Mapping` window. The test
  did not start mapping and closed only its own process.
- Clear/reset moved the actual
  `maps/slam_live_20260901.pgm` and `maps/slam_live_20260901.yaml` files to
  the Windows Recycle Bin. `maps/` now contains only `.gitkeep`.
- The action did not touch `bags/`: seven files totaling 29,066,820 bytes
  remained after cleanup.
- Final safe state has exactly one TCP-8765 owner,
  `smartcar-mapping-safe`, and false `allow_live_telemetry`,
  `enable_live_odom`, `publish_odom`, and `publish_tf` parameters. Its ROS
  graph contains `/scan` and `/diagnostics` only.
- A separate Start failure-path test with the physical S3 stream unavailable
  exited non-zero after the 60-second absolute gate, removed
  `smartcar-mapping-session`, and restored the same false-gated safe bridge.

## Not Passed In This Reset

- The fresh real mapping session did not pass its live gate because the S3
  diagnostics did not provide connected, non-stale scan and odom data. The
  session was not declared usable and was stopped; the false-gated safe bridge
  was restored.

## Evidence

- Console action log: `docker/mapping_console.log`
- Console final status: `docker/mapping_console.status`
- Console action log includes the clear/reset and failed-start recovery records:
  `docker/mapping_console.log`
- Operator design and behavior: `docs/mapping-console-design.md`
- Original successful real-SLAM artifacts: `bags/slam_live_20260901/` and the
  Windows Recycle Bin entries for `slam_live_20260901.pgm/.yaml`

## 2026-09-02 regression fix

- Reproduced the missing-container probe failure under Windows PowerShell 5.1:
  `docker inspect smartcar-mapping-session` exits non-zero when the worker has
  already removed the failed mapping container, and strict native stderr
  handling raised the .NET popup during the GUI refresh.
- `open_mapping_console.ps1` now treats that expected absence as a non-running
  mapping session. The worker's existing failure status and safe-bridge
  recovery are unchanged.
- Parser validation passed, and a direct extracted-function probe returned
  `False` with no mapping container while preserving the outer `Stop` setting.

## 2026-09-02 popup hardening follow-up

- Replaced the GUI's missing-container `docker inspect` call with a successful
  empty-result `docker ps` running-name filter, eliminating the Docker
  `no such object` error path.
- Wrapped control refresh, timer status updates, and form-closing probes so
  transient native-command or status-file failures stay in the status label
  instead of becoming an unhandled .NET dialog.
- Revalidated both script parsers, the absent-container probe, Docker's empty
  result behavior, and a GUI open/close smoke test.

## 2026-09-02 stale-process diagnosis

- The same dialog persisted because an older `Smart Car Mapping` process had
  loaded the pre-fix script before the file was edited. The stale process was
  identified by its exact command line and window title, then closed without
  stopping the safe bridge.
- The worker script no longer contains any `docker inspect` call either; all
  console/worker state checks use `docker ps` filters that return empty output
  for an absent container.
- A fresh process from the current script remained responsive for five seconds
  and then closed with exit code 0 without showing the .NET dialog.

## 2026-09-03 non-fatal live-gate timeout

- `mapping_session.ps1` now treats the 60-second live-data timeout as a warning
  in the default mode. It leaves `smartcar-mapping-session`, TCP 8765, RViz,
  SLAM, the scan bridge, and robot description running.
- The warning is `实时里程计/TF未就绪，暂不能建图`. No `/odom`, TF, `/scan`, or
  map data is synthesized.
- `-StrictLiveGate` remains available for an explicit fail-closed run; only
  that mode restores timeout cleanup. Explicit Stop/Clear and genuine startup
  failures retain their cleanup behavior.

### Runtime Evidence

- Default `Start` was executed with real `/odom` and `odom -> base_link` TF
  unavailable. After the 60-second gate, it returned exit code 0 and recorded
  `实时里程计/TF未就绪，暂不能建图`.
- `smartcar-mapping-session` remained running and was the only TCP-8765 owner.
  Its ROS graph contained `/robot_state_publisher`, `/s3_ydlidar_bridge`,
  `/slam_toolbox`, and `/rviz2`; the container process list confirmed each
  corresponding process remained alive.
- Bounded `/odom` and TF observations timed out, documenting missing live data
  rather than manufacturing it. An explicit `Stop` returned the system to one
  `smartcar-mapping-safe` container with all live telemetry, odom, and TF gates
  disabled.
