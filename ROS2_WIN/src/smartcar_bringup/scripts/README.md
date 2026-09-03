# P1 operator helpers

All paths passed to a helper must exist in the same filesystem namespace as
the ROS process.  In the Docker workflow, use `/ws/maps` and `/ws/bags`, which
are mounted from `ROS2_WIN/maps` and `ROS2_WIN/bags`.

## Map and posegraph artifacts

Save the standard Nav2 map while `slam_toolbox` is actively mapping:

```bash
bash /ws/install/smartcar_bringup/share/smartcar_bringup/scripts/save_p1_map.sh /ws/maps/site_20260830
```

Save the continuation posegraph.  The service writes both
`<prefix>.posegraph` and `<prefix>.data`:

```bash
bash /ws/install/smartcar_bringup/share/smartcar_bringup/scripts/save_p1_posegraph.sh /ws/maps/site_20260830
```

Load a saved graph after starting `localization.launch.py` or
`continue_mapping.launch.py`:

```bash
bash /ws/install/smartcar_bringup/share/smartcar_bringup/scripts/load_p1_posegraph.sh \
  /ws/maps/site_20260830 localize 0.0,0.0,0.0
bash /ws/install/smartcar_bringup/share/smartcar_bringup/scripts/load_p1_posegraph.sh \
  /ws/maps/site_20260830 continue 0.0,0.0,0.0
```

The launch files can load the graph during startup by setting
`posegraph_file:=/ws/maps/site_20260830`.  `map_start_pose` is a YAML list;
`map_start_at_dock:=true` is supported only for continuation mapping.

## Read-only rosbag workflow

Record only observation topics; no control topic is included:

```bash
bash /ws/install/smartcar_bringup/share/smartcar_bringup/scripts/record_p1_bag.sh /ws/bags/p1_run
```

Replay with the required simulated clock and start the mapping launch with
`use_sim_time:=true` in a separate shell:

```bash
bash /ws/install/smartcar_bringup/share/smartcar_bringup/scripts/play_p1_bag.sh /ws/bags/p1_run
```

Live operation must use `use_sim_time:=false` and must not run the bag player.
The helpers return a non-zero status when services, files, or map artifacts are
missing or empty; no empty map or posegraph is treated as evidence.
