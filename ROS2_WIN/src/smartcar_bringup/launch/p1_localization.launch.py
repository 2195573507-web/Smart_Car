"""Compatibility launch for the P1 saved-map workflow.

Use ``localization.launch.py`` for a serialized posegraph or
``map_server.launch.py`` for a standalone YAML/PGM map.  This legacy entry
point keeps the original argument names while ensuring that the two map
publishers are never started together.
"""

from launch import LaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _require_measured(context):
    if LaunchConfiguration("laser_extrinsics_measured").perform(context).lower() != "true":
        raise RuntimeError("navigation requires measured laser extrinsics; set laser_extrinsics_measured:=true")
    return []


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    posegraph = LaunchConfiguration("posegraph")
    map_yaml = LaunchConfiguration("map_yaml")

    description_launch = PathJoinSubstitution([
        FindPackageShare("smartcar_bringup"), "launch", "description.launch.py"
    ])
    bridge_params = PathJoinSubstitution([
        FindPackageShare("s3_ydlidar_bridge"), "config", "bridge.yaml"
    ])
    localization_params = PathJoinSubstitution([
        FindPackageShare("smartcar_bringup"), "config", "p1_localization.yaml"
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("smartcar_bringup"), "rviz", "p1_mapping.rviz"
    ])

    has_posegraph = LaunchConfigurationNotEquals("posegraph", "")
    # A YAML map is only loaded when no posegraph-localization node is active;
    # otherwise slam_toolbox is the sole /map publisher.
    map_only = IfCondition(PythonExpression([
        "'", posegraph, "' == '' and '", map_yaml, "' != ''"
    ]))
    has_saved_artifact = IfCondition(PythonExpression([
        "'", posegraph, "' != '' or '", map_yaml, "' != ''"
    ]))

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("laser_extrinsics_measured", default_value="false"),
        DeclareLaunchArgument("posegraph", default_value=""),
        DeclareLaunchArgument("map_yaml", default_value=""),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("transport", default_value="unconfigured"),
        DeclareLaunchArgument("replay_file", default_value=""),
        DeclareLaunchArgument("allow_live_telemetry", default_value="false"),
        DeclareLaunchArgument("enable_live_odom", default_value="false"),
        DeclareLaunchArgument("publish_odom", default_value="false"),
        DeclareLaunchArgument("publish_tf", default_value="false"),
        OpaqueFunction(function=_require_measured),
        DeclareLaunchArgument(
            "laser_xyz", default_value="0.200 0.000 0.155",
            description="Provisional or measured base_link to laser_frame xyz in metres",
        ),
        DeclareLaunchArgument(
            "laser_rpy", default_value="0.000 0.000 0.000",
            description="Provisional or measured base_link to laser_frame rpy in radians",
        ),
        DeclareLaunchArgument(
            "imu_xyz", default_value="0.000 0.000 0.100",
            description="Provisional or measured base_link to imu_link xyz in metres",
        ),
        DeclareLaunchArgument(
            "imu_rpy", default_value="0.000 0.000 0.000",
            description="Provisional or measured base_link to imu_link rpy in radians",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "laser_xyz": LaunchConfiguration("laser_xyz"),
                "laser_rpy": LaunchConfiguration("laser_rpy"),
                "imu_xyz": LaunchConfiguration("imu_xyz"),
                "imu_rpy": LaunchConfiguration("imu_rpy"),
            }.items(),
        ),
        Node(
            package="s3_ydlidar_bridge",
            executable="s3_ydlidar_bridge_node",
            name="s3_ydlidar_bridge",
            output="screen",
            parameters=[
                bridge_params,
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "transport": LaunchConfiguration("transport"),
                    "replay_file": LaunchConfiguration("replay_file"),
                    "allow_live_telemetry": ParameterValue(
                        LaunchConfiguration("allow_live_telemetry"), value_type=bool
                    ),
                    "enable_live_odom": ParameterValue(
                        LaunchConfiguration("enable_live_odom"), value_type=bool
                    ),
                    "publish_odom": ParameterValue(
                        LaunchConfiguration("publish_odom"), value_type=bool
                    ),
                    "publish_tf": ParameterValue(
                        LaunchConfiguration("publish_tf"), value_type=bool
                    ),
                },
            ],
            condition=has_saved_artifact,
        ),
        Node(
            package="slam_toolbox",
            executable="localization_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                localization_params,
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "map_file_name": posegraph,
                },
            ],
            condition=has_posegraph,
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                {
                    "yaml_filename": map_yaml,
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                }
            ],
            condition=map_only,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="map_server_lifecycle_manager",
            output="screen",
            parameters=[
                {
                    "autostart": True,
                    "node_names": ["map_server"],
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                }
            ],
            condition=map_only,
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        ),
    ])
