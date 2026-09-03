"""Start read-only slam_toolbox localization from a serialized posegraph."""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    transport = LaunchConfiguration("transport")
    replay_file = LaunchConfiguration("replay_file")
    posegraph_file = LaunchConfiguration("posegraph_file")
    map_start_pose = LaunchConfiguration("map_start_pose")
    map_start_at_dock = LaunchConfiguration("map_start_at_dock")

    description_launch = PathJoinSubstitution([
        FindPackageShare("smartcar_bringup"), "launch", "description.launch.py"
    ])
    bridge_params = PathJoinSubstitution([
        FindPackageShare("s3_ydlidar_bridge"), "config", "bridge.yaml"
    ])
    slam_params = PathJoinSubstitution([
        FindPackageShare("smartcar_bringup"), "config", "p1_localization.yaml"
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("smartcar_bringup"), "rviz", "p1_mapping.rviz"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use /clock only for a bag replay; false for live operation",
        ),
        DeclareLaunchArgument("transport", default_value="unconfigured"),
        DeclareLaunchArgument("replay_file", default_value=""),
        DeclareLaunchArgument(
            "posegraph_file", default_value="",
            description="slam_toolbox posegraph prefix (without .posegraph/.data)",
        ),
        DeclareLaunchArgument(
            "map_start_pose", default_value="[0.0, 0.0, 0.0]",
            description="Initial map pose as a YAML list [x, y, theta]",
        ),
        DeclareLaunchArgument(
            "map_start_at_dock", default_value="false",
            description="Continue from the first serialized node (mapping only)",
        ),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("allow_live_telemetry", default_value="false"),
        DeclareLaunchArgument("enable_live_odom", default_value="false"),
        DeclareLaunchArgument("publish_odom", default_value="false"),
        DeclareLaunchArgument("publish_tf", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
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
                    "transport": transport,
                    "replay_file": replay_file,
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
        ),
        Node(
            package="slam_toolbox",
            executable="localization_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                slam_params,
                {
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    "map_file_name": posegraph_file,
                    "map_start_pose": ParameterValue(
                        map_start_pose, value_type=List[float]
                    ),
                    "map_start_at_dock": ParameterValue(
                        map_start_at_dock, value_type=bool
                    ),
                },
            ],
            # A localization node without a serialized graph has no map to
            # localize against. Keep the no-path smoke launch descriptive and
            # bridge-only; use continue_mapping.launch.py for a fresh map.
            condition=LaunchConfigurationNotEquals("posegraph_file", ""),
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
