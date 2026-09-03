from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_launch = PathJoinSubstitution([
        FindPackageShare("smartcar_bringup"), "launch", "description.launch.py"
    ])
    bridge_params = PathJoinSubstitution([
        FindPackageShare("s3_ydlidar_bridge"), "config", "bridge.yaml"
    ])
    slam_launch = PathJoinSubstitution([
        FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"
    ])
    slam_params = PathJoinSubstitution([
        FindPackageShare("smartcar_bringup"), "config", "p1_mapping.yaml"
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("smartcar_bringup"), "rviz", "p1_mapping.rviz"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("transport", default_value="unconfigured"),
        DeclareLaunchArgument("replay_file", default_value=""),
        DeclareLaunchArgument("enable_live_odom", default_value="false"),
        DeclareLaunchArgument("publish_odom", default_value="false"),
        DeclareLaunchArgument("publish_tf", default_value="false"),
        DeclareLaunchArgument("allow_live_telemetry", default_value="false"),
        DeclareLaunchArgument("telemetry_expected_source_id", default_value="-1"),
        DeclareLaunchArgument(
            "telemetry_expected_destination_id", default_value="-1"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
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
                    "transport": LaunchConfiguration("transport"),
                    "replay_file": LaunchConfiguration("replay_file"),
                    "enable_live_odom": ParameterValue(
                        LaunchConfiguration("enable_live_odom"), value_type=bool
                    ),
                    "publish_odom": ParameterValue(
                        LaunchConfiguration("publish_odom"), value_type=bool
                    ),
                    "publish_tf": ParameterValue(
                        LaunchConfiguration("publish_tf"), value_type=bool
                    ),
                    "allow_live_telemetry": ParameterValue(
                        LaunchConfiguration("allow_live_telemetry"), value_type=bool
                    ),
                    "telemetry_expected_source_id": LaunchConfiguration(
                        "telemetry_expected_source_id"
                    ),
                    "telemetry_expected_destination_id": LaunchConfiguration(
                        "telemetry_expected_destination_id"
                    ),
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    ),
                },
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                "slam_params_file": slam_params,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            parameters=[{
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                )
            }],
        ),
    ])
