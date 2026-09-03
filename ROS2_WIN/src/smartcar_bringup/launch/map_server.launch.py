"""Load one saved YAML/PGM map through the official Nav2 map server."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    map_topic = LaunchConfiguration("map_topic")
    map_frame = LaunchConfiguration("map_frame")
    has_map = LaunchConfigurationNotEquals("map_yaml_file", "")

    rviz_config = PathJoinSubstitution([
        FindPackageShare("smartcar_bringup"), "rviz", "p1_mapping.rviz"
    ])
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "yaml_filename": map_yaml_file,
            "topic_name": map_topic,
            "frame_id": map_frame,
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
        condition=has_map,
    )
    # map_server is a lifecycle node. Use the standard lifecycle manager node
    # so launch's generated remap arguments remain valid on Humble.
    lifecycle_bringup = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="map_server_lifecycle_manager",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["map_server"],
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
        condition=has_map,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "map_yaml_file", default_value="",
            description="Absolute path to the saved map YAML file",
        ),
        DeclareLaunchArgument("map_topic", default_value="/map"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        map_server,
        TimerAction(period=1.0, actions=[lifecycle_bringup], condition=has_map),
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
