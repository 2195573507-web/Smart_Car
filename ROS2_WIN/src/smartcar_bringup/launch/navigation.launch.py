"""Measured-sensor gated saved-map navigation with a two-step goal."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _validate(context):
    if LaunchConfiguration("laser_extrinsics_measured").perform(context).lower() != "true":
        raise RuntimeError("navigation requires measured laser extrinsics; set laser_extrinsics_measured:=true")
    if not LaunchConfiguration("map_yaml_file").perform(context).strip():
        raise RuntimeError("navigation requires map_yaml_file pointing to a saved map")
    return []


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    description = PathJoinSubstitution([FindPackageShare("smartcar_bringup"), "launch", "description.launch.py"])
    bridge = PathJoinSubstitution([FindPackageShare("s3_ydlidar_bridge"), "config", "bridge.yaml"])
    nav2_params = PathJoinSubstitution([FindPackageShare("smartcar_bringup"), "config", "nav2_params.yaml"])
    nav2_bringup = PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"])
    motion_params = PathJoinSubstitution([FindPackageShare("smartcar_motion_gateway"), "config", "motion_gateway.yaml"])
    rviz = PathJoinSubstitution([FindPackageShare("smartcar_bringup"), "rviz", "p1_mapping.rviz"])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_yaml_file", default_value=""),
        DeclareLaunchArgument("laser_extrinsics_measured", default_value="false"),
        DeclareLaunchArgument("laser_xyz", default_value="0.200 0.000 0.155"),
        DeclareLaunchArgument("laser_rpy", default_value="0.000 0.000 0.000"),
        DeclareLaunchArgument("transport", default_value="tcp"),
        DeclareLaunchArgument("allow_live_telemetry", default_value="false"),
        DeclareLaunchArgument("enable_live_odom", default_value="false"),
        DeclareLaunchArgument("publish_odom", default_value="false"),
        DeclareLaunchArgument("publish_tf", default_value="false"),
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("protocol_ready", default_value="false"),
        DeclareLaunchArgument("psk_config_path", default_value=""),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=_validate),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(description), launch_arguments={
            "use_sim_time": use_sim_time,
            "laser_xyz": LaunchConfiguration("laser_xyz"),
            "laser_rpy": LaunchConfiguration("laser_rpy"),
        }.items()),
        Node(package="s3_ydlidar_bridge", executable="s3_ydlidar_bridge_node",
             name="s3_ydlidar_bridge", output="screen", parameters=[bridge, {
                 "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                 "transport": LaunchConfiguration("transport"),
                 "allow_live_telemetry": ParameterValue(LaunchConfiguration("allow_live_telemetry"), value_type=bool),
                 "enable_live_odom": ParameterValue(LaunchConfiguration("enable_live_odom"), value_type=bool),
                 "publish_odom": ParameterValue(LaunchConfiguration("publish_odom"), value_type=bool),
                 "publish_tf": ParameterValue(LaunchConfiguration("publish_tf"), value_type=bool),
             }]),
        GroupAction(actions=[
            # nav2_bringup's velocity_smoother consumes the controller's
            # internal cmd_vel_nav and publishes cmd_vel_smoothed. Only that
            # final, smoothed output is exposed to the motion gateway.
            SetRemap(src="cmd_vel_smoothed", dst="/nav2/cmd_vel"),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2_bringup), launch_arguments={
                "map": LaunchConfiguration("map_yaml_file"),
                "params_file": nav2_params,
                "use_sim_time": use_sim_time,
                "autostart": "true",
                "use_composition": "False",
            }.items()),
        ]),
        Node(package="smartcar_motion_gateway", executable="smartcar_motion_gateway_node",
             name="smartcar_motion_gateway", output="screen", parameters=[motion_params, {
                 "enable_motion": ParameterValue(LaunchConfiguration("enable_motion"), value_type=bool),
                 "protocol_ready": ParameterValue(LaunchConfiguration("protocol_ready"), value_type=bool),
                 "psk_config_path": LaunchConfiguration("psk_config_path"),
                 "enable_status_odom": ParameterValue(LaunchConfiguration("enable_live_odom"), value_type=bool),
                 "publish_status_tf": ParameterValue(LaunchConfiguration("publish_tf"), value_type=bool),
             }]),
        Node(package="smartcar_bringup", executable="goal_confirmation.py",
             name="smartcar_goal_confirmation", output="screen"),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
             arguments=["-d", rviz], condition=IfCondition(LaunchConfiguration("use_rviz")),
             parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}]),
    ])
