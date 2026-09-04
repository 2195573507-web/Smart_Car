from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _require_measured(context):
    if LaunchConfiguration("laser_extrinsics_measured").perform(context).lower() != "true":
        raise RuntimeError("mapping requires measured laser extrinsics; set laser_extrinsics_measured:=true")
    return []


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
    nav2_launch = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"
    ])
    nav2_params = PathJoinSubstitution([
        FindPackageShare("smartcar_bringup"), "config", "auto_exploration_nav2.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("laser_extrinsics_measured", default_value="false"),
        DeclareLaunchArgument("laser_xyz", default_value="0.200 0.000 0.155"),
        DeclareLaunchArgument("laser_rpy", default_value="0.000 0.000 0.000"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("transport", default_value="unconfigured"),
        DeclareLaunchArgument("replay_file", default_value=""),
        DeclareLaunchArgument("record_bag", default_value="true"),
        DeclareLaunchArgument("bag_uri", default_value="/ws/bags/mapping_session"),
        DeclareLaunchArgument("enable_nav2", default_value="true"),
        DeclareLaunchArgument("auto_exploration_speed", default_value="0.05"),
        DeclareLaunchArgument("robot_radius_m", default_value="0.0"),
        OpaqueFunction(function=_require_measured),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "laser_xyz": LaunchConfiguration("laser_xyz"),
                "laser_rpy": LaunchConfiguration("laser_rpy"),
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
        GroupAction(actions=[
            SetRemap(src="cmd_vel_smoothed", dst="/nav2/cmd_vel"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "params_file": nav2_params,
                    "autostart": "true",
                    "use_composition": "False",
                }.items(),
                condition=IfCondition(LaunchConfiguration("enable_nav2")),
            ),
        ]),
        Node(
            package="smartcar_auto_exploration",
            executable="auto_exploration.py",
            name="smartcar_auto_exploration",
            output="screen",
            parameters=[{
                "max_linear_speed": ParameterValue(
                    LaunchConfiguration("auto_exploration_speed"), value_type=float),
                "robot_radius_m": ParameterValue(
                    LaunchConfiguration("robot_radius_m"), value_type=float),
            }],
        ),
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "record", "-o", LaunchConfiguration("bag_uri"),
                "/scan", "/odom", "/tf", "/tf_static", "/map", "/diagnostics",
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("record_bag")),
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
