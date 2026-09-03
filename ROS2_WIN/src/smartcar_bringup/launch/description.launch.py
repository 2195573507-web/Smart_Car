from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    xacro_path = PathJoinSubstitution([
        FindPackageShare("smartcar_description"),
        "urdf",
        "smart_car.urdf.xacro",
    ])
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            xacro_path,
            " 'laser_xyz:=", LaunchConfiguration("laser_xyz"), "'",
            " 'laser_rpy:=", LaunchConfiguration("laser_rpy"), "'",
            " 'imu_xyz:=", LaunchConfiguration("imu_xyz"), "'",
            " 'imu_rpy:=", LaunchConfiguration("imu_rpy"), "'",
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
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
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }],
        ),
    ])
