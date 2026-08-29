from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="s3_ydlidar_bridge",
            executable="s3_ydlidar_bridge_node",
            name="s3_ydlidar_bridge",
            output="screen",
            parameters=["/ws/config/bridge.yaml"],
        )
    ])
