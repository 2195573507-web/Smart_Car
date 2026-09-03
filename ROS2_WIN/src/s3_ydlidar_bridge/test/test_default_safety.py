import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import rclpy


def generate_test_description():
    bridge = launch_ros.actions.Node(
        package="s3_ydlidar_bridge",
        executable="s3_ydlidar_bridge_node",
        name="s3_ydlidar_bridge_default_test",
        output="screen",
        parameters=[{"transport": "unconfigured"}],
    )
    return (
        launch.LaunchDescription([
            bridge,
            launch_testing.actions.ReadyToTest(),
        ]),
        {"bridge": bridge},
    )


class TestDefaultSafety(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("default_safety_probe")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def publishers_from_bridge(self, topic):
        deadline = time.monotonic() + 3.0
        endpoints = []
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            endpoints = self.node.get_publishers_info_by_topic(topic)
            if any(
                endpoint.node_name == "s3_ydlidar_bridge_default_test"
                for endpoint in endpoints
            ):
                break
        return [
            endpoint
            for endpoint in endpoints
            if endpoint.node_name == "s3_ydlidar_bridge_default_test"
        ]

    def test_bridge_starts(self, proc_info, bridge):
        proc_info.assertWaitForStartup(process=bridge, timeout=10)

    def test_probe_observes_bridge_diagnostics(self):
        self.assertNotEqual(self.publishers_from_bridge("/diagnostics"), [])

    def test_default_has_no_odom_publisher(self):
        self.assertEqual(self.publishers_from_bridge("/odom"), [])

    def test_default_has_no_dynamic_tf_publisher(self):
        self.assertEqual(self.publishers_from_bridge("/tf"), [])
