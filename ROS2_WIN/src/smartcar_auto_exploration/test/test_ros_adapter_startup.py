"""Regression test for inert ROS adapter construction."""

import importlib.util
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.qos import QoSReliabilityPolicy


def _load_adapter_module():
    script = Path(__file__).resolve().parents[1] / "scripts" / "auto_exploration.py"
    spec = importlib.util.spec_from_file_location("auto_exploration_adapter", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_adapter_constructs_without_requesting_exploration():
    initialized_here = not rclpy.ok()
    if initialized_here:
        rclpy.init()
    node = None
    try:
        node = _load_adapter_module().AutoExplorationNode()
        assert node.get_name() == "smartcar_auto_exploration"
        assert node._core.state.value == "IDLE"
        scan_subscription = next(
            endpoint
            for endpoint in node.get_subscriptions_info_by_topic("/scan")
            if endpoint.node_name == "smartcar_auto_exploration"
        )
        assert scan_subscription.qos_profile.reliability == QoSReliabilityPolicy.BEST_EFFORT
    finally:
        if node is not None:
            node.destroy_node()
        if initialized_here:
            rclpy.shutdown()


def test_telemetry_only_gateway_is_preflight_ready_without_a_lease():
    initialized_here = not rclpy.ok()
    if initialized_here:
        rclpy.init()
    node = None
    try:
        node = _load_adapter_module().AutoExplorationNode()
        diagnostic = DiagnosticArray()
        status = DiagnosticStatus(name="smartcar_motion_gateway")
        status.values = [
            KeyValue(key="authentication_ready", value="true"),
            KeyValue(key="protocol_ready", value="true"),
            KeyValue(key="scan", value="true"),
            KeyValue(key="odom", value="true"),
            KeyValue(key="tf_odom_base_link", value="true"),
            KeyValue(key="lease", value="false"),
        ]
        diagnostic.status = [status]

        node._on_gateway_diagnostics(diagnostic)

        assert node._gateway_ready
    finally:
        if node is not None:
            node.destroy_node()
        if initialized_here:
            rclpy.shutdown()
