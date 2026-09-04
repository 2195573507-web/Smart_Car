#!/usr/bin/env python3
"""ROS adapter for safe Nav2 frontier exploration."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import math
from pathlib import Path
import subprocess
import sys

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
import rclpy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

try:
    from smartcar_auto_exploration.core import Effect, EffectKind, ExplorationCore, ExplorationState, Health
    from smartcar_auto_exploration.frontier import Grid, select_frontiers
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from smartcar_auto_exploration.core import Effect, EffectKind, ExplorationCore, ExplorationState, Health
    from smartcar_auto_exploration.frontier import Grid, select_frontiers


class AutoExplorationNode(Node):
    def __init__(self) -> None:
        super().__init__("smartcar_auto_exploration")
        self._linear_speed = float(self.declare_parameter("max_linear_speed", 0.05).value)
        if not 0.02 <= self._linear_speed <= 0.08:
            raise RuntimeError("max_linear_speed must be within 0.02 through 0.08 m/s")
        self._sample_timeout_ns = int(float(
            self.declare_parameter("health_timeout_sec", 0.75).value) * 1_000_000_000)
        self._preflight_timeout_ns = int(float(
            self.declare_parameter("preflight_timeout_sec", 15.0).value) * 1_000_000_000)
        self._min_cluster_cells = int(self.declare_parameter("min_frontier_cells", 4).value)
        self._clearance_cells = int(self.declare_parameter("frontier_clearance_cells", 1).value)
        self._min_frontier_distance_m = float(
            self.declare_parameter("min_frontier_distance_m", 0.30).value)
        self._artifact_directory = str(self.declare_parameter("artifact_directory", "/ws/maps").value)
        self._robot_radius_m = float(self.declare_parameter("robot_radius_m", 0.0).value)
        self._core = ExplorationCore(int(self.declare_parameter("max_consecutive_failures", 3).value))
        self._last_scan_ns = 0
        self._last_odom_ns = 0
        self._last_tf_ns = 0
        self._motion_healthy = False
        self._gateway_ready = False
        self._map: OccupancyGrid | None = None
        self._goal_handle = None
        self._preflight_deadline_ns = 0
        self._save_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="map-save")
        self._save_inflight = False
        self.add_on_set_parameters_callback(self._validate_parameters)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._navigate_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._arm_client = self.create_client(SetBool, "/smartcar_motion_gateway/set_motion_enabled")
        self._wire_stop_client = self.create_client(Trigger, "/smartcar_motion_gateway/stop")
        self._controller_params = self.create_client(
            SetParameters, "/controller_server/set_parameters")
        self._smoother_params = self.create_client(
            SetParameters, "/velocity_smoother/set_parameters")
        self._global_costmap_params = self.create_client(
            SetParameters, "/global_costmap/global_costmap/set_parameters")
        self._local_costmap_params = self.create_client(
            SetParameters, "/local_costmap/local_costmap/set_parameters")

        self._zero_pub = self.create_publisher(Twist, "/nav2/cmd_vel", 10)
        self._state_pub = self.create_publisher(String, "~/state", 10)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 10)
        from sensor_msgs.msg import LaserScan
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, "/smartcar/motion_healthy", self._on_motion_health, 10)
        self.create_subscription(DiagnosticArray, "/smartcar/motion_diagnostics",
                                 self._on_gateway_diagnostics, 10)
        self.create_service(Trigger, "~/start", self._start)
        self.create_service(Trigger, "~/stop", self._stop)
        self.create_timer(0.10, self._tick)
        self._publish_state()

    def _steady_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _on_scan(self, message) -> None:
        if message.ranges and message.header.frame_id == "laser_frame":
            self._last_scan_ns = self._steady_ns()

    def _on_odom(self, message: Odometry) -> None:
        if message.header.frame_id == "odom" and message.child_frame_id == "base_link":
            self._last_odom_ns = self._steady_ns()

    def _on_tf(self, message: TFMessage) -> None:
        if any(transform.header.frame_id == "odom" and transform.child_frame_id == "base_link"
               for transform in message.transforms):
            self._last_tf_ns = self._steady_ns()

    def _on_map(self, message: OccupancyGrid) -> None:
        if message.info.width > 0 and message.info.height > 0 and message.data:
            self._map = message

    def _on_motion_health(self, message: Bool) -> None:
        self._motion_healthy = bool(message.data)

    def _on_gateway_diagnostics(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name != "smartcar_motion_gateway":
                continue
            values = {entry.key: entry.value for entry in status.values}
            # A telemetry-only session intentionally has no lease.  Requiring one
            # here would prevent the preflight from ever reaching its ARM effect.
            # After ARM succeeds, motion_healthy still requires an active lease.
            self._gateway_ready = all(values.get(key) == "true" for key in (
                "authentication_ready", "protocol_ready", "scan", "odom",
                "tf_odom_base_link"))
            return

    def _health(self) -> Health:
        now_ns = self._steady_ns()
        fresh = lambda sample_ns: sample_ns != 0 and 0 <= now_ns - sample_ns <= self._sample_timeout_ns
        return Health(scan=fresh(self._last_scan_ns), odom=fresh(self._last_odom_ns),
                      tf=fresh(self._last_tf_ns), gateway_ready=self._gateway_ready,
                      motion_healthy=self._motion_healthy)

    def _start(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        if self._robot_radius_m <= 0.0:
            response.success = False
            response.message = "a measured robot_radius_m is required before autonomous motion"
            return response
        if not self._core.start():
            response.success = False
            response.message = f"exploration is already {self._core.state.value.lower()}"
            return response
        self._preflight_deadline_ns = self._steady_ns() + self._preflight_timeout_ns
        self._publish_state()
        response.success = True
        response.message = "preflight started; no motion is authorized until health checks pass"
        return response

    def _stop(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._apply(self._core.stop("operator stop"))
        response.success = True
        response.message = "exploration stopped and motion gateway disarmed"
        return response

    def _tick(self) -> None:
        effects = self._core.update_health(self._health())
        self._apply(effects)
        if self._core.state is ExplorationState.PREFLIGHT and self._steady_ns() > self._preflight_deadline_ns:
            self._apply(self._core.fault("preflight timed out waiting for scan, odom, TF, and lease"))
        if self._core.state is ExplorationState.EXPLORING:
            self._consider_current_map()
        self._publish_state()

    def _consider_current_map(self) -> None:
        if self._map is None or self._core.active_goal is not None:
            return
        try:
            transform = self._tf_buffer.lookup_transform("map", "base_link", Time())
        except Exception:
            return
        info = self._map.info
        grid = Grid(int(info.width), int(info.height), float(info.resolution),
                    float(info.origin.position.x), float(info.origin.position.y), tuple(self._map.data))
        frontiers = select_frontiers(
            grid, transform.transform.translation.x, transform.transform.translation.y,
            min_cluster_cells=self._min_cluster_cells,
            clearance_cells=self._clearance_cells,
            min_distance_m=self._min_frontier_distance_m,
            excluded_keys=self._core.failed_frontiers,
        )
        self._apply(self._core.consider_frontiers(frontiers))

    def _apply(self, effects: list[Effect]) -> None:
        for effect in effects:
            if effect.kind is EffectKind.ARM:
                self._configure_nav2_speed()
            elif effect.kind is EffectKind.NAVIGATE and effect.goal is not None:
                self._send_navigation_goal(effect.goal.x, effect.goal.y)
            elif effect.kind is EffectKind.ZERO_STOP:
                self._cancel_navigation()
                self._publish_zero()
                self._send_wire_stop()
            elif effect.kind is EffectKind.HARD_STOP:
                self._cancel_navigation()
                self._publish_zero()
                self._disarm_gateway()
                self.get_logger().warning(effect.reason)
            elif effect.kind is EffectKind.SAVE:
                self._save_artifacts()
        self._publish_state()

    def _configure_nav2_speed(self) -> None:
        if self._robot_radius_m <= 0.0:
            self._apply(self._core.fault("measured robot radius is unavailable"))
            return
        if not self._navigate_client.wait_for_server(timeout_sec=0.2):
            self._apply(self._core.fault("NavigateToPose action is unavailable"))
            return
        if (not self._global_costmap_params.wait_for_service(timeout_sec=0.2) or
                not self._local_costmap_params.wait_for_service(timeout_sec=0.2)):
            self._apply(self._core.fault("Nav2 costmap parameter service is unavailable"))
            return
        global_future = self._set_remote_parameters(self._global_costmap_params, [
            Parameter("robot_radius", value=self._robot_radius_m)
        ])
        global_future.add_done_callback(
            self._costmap_radius_result)

    def _costmap_radius_result(self, future) -> None:
        try:
            result = future.result().results
            if not all(item.successful for item in result):
                raise RuntimeError("global costmap rejected measured robot radius")
        except Exception as exc:
            self._apply(self._core.fault(f"cannot configure global costmap footprint: {exc}"))
            return
        future = self._set_remote_parameters(self._local_costmap_params, [
            Parameter("robot_radius", value=self._robot_radius_m)
        ])
        future.add_done_callback(self._local_radius_result)

    def _local_radius_result(self, future) -> None:
        try:
            result = future.result().results
            if not all(item.successful for item in result):
                raise RuntimeError("local costmap rejected measured robot radius")
        except Exception as exc:
            self._apply(self._core.fault(f"cannot configure local costmap footprint: {exc}"))
            return
        if not self._controller_params.wait_for_service(timeout_sec=0.2):
            self._apply(self._core.fault("controller_server parameter service is unavailable"))
            return
        request = [
            Parameter("FollowPath.max_vel_x", value=self._linear_speed),
            Parameter("FollowPath.max_speed_xy", value=self._linear_speed),
        ]
        future = self._set_remote_parameters(self._controller_params, request)
        future.add_done_callback(self._controller_speed_result)

    def _validate_parameters(self, parameters):
        result = SetParametersResult()
        result.successful = True
        result.reason = ""
        for parameter in parameters:
            if parameter.name == "max_linear_speed":
                value = float(parameter.value)
                if not 0.02 <= value <= 0.08:
                    result.successful = False
                    result.reason = "max_linear_speed must be within 0.02 through 0.08 m/s"
                    return result
                self._linear_speed = value
            elif parameter.name == "robot_radius_m":
                value = float(parameter.value)
                if value < 0.0:
                    result.successful = False
                    result.reason = "robot_radius_m cannot be negative"
                    return result
                self._robot_radius_m = value
        return result

    def _controller_speed_result(self, future) -> None:
        try:
            result = future.result().results
            if not all(item.successful for item in result):
                raise RuntimeError("controller rejected exploration speed")
        except Exception as exc:
            self._apply(self._core.fault(f"cannot configure Nav2 controller speed: {exc}"))
            return
        if not self._smoother_params.wait_for_service(timeout_sec=0.2):
            self._apply(self._core.fault("velocity_smoother parameter service is unavailable"))
            return
        future = self._set_remote_parameters(self._smoother_params, [
            Parameter("max_velocity", value=[self._linear_speed, 0.0, 0.30])
        ])
        future.add_done_callback(self._smoother_speed_result)

    def _smoother_speed_result(self, future) -> None:
        try:
            result = future.result().results
            if not all(item.successful for item in result):
                raise RuntimeError("velocity smoother rejected exploration speed")
        except Exception as exc:
            self._apply(self._core.fault(f"cannot configure Nav2 smoother speed: {exc}"))
            return
        if not self._arm_client.wait_for_service(timeout_sec=0.2):
            self._apply(self._core.fault("motion gateway arm service is unavailable"))
            return
        request = SetBool.Request()
        request.data = True
        future = self._arm_client.call_async(request)
        future.add_done_callback(self._arm_result)

    def _arm_result(self, future) -> None:
        try:
            response = future.result()
            accepted = bool(response.success)
        except Exception:
            accepted = False
        self._apply(self._core.arm_result(accepted))

    @staticmethod
    def _set_remote_parameters(client, parameters):
        request = SetParameters.Request()
        request.parameters = [parameter.to_parameter_msg() for parameter in parameters]
        return client.call_async(request)

    def _send_navigation_goal(self, x: float, y: float) -> None:
        try:
            transform = self._tf_buffer.lookup_transform("map", "base_link", Time())
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except Exception:
            self._apply(self._core.navigation_result(False))
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        yaw = math.atan2(y - robot_y, x - robot_x)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        future = self._navigate_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception:
            handle = None
        if handle is None or not handle.accepted:
            self._apply(self._core.navigation_result(False))
            return
        self._goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._goal_result)

    def _goal_result(self, future) -> None:
        self._goal_handle = None
        try:
            succeeded = future.result().status == GoalStatus.STATUS_SUCCEEDED
        except Exception:
            succeeded = False
        self._apply(self._core.navigation_result(succeeded))

    def _cancel_navigation(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def _publish_zero(self) -> None:
        self._zero_pub.publish(Twist())

    def _send_wire_stop(self) -> None:
        if self._wire_stop_client.service_is_ready():
            self._wire_stop_client.call_async(Trigger.Request())

    def _disarm_gateway(self) -> None:
        if self._arm_client.service_is_ready():
            request = SetBool.Request()
            request.data = False
            self._arm_client.call_async(request)

    def _save_artifacts(self) -> None:
        if self._save_inflight:
            return
        self._save_inflight = True
        timestamp = datetime.utcnow().strftime("%Y%m%d_%H%M%S")
        prefix = str(Path(self._artifact_directory) / f"auto_mapping_{timestamp}")
        future = self._save_executor.submit(self._run_existing_save_scripts, prefix)
        future.add_done_callback(self._save_result)

    @staticmethod
    def _run_existing_save_scripts(prefix: str) -> None:
        scripts_dir = Path(get_package_share_directory("smartcar_bringup")) / "scripts"
        for script_name in ("save_p1_map.sh", "save_p1_posegraph.sh"):
            subprocess.run(["bash", str(scripts_dir / script_name), prefix],
                           check=True, timeout=45)

    def _save_result(self, future) -> None:
        self._save_inflight = False
        try:
            future.result()
            effects = self._core.save_result(True)
        except Exception as exc:
            effects = self._core.save_result(False)
            self.get_logger().error(f"automatic map save failed: {exc}")
        self._apply(effects)

    def _publish_state(self) -> None:
        message = String()
        message.data = self._core.state.value
        if self._core.fault_reason:
            message.data += f": {self._core.fault_reason}"
        self._state_pub.publish(message)

    def destroy_node(self):
        self._publish_zero()
        self._save_executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoExplorationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
