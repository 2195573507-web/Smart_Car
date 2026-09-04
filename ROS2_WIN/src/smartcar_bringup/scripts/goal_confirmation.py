#!/usr/bin/env python3
import math
from pathlib import Path
import sys

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Path as PathMsg
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus

try:
    from .goal_confirmation_core import GoalConfirmation, GoalState
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from goal_confirmation_core import GoalConfirmation, GoalState


class GoalConfirmationNode(Node):
    def __init__(self):
        super().__init__("smartcar_goal_confirmation")
        self._state = GoalConfirmation()
        self._healthy = False
        self._goal_handle = None
        self._preview_handle = None
        self._preview_request_id = 0
        self._pending_pose = None
        self._goal_topic = self.declare_parameter("goal_topic", "/goal_pose").value
        self._pending_topic = self.declare_parameter("pending_goal_topic", "/smartcar/pending_goal").value
        self._preview_topic = self.declare_parameter("preview_path_topic", "/smartcar/goal_preview").value
        self._cmd_topic = self.declare_parameter("nav_cmd_topic", "/nav2/cmd_vel").value
        self._goal_sub = self.create_subscription(PoseStamped, self._goal_topic, self._on_goal, 10)
        self._health_sub = self.create_subscription(Bool, "/smartcar/motion_healthy", self._on_health, 10)
        self._pending_pub = self.create_publisher(PoseStamped, self._pending_topic, 10)
        self._preview_pub = self.create_publisher(PathMsg, self._preview_topic, 10)
        self._zero_pub = self.create_publisher(Twist, self._cmd_topic, 10)
        self._start_srv = self.create_service(Trigger, "~/start", self._start)
        self._cancel_srv = self.create_service(Trigger, "~/cancel", self._cancel)
        self._preview_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

    @staticmethod
    def _yaw(pose):
        return math.atan2(2.0 * (pose.orientation.w * pose.orientation.z),
                          1.0 - 2.0 * (pose.orientation.z * pose.orientation.z))

    def _on_goal(self, message):
        if message.header.frame_id != "map":
            self.get_logger().warning("goal frame must be map")
            return
        if self._state.state is GoalState.ACTIVE:
            self._cancel_active("new goal selected")
        self._state.select(message.pose.position.x, message.pose.position.y, self._yaw(message.pose))
        self._pending_pose = message
        self._pending_pub.publish(message)
        self._request_preview(message)

    def _request_preview(self, message):
        self._preview_request_id += 1
        request_id = self._preview_request_id
        if not self._preview_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().warning("compute_path_to_pose action unavailable; no preview path")
            self._state.clear_preview()
            self._preview_pub.publish(PathMsg())
            return
        goal = ComputePathToPose.Goal()
        goal.goal = message
        goal.use_start = False
        future = self._preview_client.send_goal_async(goal)
        future.add_done_callback(lambda result: self._preview_response(result, request_id))

    def _preview_response(self, future, request_id):
        if request_id != self._preview_request_id or self._state.state is not GoalState.PENDING:
            return
        self._preview_handle = future.result()
        if self._preview_handle is None or not self._preview_handle.accepted:
            self._state.clear_preview()
            self._preview_pub.publish(PathMsg())
            return
        result_future = self._preview_handle.get_result_async()
        result_future.add_done_callback(lambda result: self._preview_result(result, request_id))

    def _preview_result(self, future, request_id):
        if request_id != self._preview_request_id or self._state.state is not GoalState.PENDING:
            return
        result = future.result().result
        self._state.set_preview(
            (pose.pose.position.x, pose.pose.position.y, self._yaw(pose.pose))
            for pose in result.path.poses
        )
        self._preview_pub.publish(result.path)
        self._preview_handle = None

    def _on_health(self, message):
        self._healthy = bool(message.data)
        if not self._healthy and self._state.state is not GoalState.EMPTY:
            self._cancel_active("health gate lost")

    def _start(self, request, response):
        del request
        if not self._state.start(self._healthy):
            response.success = False
            response.message = "pending goal or healthy motion gate required"
            return response
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self._cancel_active("navigate_to_pose action unavailable")
            response.success = False
            response.message = "navigate_to_pose action unavailable"
            return response
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        # Keep the exact selected pose independent of publisher implementation.
        if self._state.goal:
            x, y, yaw = self._state.goal
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)
        response.success = True
        response.message = "navigation started"
        return response

    def _goal_response(self, future):
        self._goal_handle = future.result()
        if self._goal_handle is None or not self._goal_handle.accepted:
            self._state.navigation_failed()
            self._publish_zero()
            self._goal_handle = None
            return
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result)

    def _goal_result(self, future):
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._state.navigation_failed()
            self._publish_zero()
        else:
            self._state.cancel()
            self._publish_zero()
        self._goal_handle = None

    def _cancel_active(self, reason):
        self._preview_request_id += 1
        if self._preview_handle is not None:
            self._preview_handle.cancel_goal_async()
            self._preview_handle = None
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._state.cancel()
        self._publish_zero()
        self.get_logger().warning(reason)

    def _cancel(self, request, response):
        del request
        changed = self._state.state is not GoalState.EMPTY
        self._cancel_active("cancel requested")
        response.success = changed
        response.message = "cancelled" if changed else "no pending goal"
        return response

    def _publish_zero(self):
        self._zero_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = GoalConfirmationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
