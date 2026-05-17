"""Thin wrapper around Nav2's NavigateToPose action client."""

from __future__ import annotations

import math

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class NavClient:
    def __init__(self, node: Node, map_frame: str) -> None:
        self._node = node
        self._map_frame = map_frame
        self._client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self._active = False
        self._handle = None

    @property
    def active(self) -> bool:
        return self._active

    @property
    def server_ready(self) -> bool:
        return self._client.wait_for_server(timeout_sec=0.0)

    def send(self, x: float, y: float, yaw: float = 0.0) -> bool:
        if not self._client.wait_for_server(timeout_sec=0.0):
            self._node.get_logger().warn("Nav2 action server not available yet")
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self._node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        self._active = True
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_accepted)
        return True

    def cancel(self) -> None:
        if self._handle is not None:
            self._handle.cancel_goal_async()
        self._active = False
        self._handle = None

    def _on_goal_accepted(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self._node.get_logger().warn("Nav2 rejected goal")
            self._active = False
            return
        self._handle = handle
        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        status = future.result().status
        self._active = False
        self._handle = None
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().warn(f"Nav2 goal finished with status {status}")
