#!/usr/bin/env python3
"""Mission FSM for the escape room.

This module owns only the FSM dispatcher (`_tick` + `_handlers`), the
state transitions (`enter`), the navigation-tick fallthrough
(`_tick_navigate`), and the ROS callbacks. Everything else lives in
``escape_room.nodes.explorer.*``:

    params.py        parameter declaration + loading
    sim_setup.py     CoppeliaSim ZMQ resolution + engage-dist auto-detect
    pose.py          TF pose lookup
    door.py          door-threshold approach geometry
    explore.py       frontier-goal selection
    gripper_phase.py pickup_open / pickup_close / drop_open dispatcher
    pickup.py        pickup_align tick
    drop.py          drop_align + drop_backup ticks
    exit_drive.py    exit_drive tick
    frontier.py      occupancy-grid frontier extraction
    gripper_io.py    ZMQ gripper / cube-detectability wrapper
    nav_client.py    Nav2 NavigateToPose action client wrapper

State sequence:

    explore
      → go_to_key
      → pickup_open → pickup_align → pickup_close
      → go_to_plate
      → drop_align → drop_open → drop_backup
      → go_to_door → exit_drive
      → done
"""

from __future__ import annotations

import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from .explorer.door import door_threshold_xy_yaw
from .explorer.drop import tick_drop_align, tick_drop_backup
from .explorer.exit_drive import tick_exit_drive
from .explorer.explore import send_frontier_goal
from .explorer.gripper_phase import tick_gripper_wait
from .explorer.nav_client import NavClient
from .explorer.params import declare_explorer_params
from .explorer.pickup import tick_pickup_align
from .explorer.pose import lookup_pose
from .explorer.sim_setup import setup_sim


class ExplorerNode(Node):
    """Mission FSM: sends Nav2 goals and drives gripper via ZMQ."""

    def __init__(self) -> None:
        super().__init__("explorer_node")
        declare_explorer_params(self)
        self.gripper = setup_sim(self)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.nav = NavClient(self, self.map_frame)

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.path_pub = self.create_publisher(PathMsg, "/exploration/path", 10)
        marker_pub = self.create_publisher(MarkerArray, "/targets/markers", latched)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self._publish_target_markers(marker_pub)

        self.targets: dict[str, tuple[float, float]] = dict(self.sim_targets)
        self.mode = "explore"
        self.action_t = 0.0
        self.current_map: OccupancyGrid | None = None
        self._started = False

        self._handlers = {
            "explore": self._tick_navigate,
            "go_to_key": self._tick_navigate,
            "go_to_plate": self._tick_navigate,
            "go_to_door": self._tick_navigate,
            "pickup_open": lambda: tick_gripper_wait(self),
            "pickup_align": lambda: tick_pickup_align(self),
            "pickup_close": lambda: tick_gripper_wait(self),
            "drop_open": lambda: tick_gripper_wait(self),
            "drop_align": lambda: tick_drop_align(self),
            "drop_backup": lambda: tick_drop_backup(self),
            "exit_drive": lambda: tick_exit_drive(self),
            "done": self.stop,
        }

        self.create_timer(1.0 / self.control_rate_hz, self._tick)
        self.get_logger().info("ready; waiting for Nav2 and slam_toolbox...")

    # ===== ROS callbacks =============================================

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.current_map = msg

    # ===== FSM dispatcher ============================================

    def _tick(self) -> None:
        if not self._started:
            if self._is_ready():
                self._started = True
                self.get_logger().info("Nav2 + map + TF ready; starting mission")
            return
        self._handlers[self.mode]()

    def _is_ready(self) -> bool:
        return (
            self.nav.server_ready
            and self.current_map is not None
            and self.get_robot_pose() is not None
        )

    def _tick_navigate(self) -> None:
        """Runs while a Nav2 goal is in flight or has just returned."""
        if self.nav.active:
            return
        if self.mode == "explore":
            if not send_frontier_goal(self):
                self.enter("go_to_key")
        elif self.mode == "go_to_key":
            self.stop()
            self.enter("pickup_open")
            self.action_t = self.clock_s()
            self.gripper.open()
        elif self.mode == "go_to_plate":
            self.stop()
            self.enter("drop_align")
        elif self.mode == "go_to_door":
            self.enter("exit_drive")
            self.action_t = self.clock_s()

    # ===== state transitions =========================================

    def enter(self, mode: str) -> None:
        self.get_logger().info(f"mode: {self.mode} → {mode}")
        self.nav.cancel()
        self.mode = mode
        if mode == "go_to_key":
            cx, cy = self.targets["cube"]
            rx, ry, _ = self.get_robot_pose()
            yaw = math.atan2(cy - ry, cx - rx)
            self.publish_nav_goal_path(cx, cy)
            self.nav.send(cx, cy, yaw=yaw)
        elif mode == "go_to_plate":
            px, py = self.targets["plate"]
            self.publish_nav_goal_path(px, py)
            self.nav.send(px, py)
        elif mode == "go_to_door":
            tx, ty, yaw = door_threshold_xy_yaw(
                self.targets["door"], self.door_normal, self.door_threshold_inset
            )
            self.get_logger().info(
                f"door threshold=({tx:.2f}, {ty:.2f}, yaw={math.degrees(yaw):+.0f}°)"
            )
            self.publish_nav_goal_path(tx, ty)
            self.nav.send(tx, ty, yaw=yaw)

    # ===== shared helpers used by phase modules ======================

    def _publish_target_markers(self, pub) -> None:
        colors = {
            "cube":  (0.9, 0.1, 0.9),
            "plate": (0.1, 1.0, 0.1),
            "door":  (0.1, 0.3, 1.0),
        }
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        for i, (name, (mx, my)) in enumerate(self.sim_targets.items()):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = "targets"
            m.id = i * 2
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = mx
            m.pose.position.y = my
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.18
            r, g, b = colors[name]
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.85
            arr.markers.append(m)
            t = Marker()
            t.header = m.header
            t.ns = "target_labels"
            t.id = i * 2 + 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = mx
            t.pose.position.y = my
            t.pose.position.z = 0.6
            t.pose.orientation.w = 1.0
            t.scale.z = 0.18
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text = name
            arr.markers.append(t)
        pub.publish(arr)

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def get_robot_pose(self) -> tuple[float, float, float] | None:
        return lookup_pose(self._tf_buffer, self.map_frame, self.base_frame)

    def clock_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_nav_goal_path(self, x: float, y: float) -> None:
        msg = PathMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        msg.poses = [ps]
        self.path_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ExplorerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.cmd_pub.publish(Twist())
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
