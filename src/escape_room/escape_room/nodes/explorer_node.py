#!/usr/bin/env python3
"""Mission FSM for the escape room.

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
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

from .explorer.frontier import send_frontier_goal
from .explorer.nav_client import NavClient
from .explorer.phases import (
    tick_drop_align,
    tick_drop_backup,
    tick_exit_drive,
    tick_gripper_wait,
    tick_pickup_align,
)
from .explorer.sim import setup_sim


class ExplorerNode(Node):
    """Mission FSM: sends Nav2 goals and drives gripper via ZMQ."""

    def __init__(self) -> None:
        super().__init__("explorer_node")
        self._declare_params()
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
        for name in ("cube", "plate", "door"):
            self.create_subscription(
                PoseStamped,
                f"/targets/{name}",
                lambda m, n=name: self._on_target(n, m),
                latched,
            )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)

        self.targets: dict[str, tuple[float, float]] = {}
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
            "done": self._tick_done,
        }

        self.create_timer(1.0 / self.control_rate_hz, self._tick)
        self.get_logger().info("ready; waiting for Nav2 and slam_toolbox...")

    def _declare_params(self) -> None:
        p = self.declare_parameter
        p("robot_alias", "/RoboMasterEP/BaseLinkFrame")
        p("cube_alias", "/TargetCube")
        p("map_frame", "map")
        p("base_frame", "base_link")
        p("control_rate_hz", 4.0)
        p("door_threshold_inset_m", 0.20)
        p("exit_drive_speed_mps", 0.10)
        p("exit_drive_duration_s", 5.0)
        p("pickup_standoff_m", 0.50)
        p("pickup_engage_dist_tol_m", 0.03)
        p("drop_backup_speed_mps", 0.05)
        p("drop_backup_duration_s", 8.0)
        p("plate_drop_distance_m", 0.30)
        p("plate_drop_dist_tol_m", 0.04)
        p("park_max_speed_mps", 0.06)
        p("align_yaw_tol_rad", 0.08)
        p("align_kp", 1.5)
        p("align_max_omega", 0.6)
        p("gripper_timeout_s", 4.0)

        def g(n):
            return self.get_parameter(n).value

        self.robot_alias = g("robot_alias")
        self.cube_alias = g("cube_alias")
        self.map_frame = g("map_frame")
        self.base_frame = g("base_frame")
        self.control_rate_hz = float(g("control_rate_hz"))
        self.door_threshold_inset = float(g("door_threshold_inset_m"))
        self.exit_drive_speed = float(g("exit_drive_speed_mps"))
        self.exit_drive_duration = float(g("exit_drive_duration_s"))
        self.pickup_standoff = float(g("pickup_standoff_m"))
        self.pickup_engage_dist_tol = float(g("pickup_engage_dist_tol_m"))
        self.backup_speed = float(g("drop_backup_speed_mps"))
        self.backup_duration = float(g("drop_backup_duration_s"))
        self.drop_distance = float(g("plate_drop_distance_m"))
        self.drop_dist_tol = float(g("plate_drop_dist_tol_m"))
        self.park_max_speed = float(g("park_max_speed_mps"))
        self.align_yaw_tol = float(g("align_yaw_tol_rad"))
        self.align_kp = float(g("align_kp"))
        self.align_max_omega = float(g("align_max_omega"))
        self.gripper_timeout = float(g("gripper_timeout_s"))

    # ===== ROS callbacks =============================================

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.current_map = msg

    def _on_target(self, name: str, msg: PoseStamped) -> None:
        x, y = msg.pose.position.x, msg.pose.position.y
        if name in self.targets:
            ox, oy = self.targets[name]
            if math.hypot(x - ox, y - oy) <= 0.1:
                return
            self.get_logger().info(
                f"target '{name}' updated ({ox:.2f},{oy:.2f})→({x:.2f},{y:.2f})"
            )
        else:
            self.get_logger().info(
                f"saw '{name}' at ({x:.2f}, {y:.2f}) [{len(self.targets) + 1}/3]"
            )
        self.targets[name] = (x, y)

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
        # Immediately cancel frontier nav when all targets known
        if self.mode == "explore" and len(self.targets) == 3:
            self.enter("go_to_key")
            return

        if self.nav.active:
            return

        if self.mode == "explore":
            send_frontier_goal(self)

        elif self.mode == "go_to_key":
            if not self.nav.succeeded:
                self.get_logger().warn("go_to_key nav failed; retrying")
                self.enter("go_to_key")
                return
            self.stop()
            self.enter("pickup_open")

        elif self.mode == "go_to_plate":
            if not self.nav.succeeded:
                self.get_logger().warn("go_to_plate nav failed; retrying")
                self.enter("go_to_plate")
                return
            self.stop()
            self.enter("drop_align")

        elif self.mode == "go_to_door":
            if not self.nav.succeeded:
                self.get_logger().warn("go_to_door nav failed; retrying")
                self.enter("go_to_door")
                return
            self.enter("exit_drive")

    def _tick_done(self) -> None:
        pass

    # ===== state transitions =========================================

    def enter(self, mode: str) -> None:
        self.get_logger().info(f"mode: {self.mode} → {mode}")
        self.nav.cancel()
        self.mode = mode

        if mode == "go_to_key":
            pose = self.get_robot_pose()
            if pose is None:
                self.get_logger().warn("enter(go_to_key): TF unavailable, deferring")
                self.mode = "explore"
                return
            rx, ry, _ = pose
            cx, cy = self.targets["cube"]
            yaw = math.atan2(cy - ry, cx - rx)
            sx = cx - self.pickup_standoff * math.cos(yaw)
            sy = cy - self.pickup_standoff * math.sin(yaw)
            self.publish_nav_goal_path(sx, sy)
            self.nav.send(sx, sy, yaw=yaw)

        elif mode == "go_to_plate":
            px, py = self.targets["plate"]
            pose = self.get_robot_pose()
            if pose is not None:
                rx, ry, _ = pose
                yaw = math.atan2(py - ry, px - rx)
            else:
                yaw = 0.0
                self.get_logger().warn("enter(go_to_plate): TF unavailable, yaw=0.0")
            self.publish_nav_goal_path(px, py)
            self.nav.send(px, py, yaw=yaw)

        elif mode == "go_to_door":
            dx, dy = self.targets["door"]
            nx, ny = self.door_normal
            tx = dx - self.door_threshold_inset * nx
            ty = dy - self.door_threshold_inset * ny
            yaw = math.atan2(ny, nx)
            self.get_logger().info(
                f"door threshold=({tx:.2f}, {ty:.2f}, yaw={math.degrees(yaw):+.0f}°)"
            )
            self.publish_nav_goal_path(tx, ty)
            self.nav.send(tx, ty, yaw=yaw)

        elif mode == "pickup_open":
            self.action_t = self.clock_s()
            self.gripper.open()

        elif mode == "pickup_close":
            self.action_t = self.clock_s()
            self.gripper.close()

        elif mode == "drop_open":
            self.action_t = self.clock_s()
            self.gripper.open()

        elif mode == "drop_backup":
            self.action_t = self.clock_s()

        elif mode == "exit_drive":
            self.action_t = self.clock_s()

        elif mode == "done":
            self.get_logger().info("mission complete")

    # ===== shared helpers used by phase modules ======================

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def get_robot_pose(self) -> tuple[float, float, float] | None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.1)
            )
        except Exception:
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2)
        )
        return float(x), float(y), float(yaw)

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
