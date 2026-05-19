#!/usr/bin/env python3
"""Mission FSM for the escape room.

State sequence:
    EXPLORE
      → GO_TO_KEY
      → PICKUP_OPEN → PICKUP_ALIGN → PICKUP_CLOSE
      → GO_TO_PLATE
      → DROP_ALIGN → DROP_OPEN → DROP_BACKUP
      → GO_TO_DOOR → EXIT_DRIVE
      → DONE
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

from .explorer.frontier import compute_frontiers
from .explorer.gripper import GripperIO
from .explorer.nav_client import NavClient
from .explorer.state import State


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class ExplorerNode(Node):
    """Mission FSM: sends Nav2 goals and drives gripper via ZMQ."""

    def __init__(self) -> None:
        super().__init__("explorer_node")
        self._declare_params()
        self.gripper = GripperIO(self.robot_alias, self.cube_alias, self.get_logger())

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
        self.mode: State = State.EXPLORE
        self.action_t: float = 0.0
        self.current_map: OccupancyGrid | None = None
        self._started: bool = False

        self._handlers = {
            State.EXPLORE: self._explore,
            State.GO_TO_KEY: self._go_to_key,
            State.GO_TO_PLATE: self._go_to_plate,
            State.GO_TO_DOOR: self._go_to_door,
            State.PICKUP_OPEN: self._pickup_open,
            State.PICKUP_ALIGN: self._pickup_align,
            State.PICKUP_CLOSE: self._pickup_close,
            State.DROP_OPEN: self._drop_open,
            State.DROP_ALIGN: self._drop_align,
            State.DROP_BACKUP: self._drop_backup,
            State.EXIT_DRIVE: self._exit_drive,
            State.DONE: self._done,
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

    # ===== nav-phase tick methods =====================================

    def _explore(self) -> None:
        if len(self.targets) == 3:
            self._transition(State.GO_TO_KEY)
            self._nav_go_to_key()
            return
        if self.nav.active:
            return
        frontiers = compute_frontiers(self.current_map)
        if not frontiers:
            self.get_logger().info(
                "no frontiers; map fully explored", throttle_duration_sec=5.0
            )
            return
        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry, _ = pose
        fx, fy = min(frontiers, key=lambda f: math.hypot(f[0] - rx, f[1] - ry))
        yaw = math.atan2(fy - ry, fx - rx)
        self.publish_nav_goal(fx, fy, yaw)
        self.nav.send(fx, fy, yaw)

    def _go_to_key(self) -> None:
        if self.nav.active:
            return
        if not self.nav.succeeded:
            self.get_logger().warn("go_to_key nav failed; retrying")
            self._nav_go_to_key()
            return
        self.stop()
        self._transition(State.PICKUP_OPEN)
        self.action_t = self.clock_s()
        self.gripper.open()

    def _go_to_plate(self) -> None:
        if self.nav.active:
            return
        if not self.nav.succeeded:
            self.get_logger().warn("go_to_plate nav failed; retrying")
            self._nav_go_to_plate()
            return
        self.stop()
        self._transition(State.DROP_ALIGN)

    def _go_to_door(self) -> None:
        if self.nav.active:
            return
        if not self.nav.succeeded:
            self.get_logger().warn("go_to_door nav failed; retrying")
            self._nav_go_to_door()
            return
        self._transition(State.EXIT_DRIVE)
        self.action_t = self.clock_s()

    def _done(self) -> None:
        pass

    # ===== gripper-wait tick methods ==================================

    def _pickup_open(self) -> None:
        self.stop()
        elapsed = self.clock_s() - self.action_t
        if self.gripper.is_open(elapsed, self.gripper_timeout):
            self._transition(State.PICKUP_ALIGN)

    def _pickup_close(self) -> None:
        self.stop()
        elapsed = self.clock_s() - self.action_t
        if self.gripper.is_closed(elapsed, self.gripper_timeout):
            self.gripper.set_cube_visible(False)
            self._transition(State.GO_TO_PLATE)
            self._nav_go_to_plate()

    def _drop_open(self) -> None:
        self.stop()
        elapsed = self.clock_s() - self.action_t
        if self.gripper.is_open(elapsed, self.gripper_timeout):
            self.gripper.set_cube_visible(True)
            self._transition(State.DROP_BACKUP)
            self.action_t = self.clock_s()

    # ===== align-phase tick methods ===================================

    def _pickup_align(self) -> None:
        """P-controller: face cube, approach to pickup_engage_dist, then close."""
        if self._align(
            self.targets["cube"], self.gripper.pickup_engage_dist,
            self.pickup_engage_dist_tol, State.PICKUP_CLOSE,
        ):
            self.action_t = self.clock_s()
            self.gripper.close()

    def _drop_align(self) -> None:
        """P-controller: face plate, approach to drop_distance, then open gripper."""
        if self._align(
            self.targets["plate"], self.drop_distance,
            self.drop_dist_tol, State.DROP_OPEN,
        ):
            self.action_t = self.clock_s()
            self.gripper.open()

    def _align(
        self,
        target_xy: tuple[float, float],
        engage_dist: float,
        dist_tol: float,
        next_state: State,
    ) -> bool:
        """Shared P-controller. Returns True when transition to next_state fires."""
        pose = self.get_robot_pose()
        if pose is None:
            return False
        rx, ry, ryaw = pose
        tx, ty = target_xy
        dist = math.hypot(tx - rx, ty - ry)
        target_yaw = math.atan2(ty - ry, tx - rx)
        yaw_err = _wrap(target_yaw - ryaw)

        if abs(yaw_err) > self.align_yaw_tol:
            twist = Twist()
            twist.angular.z = _clamp(
                self.align_kp * yaw_err, -self.align_max_omega, self.align_max_omega
            )
            self.cmd_pub.publish(twist)
            return False

        dist_err = dist - engage_dist
        if abs(dist_err) <= dist_tol:
            self.stop()
            self._transition(next_state)
            return True

        twist = Twist()
        twist.linear.x = _clamp(
            0.5 * dist_err, -self.park_max_speed, self.park_max_speed
        )
        twist.angular.z = 0.5 * yaw_err
        self.cmd_pub.publish(twist)
        return False

    # ===== timed-drive tick methods ===================================

    def _drop_backup(self) -> None:
        if self.clock_s() - self.action_t >= self.backup_duration:
            self.stop()
            self._transition(State.GO_TO_DOOR)
            self._nav_go_to_door()
            return
        twist = Twist()
        twist.linear.x = -self.backup_speed
        self.cmd_pub.publish(twist)

    def _exit_drive(self) -> None:
        if self.clock_s() - self.action_t >= self.exit_drive_duration:
            self.stop()
            self._transition(State.DONE)
            self.get_logger().info("mission complete")
            return
        twist = Twist()
        twist.linear.x = self.exit_drive_speed
        self.cmd_pub.publish(twist)

    # ===== state transition + nav goal helpers =======================

    def _transition(self, state: State) -> None:
        self.get_logger().info(f"{self.mode} → {state}")
        self.nav.cancel()
        self.mode = state

    def _nav_go_to_key(self) -> None:
        pose = self.get_robot_pose()
        if pose is None:
            self.get_logger().warn("go_to_key: TF unavailable, nav goal deferred")
            return
        rx, ry, _ = pose
        cx, cy = self.targets["cube"]
        yaw = math.atan2(cy - ry, cx - rx)
        sx = cx - self.pickup_standoff * math.cos(yaw)
        sy = cy - self.pickup_standoff * math.sin(yaw)
        self.publish_nav_goal_path(sx, sy)
        self.nav.send(sx, sy, yaw=yaw)

    def _nav_go_to_plate(self) -> None:
        px, py = self.targets["plate"]
        pose = self.get_robot_pose()
        if pose is not None:
            rx, ry, _ = pose
            yaw = math.atan2(py - ry, px - rx)
        else:
            yaw = 0.0
            self.get_logger().warn("go_to_plate: TF unavailable, yaw=0.0")
        self.publish_nav_goal_path(px, py)
        self.nav.send(px, py, yaw=yaw)

    def _nav_go_to_door(self) -> None:
        dx, dy = self.targets["door"]
        nx, ny = self.gripper.door_normal
        tx = dx - self.door_threshold_inset * nx
        ty = dy - self.door_threshold_inset * ny
        yaw = math.atan2(ny, nx)
        self.get_logger().info(
            f"door threshold=({tx:.2f}, {ty:.2f}, yaw={math.degrees(yaw):+.0f}°)"
        )
        self.publish_nav_goal_path(tx, ty)
        self.nav.send(tx, ty, yaw=yaw)

    # ===== shared helpers ============================================

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
