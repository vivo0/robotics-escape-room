#!/usr/bin/env python3
"""Mission FSM for the escape room.

State sequence:
    EXPLORE
      → GO_TO_KEY
      → PICKUP_OPEN → PICKUP_ALIGN → PICKUP_CLOSE
      → GO_TO_PLATE
      → DROP_OPEN
      → GO_TO_DOOR → EXIT_DRIVE
      → DONE
"""

from __future__ import annotations

import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

from .explorer.frontier import compute_frontiers
from .explorer.gripper import GripperIO
from .explorer.nav_client import NavClient
from .explorer.state import State


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
        self.goal_pub = self.create_publisher(PoseStamped, "/exploration/goal", 10)
        for name in ("cube", "plate", "door"):
            self.create_subscription(
                PoseStamped,
                f"/targets/{name}",
                lambda m, n=name: self._on_target(n, m),
                latched,
            )
        # Live cube bearing (base_link) for visual-servoing pickup.
        self.create_subscription(
            PoseStamped, "/targets/cube_live", self._on_cube_live, 10
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)

        self.targets: dict[str, tuple[float, float]] = {}
        self.mode: State = State.EXPLORE
        self.action_t: float = 0.0
        self.current_map: OccupancyGrid | None = None
        self._started: bool = False
        # Latest cube detection in base_link: (x_fwd, y_left, stamp_s).
        self._cube_live: tuple[float, float, float] | None = None
        # Odometry-measured final-approach ("lunge") state.
        self._lunge_active: bool = False
        self._lunge_start: tuple[float, float] | None = None
        self._lunge_dist: float = 0.0
        self._backup_start: tuple[float, float] | None = None

        self._handlers = {
            State.EXPLORE: self._explore,
            State.GO_TO_KEY: self._go_to_key,
            State.GO_TO_PLATE: self._go_to_plate,
            State.GO_TO_DOOR: self._go_to_door,
            State.PICKUP_OPEN: self._pickup_open,
            State.PICKUP_ALIGN: self._pickup_align,
            State.PICKUP_CLOSE: self._pickup_close,
            State.DROP_ALIGN: self._drop_align,
            State.DROP_OPEN: self._drop_open,
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
        p("odom_frame", "odom")
        p("control_rate_hz", 4.0)
        p("door_threshold_inset_m", 0.20)
        p("exit_drive_speed_mps", 0.10)
        p("exit_drive_duration_s", 10.0)
        p("pickup_standoff_m", 0.90)
        p("park_max_speed_mps", 0.06)
        p("align_yaw_tol_rad", 0.08)
        p("align_kp", 1.5)
        p("align_max_omega", 0.6)
        p("gripper_timeout_s", 4.0)
        # Visual-servoing pickup (perception only). engage_dist is the
        # attachPoint offset from base_link (a fixed robot geometry constant).
        p("pickup_engage_dist_m", 0.177)
        p("cube_live_timeout_s", 0.8)
        # Range at which we stop trusting the height-based monocular depth
        # (the 0.20 m column's base clips out of frame nearer than this) and
        # commit to a straight, odometry-measured final approach.
        p("pickup_lunge_start_m", 0.90)
        # Stop the lunge this much short of the engage distance. Too large
        # and the cube is gripped at the fingertips (weak friction hold → it
        # falls while driving, and is carried too far forward); too small and
        # the approach nudges the thin column over. The carried cube sits
        # ~(engage + this) ahead of base_link, which the plate drop reuses.
        p("pickup_lunge_margin_m", 0.03)
        # After dropping, reverse this far before turning so the gripper
        # clears the cube instead of sweeping it off the plate.
        p("drop_backup_m", 0.25)

        def g(n):
            return self.get_parameter(n).value

        self.robot_alias = g("robot_alias")
        self.cube_alias = g("cube_alias")
        self.map_frame = g("map_frame")
        self.base_frame = g("base_frame")
        self.odom_frame = g("odom_frame")
        self.control_rate_hz = float(g("control_rate_hz"))
        self.door_threshold_inset = float(g("door_threshold_inset_m"))
        self.exit_drive_speed = float(g("exit_drive_speed_mps"))
        self.exit_drive_duration = float(g("exit_drive_duration_s"))
        self.pickup_standoff = float(g("pickup_standoff_m"))
        self.park_max_speed = float(g("park_max_speed_mps"))
        self.align_yaw_tol = float(g("align_yaw_tol_rad"))
        self.align_kp = float(g("align_kp"))
        self.align_max_omega = float(g("align_max_omega"))
        self.gripper_timeout = float(g("gripper_timeout_s"))
        self.pickup_engage_dist = float(g("pickup_engage_dist_m"))
        self.cube_live_timeout = float(g("cube_live_timeout_s"))
        self.pickup_lunge_start = float(g("pickup_lunge_start_m"))
        self.pickup_lunge_margin = float(g("pickup_lunge_margin_m"))
        self.drop_backup_dist = float(g("drop_backup_m"))

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

    def _on_cube_live(self, msg: PoseStamped) -> None:
        # Pose is in base_link: x forward, y left.
        self._cube_live = (
            msg.pose.position.x,
            msg.pose.position.y,
            self.clock_s(),
        )

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
        self._lunge_active = False
        self._lunge_start = None
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

    def _drop_align(self) -> None:
        """Place the carried cube on the plate centre.

        Nav2 stops base_link only within its 0.25 m goal tolerance, and the
        cube sits ``engage_dist`` ahead of base_link — so a plain drop lands
        the cube short of / beside the plate. Here we drive base_link to
        exactly ``engage_dist`` from the perceived plate, facing it, so the
        gripper is over the plate centre, then open."""
        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose
        px, py = self.targets["plate"]
        c, s = math.cos(-ryaw), math.sin(-ryaw)
        dx, dy = px - rx, py - ry
        bx, by = c * dx - s * dy, s * dx + c * dy   # plate in base_link
        bearing = math.atan2(by, bx)
        if abs(bearing) > self.align_yaw_tol:
            self._drive(0.0, self.align_kp * bearing)
            return
        # The carried cube sits ~(engage + lunge margin) ahead of base_link,
        # so stop that far from the plate to drop it on the centre.
        carried = self.pickup_engage_dist + self.pickup_lunge_margin
        err = math.hypot(bx, by) - carried
        if abs(err) <= 0.03:
            self.stop()
            self._transition(State.DROP_OPEN)
            self.action_t = self.clock_s()
            self.gripper.open()
            return
        self._drive(0.5 * err, 0.5 * bearing)

    def _drop_open(self) -> None:
        self.stop()
        elapsed = self.clock_s() - self.action_t
        if not self.gripper.is_open(elapsed, self.gripper_timeout):
            return
        start = self.get_odom_pose()
        if start is None:
            return
        self.gripper.set_cube_visible(True)
        self._backup_start = start
        self._transition(State.DROP_BACKUP)

    def _drop_backup(self) -> None:
        """Reverse straight to clear the just-dropped cube before turning."""
        pose = self.get_odom_pose()
        if pose is None:
            return
        advanced = math.hypot(
            pose[0] - self._backup_start[0], pose[1] - self._backup_start[1])
        if advanced >= self.drop_backup_dist:
            self.stop()
            self._transition(State.GO_TO_DOOR)
            self._nav_go_to_door()
            return
        self._drive(-self.park_max_speed, 0.0, cap_linear=False)

    # ===== align-phase tick methods ===================================

    def _pickup_align(self) -> None:
        """Visual servoing onto the cube (perception only).

        Centre the live ``/targets/cube_live`` bearing, then approach. The
        height-based monocular range is trusted only while the whole column is
        in frame; once centred and within ``pickup_lunge_start`` we hand off to
        ``_pickup_lunge`` (a straight, odometry-measured final approach)."""
        if self._lunge_active:
            self._pickup_lunge()
            return
        if not self._cube_fresh():
            return

        bx, by, _ = self._cube_live
        rng = math.hypot(bx, by)
        bearing = math.atan2(by, bx)

        if abs(bearing) > self.align_yaw_tol:
            self._drive(0.0, self.align_kp * bearing)
            return

        if rng <= self.pickup_lunge_start:
            start = self.get_odom_pose()
            if start is None:
                return
            self._lunge_active = True
            self._lunge_start = start
            self._lunge_dist = max(
                0.0, rng - self.pickup_engage_dist - self.pickup_lunge_margin)
            self.stop()
            self.get_logger().info(
                f"pickup: {self._lunge_dist:.2f} m straight approach")
            return

        self._drive(0.5 * (rng - self.pickup_engage_dist), 0.5 * bearing)

    def _pickup_lunge(self) -> None:
        """Straight, odometry-measured final approach, then close the gripper.

        Advances ``_lunge_dist`` in the smooth odom frame; while the cube is
        still visible its bearing keeps the heading centred, then the last few
        centimetres (cube below the camera FOV) are dead-straight."""
        pose = self.get_odom_pose()
        if pose is None:
            return
        advanced = math.hypot(
            pose[0] - self._lunge_start[0], pose[1] - self._lunge_start[1])
        if advanced >= self._lunge_dist:
            self.stop()
            self._lunge_active = False
            self._engage_cube()
            return
        bearing = math.atan2(self._cube_live[1], self._cube_live[0]) \
            if self._cube_fresh() else 0.0
        self._drive(self.park_max_speed, 0.5 * bearing, cap_linear=False)

    def _cube_fresh(self) -> bool:
        return (self._cube_live is not None
                and self.clock_s() - self._cube_live[2] <= self.cube_live_timeout)

    def _drive(self, linear: float, angular: float,
               cap_linear: bool = True) -> None:
        twist = Twist()
        twist.linear.x = (_clamp(linear, -self.park_max_speed, self.park_max_speed)
                          if cap_linear else linear)
        twist.angular.z = _clamp(angular, -self.align_max_omega, self.align_max_omega)
        self.cmd_pub.publish(twist)

    def _engage_cube(self) -> None:
        """Close the gripper on the cube and advance the FSM."""
        self._transition(State.PICKUP_CLOSE)
        self.action_t = self.clock_s()
        self.gripper.close()

    # ===== timed-drive tick methods ===================================

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
            return
        rx, ry, _ = pose
        cx, cy = self.targets["cube"]
        yaw = math.atan2(cy - ry, cx - rx)
        sx = cx - self.pickup_standoff * math.cos(yaw)
        sy = cy - self.pickup_standoff * math.sin(yaw)
        self.publish_nav_goal(sx, sy, yaw)
        self.nav.send(sx, sy, yaw)

    def _nav_go_to_plate(self) -> None:
        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry, _ = pose
        px, py = self.targets["plate"]
        yaw = math.atan2(py - ry, px - rx)
        self.publish_nav_goal(px, py, yaw)
        self.nav.send(px, py, yaw)

    def _nav_go_to_door(self) -> None:
        dx, dy = self.targets["door"]
        nx, ny = self._door_outward_normal(dx, dy)
        tx = dx - self.door_threshold_inset * nx
        ty = dy - self.door_threshold_inset * ny
        yaw = math.atan2(ny, nx)
        self.get_logger().info(
            f"door threshold=({tx:.2f}, {ty:.2f}, yaw={math.degrees(yaw):+.0f}°)"
        )
        self.publish_nav_goal(tx, ty, yaw)
        self.nav.send(tx, ty, yaw)

    # ===== shared helpers ============================================

    def _door_outward_normal(self, dx: float, dy: float) -> tuple[float, float]:
        """Return cardinal outward normal of the door wall using the SLAM map.

        Scans cells in each direction from the door position.  Outside the room
        SLAM never maps, so those cells stay unknown (-1).  The direction with
        the most unknown cells just past the door is outward.
        """
        info = self.current_map.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        w, h = info.width, info.height
        data = self.current_map.data
        cx = int((dx - ox) / res)
        cy = int((dy - oy) / res)

        def unknown_count(step_x: int, step_y: int, steps: int = 6) -> int:
            count = 0
            for i in range(2, steps + 2):
                gx, gy = cx + step_x * i, cy + step_y * i
                if gx < 0 or gy < 0 or gx >= w or gy >= h:
                    count += 1
                elif data[gy * w + gx] < 0:
                    count += 1
            return count

        candidates = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        nx, ny = max(candidates, key=lambda d: unknown_count(d[0], d[1]))
        return float(nx), float(ny)

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
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))
        return float(x), float(y), float(yaw)

    def get_odom_pose(self) -> tuple[float, float] | None:
        """Robot position in the odom frame (smooth, no SLAM jumps) — used to
        measure short open-loop advances during the pickup lunge."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.1)
            )
        except Exception:
            return None
        return float(tf.transform.translation.x), float(tf.transform.translation.y)

    def clock_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_nav_goal(self, x: float, y: float, yaw: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        self.goal_pub.publish(msg)


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
