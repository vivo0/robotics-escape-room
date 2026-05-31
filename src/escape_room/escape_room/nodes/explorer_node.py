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
        self._lunge_t0: float = 0.0

        self._handlers = {
            State.EXPLORE: self._explore,
            State.GO_TO_KEY: self._go_to_key,
            State.GO_TO_PLATE: self._go_to_plate,
            State.GO_TO_DOOR: self._go_to_door,
            State.PICKUP_OPEN: self._pickup_open,
            State.PICKUP_ALIGN: self._pickup_align,
            State.PICKUP_CLOSE: self._pickup_close,
            State.DROP_OPEN: self._drop_open,
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
        p("pickup_engage_dist_tol_m", 0.03)
        p("park_max_speed_mps", 0.06)
        p("align_yaw_tol_rad", 0.08)
        p("align_kp", 1.5)
        p("align_max_omega", 0.6)
        p("gripper_timeout_s", 4.0)
        # Visual-servoing pickup (perception-only, no ground truth).
        p("cube_live_timeout_s", 0.8)
        p("pickup_search_omega", 0.3)
        # Range at which we stop trusting the (height-based) monocular depth
        # and commit to an odometry-measured straight final approach. The
        # cube is a 0.20 m column; nearer than this its base clips out of the
        # camera's bottom edge, shrinking its pixel height and inflating the
        # estimated range — which would drive the robot through the cube.
        p("pickup_lunge_start_m", 0.90)
        # Stop the straight lunge this much short of the computed engage
        # distance, so the gripper closes just before reaching the cube
        # rather than nudging it forward.
        p("pickup_lunge_margin_m", 0.05)

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
        self.pickup_engage_dist_tol = float(g("pickup_engage_dist_tol_m"))
        self.park_max_speed = float(g("park_max_speed_mps"))
        self.align_yaw_tol = float(g("align_yaw_tol_rad"))
        self.align_kp = float(g("align_kp"))
        self.align_max_omega = float(g("align_max_omega"))
        self.gripper_timeout = float(g("gripper_timeout_s"))
        self.cube_live_timeout = float(g("cube_live_timeout_s"))
        self.pickup_search_omega = float(g("pickup_search_omega"))
        self.pickup_lunge_start = float(g("pickup_lunge_start_m"))
        self.pickup_lunge_margin = float(g("pickup_lunge_margin_m"))

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
        self._transition(State.DROP_OPEN)
        self.action_t = self.clock_s()
        self.gripper.open()

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
            self._transition(State.GO_TO_DOOR)
            self._nav_go_to_door()

    # ===== align-phase tick methods ===================================

    def _pickup_align(self) -> None:
        """Visual servoing onto the cube using perception only.

        Drives on the live ``/targets/cube_live`` bearing (base_link): turns
        to centre the cube, then approaches. The monocular range (from the
        cube's pixel height) is trustworthy only while the whole column is in
        frame; closer than ``pickup_lunge_start`` its base clips out of the
        bottom edge and the range balloons. So once centred and within that
        range we commit to ``_pickup_lunge``: a straight, odometry-measured
        final approach that ignores the (now unreliable) range. No
        map/ground-truth cube coordinate is used."""
        if self._lunge_active:
            self._pickup_lunge()
            return

        engage = self.gripper.pickup_engage_dist
        now = self.clock_s()
        fresh = (
            self._cube_live is not None
            and now - self._cube_live[2] <= self.cube_live_timeout
        )
        if not fresh:
            # Lost / never acquired: rotate slowly to search.
            twist = Twist()
            twist.angular.z = self.pickup_search_omega
            self.cmd_pub.publish(twist)
            return

        bx, by, _ = self._cube_live
        rng = math.hypot(bx, by)
        bearing = math.atan2(by, bx)

        if abs(bearing) > self.align_yaw_tol:
            twist = Twist()
            twist.angular.z = _clamp(
                self.align_kp * bearing,
                -self.align_max_omega,
                self.align_max_omega,
            )
            self.cmd_pub.publish(twist)
            return

        # Centred and close enough: commit to the odometry-measured lunge.
        if rng <= self.pickup_lunge_start:
            pose = self.get_odom_pose()
            if pose is None:
                self.stop()
                self.get_logger().warn("pickup: odom TF unavailable, waiting")
                return
            self._lunge_active = True
            self._lunge_start = (pose[0], pose[1])
            self._lunge_dist = max(0.0, rng - engage - self.pickup_lunge_margin)
            self._lunge_t0 = now
            self.stop()
            self.get_logger().info(
                f"pickup: committing to {self._lunge_dist:.2f} m straight "
                f"approach (range {rng:.2f} m)")
            return

        # Approach forward while keeping centred.
        twist = Twist()
        twist.linear.x = _clamp(
            0.5 * (rng - engage), -self.park_max_speed, self.park_max_speed
        )
        twist.angular.z = 0.5 * bearing
        self.cmd_pub.publish(twist)

    def _pickup_lunge(self) -> None:
        """Straight, odometry-measured final approach to the cube.

        Drives forward until the robot has advanced ``_lunge_dist`` (measured
        in the smooth ``odom`` frame), then closes the gripper. While the cube
        is still visible its bearing keeps steering the heading; for the last
        few centimetres (cube below the camera FOV) it drives dead-straight.
        A time-out guards against a stalled odom lookup."""
        now = self.clock_s()
        pose = self.get_odom_pose()
        advanced = 0.0
        if pose is not None and self._lunge_start is not None:
            advanced = math.hypot(
                pose[0] - self._lunge_start[0], pose[1] - self._lunge_start[1])
        timeout = self._lunge_dist / max(self.park_max_speed, 1e-3) + 4.0

        if advanced >= self._lunge_dist or now - self._lunge_t0 > timeout:
            self.stop()
            self._lunge_active = False
            self._engage_cube()
            return

        twist = Twist()
        twist.linear.x = self.park_max_speed
        if (self._cube_live is not None
                and now - self._cube_live[2] <= self.cube_live_timeout):
            bx, by, _ = self._cube_live
            twist.angular.z = _clamp(
                0.5 * math.atan2(by, bx),
                -self.align_max_omega,
                self.align_max_omega,
            )
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
            self.get_logger().warn("go_to_key: TF unavailable, nav goal deferred")
            return
        rx, ry, _ = pose
        cx, cy = self.targets["cube"]
        yaw = math.atan2(cy - ry, cx - rx)
        sx = cx - self.pickup_standoff * math.cos(yaw)
        sy = cy - self.pickup_standoff * math.sin(yaw)
        self.publish_nav_goal(sx, sy, yaw)
        self.nav.send(sx, sy, yaw)

    def _nav_go_to_plate(self) -> None:
        px, py = self.targets["plate"]
        pose = self.get_robot_pose()
        if pose is not None:
            rx, ry, _ = pose
            yaw = math.atan2(py - ry, px - rx)
        else:
            yaw = 0.0
            self.get_logger().warn("go_to_plate: TF unavailable, yaw=0.0")
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
