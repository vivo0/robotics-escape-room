#!/usr/bin/env python3
"""Mission FSM for the escape room.

Each state is one tick-method; the robot walks through them in order:
    EXPLORE -> GO_TO_KEY
            -> PICKUP_OPEN -> PICKUP_ALIGN -> PICKUP_CLOSE
            -> GO_TO_PLATE -> DROP_ALIGN -> DROP_OPEN -> DROP_BACKUP -> DONE
The mission ends once the key is on the pressure plate (which opens the
door); the robot itself never drives to the door.
Nav2 does the long-range driving; cmd_vel does the short precise moves.
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
    # keep x inside [lo, hi] (used to cap speeds)
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

        # "latched" = keep the last message so we still get it if we
        # subscribe after the detector already published (transient local).
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/exploration/goal", 10)
        # one callback for all three target topics; n=name binds the loop
        # variable now (otherwise every lambda would see the last name).
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

        self.targets: dict[str, tuple[float, float]] = {}  # name -> (x, y) in map
        self.mode: State = State.EXPLORE
        self.action_t: float = 0.0          # time a timed action started
        self.current_map: OccupancyGrid | None = None
        self._started: bool = False         # becomes True once Nav2/TF are up
        # Latest cube detection in base_link: (x_fwd, y_left, stamp_s).
        self._cube_live: tuple[float, float, float] | None = None
        # Odometry-measured final-approach ("lunge") state.
        self._lunge_active: bool = False
        self._lunge_start: tuple[float, float] | None = None
        self._lunge_dist: float = 0.0
        self._backup_start: tuple[float, float] | None = None

        # map each state to the method that runs every tick while in it
        self._handlers = {
            State.EXPLORE: self._explore,
            State.GO_TO_KEY: self._go_to_key,
            State.GO_TO_PLATE: self._go_to_plate,
            State.PICKUP_OPEN: self._pickup_open,
            State.PICKUP_ALIGN: self._pickup_align,
            State.PICKUP_CLOSE: self._pickup_close,
            State.DROP_ALIGN: self._drop_align,
            State.DROP_OPEN: self._drop_open,
            State.DROP_BACKUP: self._drop_backup,
            State.DONE: self._done,
        }

        # the whole FSM runs on this fixed-rate timer
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
        # the detector re-publishes a refined pose every frame; ignore small
        # jitter and only log when it actually moved (> 0.1 m)
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
        # wait until everything is up, then just run the current state's method
        if not self._started:
            if self._is_ready():
                self._started = True
                self.get_logger().info("Nav2 + map + TF ready; starting mission")
            return
        self._handlers[self.mode]()

    def _is_ready(self) -> bool:
        # need the Nav2 action server, a map, and a valid robot pose (TF)
        return (
            self.nav.server_ready
            and self.current_map is not None
            and self.get_robot_pose() is not None
        )

    # ===== nav-phase tick methods =====================================

    def _explore(self) -> None:
        # found all three landmarks? stop exploring and go grab the cube
        if len(self.targets) == 3:
            self._transition(State.GO_TO_KEY)
            self._nav_go_to_key()
            return
        if self.nav.active:        # already driving to a frontier, let it finish
            return
        # frontiers = boundary cells between known free space and the unknown
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
        # greedily head for the closest frontier, facing it
        fx, fy = min(frontiers, key=lambda f: math.hypot(f[0] - rx, f[1] - ry))
        yaw = math.atan2(fy - ry, fx - rx)
        self.publish_nav_goal(fx, fy, yaw)
        self.nav.send(fx, fy, yaw)

    def _go_to_key(self) -> None:
        # same pattern for every nav phase: wait while driving, retry on
        # failure (Nav2 sometimes aborts near walls), act once it succeeds
        if self.nav.active:
            return
        if not self.nav.succeeded:
            self.get_logger().warn("go_to_key nav failed; retrying")
            self._nav_go_to_key()
            return
        self.stop()
        self._lunge_active = False     # reset lunge state for a fresh pickup
        self._lunge_start = None
        self._transition(State.PICKUP_OPEN)
        self.action_t = self.clock_s()
        self.gripper.open()            # open early so we approach ready to grab

    def _go_to_plate(self) -> None:
        if self.nav.active:
            return
        if not self.nav.succeeded:
            self.get_logger().warn("go_to_plate nav failed; retrying")
            self._nav_go_to_plate()
            return
        self.stop()
        self._transition(State.DROP_ALIGN)   # Nav2 got us close; align precisely

    def _done(self) -> None:
        pass

    # ===== gripper-wait tick methods ==================================

    def _pickup_open(self) -> None:
        # stand still until the jaws report fully open, then start aligning
        self.stop()
        elapsed = self.clock_s() - self.action_t
        if self.gripper.is_open(elapsed, self.gripper_timeout):
            self._transition(State.PICKUP_ALIGN)

    def _pickup_close(self) -> None:
        self.stop()
        elapsed = self.clock_s() - self.action_t
        if self.gripper.is_closed(elapsed, self.gripper_timeout):
            # hide the carried cube from the lidar so it isn't mapped as an
            # obstacle stuck in front of us, then carry it to the plate
            self.gripper.set_cube_visible(False)
            self._transition(State.GO_TO_PLATE)
            self._nav_go_to_plate()

    def _drop_align(self) -> None:
        # Nav2's 0.25 m tolerance is too loose to drop on a 0.3 m plate, so
        # we finish with cmd_vel: face the plate and stop so the gripper (which
        # holds the cube ahead of us) ends up over the plate centre.
        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose
        px, py = self.targets["plate"]
        # express the plate in base_link: translate, then rotate by -yaw
        c, s = math.cos(-ryaw), math.sin(-ryaw)
        dx, dy = px - rx, py - ry
        bx, by = c * dx - s * dy, s * dx + c * dy   # x = forward, y = left
        bearing = math.atan2(by, bx)
        if abs(bearing) > self.align_yaw_tol:       # not facing it yet: turn
            self._drive(0.0, self.align_kp * bearing)
            return
        # the cube is carried ~(engage + lunge margin) ahead, so base_link must
        # stop that far from the plate for the cube to land on the centre
        carried = self.pickup_engage_dist + self.pickup_lunge_margin
        err = math.hypot(bx, by) - carried
        if abs(err) <= 0.03:                         # close enough: drop it
            self.stop()
            self._transition(State.DROP_OPEN)
            self.action_t = self.clock_s()
            self.gripper.open()
            return
        self._drive(0.5 * err, 0.5 * bearing)        # P-control on distance + yaw

    def _drop_open(self) -> None:
        self.stop()
        elapsed = self.clock_s() - self.action_t
        if not self.gripper.is_open(elapsed, self.gripper_timeout):
            return
        start = self.get_odom_pose()
        if start is None:                  # need an odom start to measure backup
            return
        self.gripper.set_cube_visible(True)   # cube is on the plate now: show it
        self._backup_start = start
        self._transition(State.DROP_BACKUP)

    def _drop_backup(self) -> None:
        # the gripper is still over the cube; if we turned now we'd knock it off
        # the plate, so first reverse straight a bit, measured in odom
        pose = self.get_odom_pose()
        if pose is None:
            return
        advanced = math.hypot(
            pose[0] - self._backup_start[0], pose[1] - self._backup_start[1])
        if advanced >= self.drop_backup_dist:
            self.stop()
            # key is on the plate and the door opens: mission complete
            self._transition(State.DONE)
            self.get_logger().info("mission complete")
            return
        self._drive(-self.park_max_speed, 0.0, cap_linear=False)  # straight back

    # ===== align-phase tick methods ===================================

    def _pickup_align(self) -> None:
        # Visual servoing on the cube: turn to centre it, drive up to it, and
        # hand off to the lunge for the last stretch (perception only, the
        # cube_live pose is already in base_link).
        if self._lunge_active:
            self._pickup_lunge()
            return
        if not self._cube_fresh():        # no recent detection: wait, don't guess
            return

        bx, by, _ = self._cube_live
        rng = math.hypot(bx, by)          # distance to cube
        bearing = math.atan2(by, bx)      # angle to cube (0 = straight ahead)

        if abs(bearing) > self.align_yaw_tol:
            self._drive(0.0, self.align_kp * bearing)   # rotate to face it first
            return

        # Close enough that the cube's base will soon clip out of the camera
        # (its pixel height shrinks → range gets unreliable). So freeze the
        # remaining distance now and finish it open-loop on odometry.
        if rng <= self.pickup_lunge_start:
            start = self.get_odom_pose()
            if start is None:
                return
            self._lunge_active = True
            self._lunge_start = start
            # leave a small margin so we stop just before the cube
            self._lunge_dist = max(
                0.0, rng - self.pickup_engage_dist - self.pickup_lunge_margin)
            self.stop()
            self.get_logger().info(
                f"pickup: {self._lunge_dist:.2f} m straight approach")
            return

        # still far: drive forward (capped) while keeping centred
        self._drive(0.5 * (rng - self.pickup_engage_dist), 0.5 * bearing)

    def _pickup_lunge(self) -> None:
        # Drive straight until odometry says we've advanced _lunge_dist, then
        # close. Odom is used (not the map/SLAM pose) because it's smooth over
        # this short move; the camera range can't be trusted this close.
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
        # keep nudging the heading while the cube is still visible, else go straight
        bearing = math.atan2(self._cube_live[1], self._cube_live[0]) \
            if self._cube_fresh() else 0.0
        self._drive(self.park_max_speed, 0.5 * bearing, cap_linear=False)

    def _cube_fresh(self) -> bool:
        # True if we have a cube detection from within the last timeout window
        return (self._cube_live is not None
                and self.clock_s() - self._cube_live[2] <= self.cube_live_timeout)

    def _drive(self, linear: float, angular: float,
               cap_linear: bool = True) -> None:
        # publish one velocity command, clamping so we never go too fast
        twist = Twist()
        twist.linear.x = (_clamp(linear, -self.park_max_speed, self.park_max_speed)
                          if cap_linear else linear)
        twist.angular.z = _clamp(angular, -self.align_max_omega, self.align_max_omega)
        self.cmd_pub.publish(twist)

    def _engage_cube(self) -> None:
        # close the jaws and move on to wait for them to finish closing
        self._transition(State.PICKUP_CLOSE)
        self.action_t = self.clock_s()
        self.gripper.close()

    # ===== state transition + nav goal helpers =======================

    def _transition(self, state: State) -> None:
        # cancel any running Nav2 goal so it can't fire after we've moved on
        self.get_logger().info(f"{self.mode} → {state}")
        self.nav.cancel()
        self.mode = state

    def _nav_go_to_key(self) -> None:
        pose = self.get_robot_pose()
        if pose is None:
            return
        rx, ry, _ = pose
        cx, cy = self.targets["cube"]
        yaw = math.atan2(cy - ry, cx - rx)          # heading from robot to cube
        # don't drive onto the cube: stop pickup_standoff short of it, facing it
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

    # ===== shared helpers ============================================

    def stop(self) -> None:
        self.cmd_pub.publish(Twist())     # empty Twist = zero velocity

    def get_robot_pose(self) -> tuple[float, float, float] | None:
        # robot pose in the map frame (from SLAM via TF); None if TF not ready
        try:
            tf = self._tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.1)
            )
        except Exception:
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        # yaw from quaternion (z-axis rotation only, the robot is planar)
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))
        return float(x), float(y), float(yaw)

    def get_odom_pose(self) -> tuple[float, float] | None:
        # position in the odom frame: drifts slowly but is smooth (no SLAM
        # jumps), which is what we want for measuring a short straight move
        try:
            tf = self._tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.1)
            )
        except Exception:
            return None
        return float(tf.transform.translation.x), float(tf.transform.translation.y)

    def clock_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9   # ROS time in seconds

    def publish_nav_goal(self, x: float, y: float, yaw: float) -> None:
        # publish the goal as an arrow for RViz (Nav2 itself is sent via NavClient)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        # yaw -> quaternion (again only z because the robot is flat on the floor)
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
