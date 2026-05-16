#!/usr/bin/env python3
"""Mission state machine for the escape room.

Navigation is delegated to Nav2 (NavigateToPose action).  Exploration uses
frontier detection on the live /map occupancy grid from slam_toolbox.
Robot pose is read from the map→base_link TF published by slam_toolbox.

Gripper control still uses the CoppeliaSim ZMQ API (Lua helpers injected by
build_scene.py), as does cube attach/detach.

State sequence:

    explore
      → go_to_key
      → pickup_open → pickup_align → pickup_drive → pickup_close
      → go_to_plate
      → drop_align → drop_open → drop_backup
      → go_to_door
      → done
"""

from __future__ import annotations

import math

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path as PathMsg
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

# Gripper state values mirror robomaster_msgs/action/GripperControl.
GRIPPER_OPEN = 1
GRIPPER_CLOSE = 2

# Minimum frontier cluster size (cells) to filter noise in the occupancy grid.
_MIN_FRONTIER_CELLS = 5


def wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x: float, lo: float, hi: float) -> float:
    """Clamp x to [lo, hi]."""
    return max(lo, min(hi, x))


# ── GripperIO ──────────────────────────────────────────────────────────────


class GripperIO:
    """Stateless ZMQ wrapper for gripper open/close and cube attach/detach."""

    def __init__(self, sim, gripper_script_h: int, cube_h: int, attach_h: int) -> None:
        """Initialise with CoppeliaSim sim handle and pre-resolved object handles."""
        self._sim = sim
        self._script_h = gripper_script_h
        self._cube_h = cube_h
        self._attach_h = attach_h

    def open(self) -> None:
        """Command gripper to open."""
        self._sim.callScriptFunction("_ext_set_target", self._script_h, GRIPPER_OPEN)

    def close(self) -> None:
        """Command gripper to close."""
        self._sim.callScriptFunction("_ext_set_target", self._script_h, GRIPPER_CLOSE)

    def reached(self, target: int, elapsed_s: float, timeout_s: float, logger) -> bool:
        """Return True when gripper reports target state, or on timeout."""
        if elapsed_s >= timeout_s:
            logger.warn(f"gripper timeout waiting for state {target}")
            return True
        cur = self._sim.callScriptFunction("_ext_get_state", self._script_h)
        return cur is not None and int(cur) == target

    def attach_cube(self) -> None:
        """Parent cube to gripper attach point."""
        self._sim.setObjectParent(self._cube_h, self._attach_h, True)
        self._sim.resetDynamicObject(self._cube_h)

    def detach_cube(self) -> None:
        """Release cube to world frame."""
        self._sim.setObjectParent(self._cube_h, -1, True)
        self._sim.resetDynamicObject(self._cube_h)


# ── NavClient ─────────────────────────────────────────────────────────────


class NavClient:
    """Nav2 NavigateToPose action client."""

    def __init__(self, node: Node, map_frame: str) -> None:
        """Initialise action client."""
        self._node = node
        self._map_frame = map_frame
        self._client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self._active = False
        self._handle = None

    @property
    def active(self) -> bool:
        """True while a Nav2 goal is in flight."""
        return self._active

    @property
    def server_ready(self) -> bool:
        """True when the Nav2 action server has been discovered."""
        return self._client.wait_for_server(timeout_sec=0.0)

    def send(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """Send a NavigateToPose goal. Returns False if server unavailable."""
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
        """Cancel any active goal."""
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


# ── ExplorerNode ──────────────────────────────────────────────────────────


# TODO: remove zmq from color detection
# TODO: deslop configs
class ExplorerNode(Node):
    """Mission FSM: sends Nav2 goals and drives gripper via ZMQ."""

    def __init__(self) -> None:
        """Initialise node, ZMQ, Nav2 client, and mission state."""
        super().__init__("explorer_node")

        # ----- parameters --------------------------------------------
        p = self.declare_parameter
        p("robot_alias", "/RoboMasterEP/BaseLinkFrame")
        p("cube_alias", "/TargetCube")
        p("map_frame", "map")
        p("base_frame", "base_link")
        p("control_rate_hz", 4.0)
        p("door_push_m", 0.6)
        p("pickup_standoff_m", 0.50)
        p("pickup_engage_dist_m", 0.15)
        p("pickup_engage_dist_tol_m", 0.03)
        p("engage_speed_mps", 0.05)
        p("engage_duration_s", 0.4)
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

        self._map_frame = g("map_frame")
        self._base_frame = g("base_frame")
        self.door_push = float(g("door_push_m"))
        self.pickup_standoff = float(g("pickup_standoff_m"))
        self.pickup_engage_dist = float(g("pickup_engage_dist_m"))
        self.pickup_engage_dist_tol = float(g("pickup_engage_dist_tol_m"))
        self.engage_speed = float(g("engage_speed_mps"))
        self.engage_duration = float(g("engage_duration_s"))
        self.backup_speed = float(g("drop_backup_speed_mps"))
        self.backup_duration = float(g("drop_backup_duration_s"))
        self.drop_distance = float(g("plate_drop_distance_m"))
        self.drop_dist_tol = float(g("plate_drop_dist_tol_m"))
        self.park_max_speed = float(g("park_max_speed_mps"))
        self.align_yaw_tol = float(g("align_yaw_tol_rad"))
        self.align_kp = float(g("align_kp"))
        self.align_max_omega = float(g("align_max_omega"))
        self.gripper_timeout = float(g("gripper_timeout_s"))

        # ----- ZMQ (gripper + cube attach only) ----------------------
        sim = RemoteAPIClient().require("sim")
        robot_alias = g("robot_alias")
        model_alias = "/" + robot_alias.lstrip("/").split("/")[0]
        model_h = sim.getObject(model_alias)
        cube_h = sim.getObject(g("cube_alias"))
        attach_h = self._find_in_tree(sim, model_h, "attachPoint")
        gripper_h = self._find_in_tree(sim, model_h, "gripper_link_respondable")
        script_h = sim.getScript(1, gripper_h)
        self._gripper = GripperIO(sim, script_h, cube_h, attach_h)

        # ----- TF (robot pose in map frame) --------------------------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ----- Nav2 --------------------------------------------------
        self._nav = NavClient(self, self._map_frame)

        # ----- ROS pub/sub -------------------------------------------
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

        # ----- mission state -----------------------------------------
        self.targets: dict[str, tuple[float, float]] = {}
        self.mode = "explore"
        self.action_t = 0.0
        self._current_map: OccupancyGrid | None = None
        self._started = False

        self._handlers = {
            "explore": self._tick_navigate,
            "go_to_key": self._tick_navigate,
            "go_to_plate": self._tick_navigate,
            "go_to_door": self._tick_navigate,
            "pickup_open": self._tick_gripper_wait,
            "pickup_align": self._tick_pickup_align,
            "pickup_drive": self._tick_engage,
            "pickup_close": self._tick_gripper_wait,
            "drop_open": self._tick_gripper_wait,
            "drop_align": self._tick_drop_align,
            "drop_backup": self._tick_drop_backup,
            "done": self._stop,
        }

        self.create_timer(1.0 / float(g("control_rate_hz")), self._tick)
        self.get_logger().info("ready; waiting for Nav2 and slam_toolbox...")

    # ===== ROS callbacks =============================================

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._current_map = msg

    def _on_target(self, name: str, msg: PoseStamped) -> None:
        if name in self.targets:
            return
        self.targets[name] = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(
            f"saw '{name}' at ({self.targets[name][0]:.2f}, "
            f"{self.targets[name][1]:.2f}) [{len(self.targets)}/3]"
        )
        if self.mode == "explore" and len(self.targets) == 3:
            self._enter("go_to_key")

    # ===== main loop =================================================

    def _tick(self) -> None:
        if not self._started:
            if self._is_ready():
                self._started = True
                self.get_logger().info("Nav2 + map + TF ready; starting mission")
            return
        self._handlers[self.mode]()

    def _is_ready(self) -> bool:
        if not self._nav.server_ready:
            return False
        if self._current_map is None:
            return False
        if self._get_robot_pose() is None:
            return False
        return True

    # ===== navigation tick ===========================================

    def _tick_navigate(self) -> None:
        if self._nav.active:
            return
        if self.mode == "explore":
            if len(self.targets) == 3:
                self._enter("go_to_key")
                return
            self._send_frontier_goal()
        elif self.mode == "go_to_key":
            self._begin_pickup()
        elif self.mode == "go_to_plate":
            self._begin_drop()
        elif self.mode == "go_to_door":
            self._enter("done")
            self._stop()

    def _send_frontier_goal(self) -> None:
        """Navigate to the nearest unexplored frontier on the current map."""
        if self._current_map is None:
            return
        frontiers = self._compute_frontiers(self._current_map)
        if not frontiers:
            self.get_logger().info("no frontiers; map fully explored")
            return
        pose = self._get_robot_pose()
        if pose is None:
            return
        rx, ry, _ = pose
        fx, fy = min(frontiers, key=lambda f: math.hypot(f[0] - rx, f[1] - ry))
        self._publish_nav_goal_path(fx, fy)
        self._nav.send(fx, fy)

    def _compute_frontiers(self, grid: OccupancyGrid) -> list[tuple[float, float]]:
        """Return world-frame centroids of frontier clusters.

        A frontier cell is a free cell (0) with at least one unknown (-1)
        4-neighbour.  Adjacent frontier cells are merged into clusters; only
        clusters with >= _MIN_FRONTIER_CELLS cells are returned.
        """
        w = grid.info.width
        h = grid.info.height
        data = grid.data
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y

        frontier_set: set[tuple[int, int]] = set()
        for r in range(1, h - 1):
            for c in range(1, w - 1):
                if data[r * w + c] != 0:
                    continue
                for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    if data[(r + dr) * w + (c + dc)] == -1:
                        frontier_set.add((r, c))
                        break

        visited: set[tuple[int, int]] = set()
        centroids: list[tuple[float, float]] = []
        for seed in frontier_set:
            if seed in visited:
                continue
            cluster: list[tuple[int, int]] = []
            stack = [seed]
            visited.add(seed)
            while stack:
                r, c = stack.pop()
                cluster.append((r, c))
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        nb = (r + dr, c + dc)
                        if nb in frontier_set and nb not in visited:
                            visited.add(nb)
                            stack.append(nb)
            if len(cluster) >= _MIN_FRONTIER_CELLS:
                cr = sum(r for r, _ in cluster) / len(cluster)
                cc = sum(c for _, c in cluster) / len(cluster)
                centroids.append((ox + (cc + 0.5) * res, oy + (cr + 0.5) * res))
        return centroids

    # ===== pickup substates ==========================================

    def _begin_pickup(self) -> None:
        self._stop()
        self._enter("pickup_open")
        self.action_t = self._clock_s()
        self._gripper.open()

    def _tick_pickup_align(self) -> None:
        """P-controller: face cube, approach to pickup_engage_dist."""
        pose = self._get_robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose
        cx, cy = self.targets["cube"]
        dist = math.hypot(cx - rx, cy - ry)
        target_yaw = math.atan2(cy - ry, cx - rx)
        yaw_err = wrap_angle(target_yaw - ryaw)

        if abs(yaw_err) > self.align_yaw_tol:
            twist = Twist()
            twist.angular.z = clamp(
                self.align_kp * yaw_err, -self.align_max_omega, self.align_max_omega
            )
            self.cmd_pub.publish(twist)
            return

        dist_err = dist - self.pickup_engage_dist
        if abs(dist_err) <= self.pickup_engage_dist_tol:
            self._stop()
            self._enter("pickup_drive")
            self.action_t = self._clock_s()
            return
        twist = Twist()
        twist.linear.x = clamp(
            0.5 * dist_err, -self.park_max_speed, self.park_max_speed
        )
        twist.angular.z = 0.5 * yaw_err
        self.cmd_pub.publish(twist)

    def _tick_engage(self) -> None:
        if self._clock_s() - self.action_t >= self.engage_duration:
            self._stop()
            self._enter("pickup_close")
            self.action_t = self._clock_s()
            self._gripper.close()
            return
        twist = Twist()
        twist.linear.x = self.engage_speed
        self.cmd_pub.publish(twist)

    # ===== drop substates ============================================

    def _begin_drop(self) -> None:
        self._stop()
        self._enter("drop_align")

    def _tick_drop_align(self) -> None:
        """P-controller: face plate, approach to drop_distance."""
        pose = self._get_robot_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose
        px, py = self.targets["plate"]
        dist = math.hypot(px - rx, py - ry)
        target_yaw = math.atan2(py - ry, px - rx)
        yaw_err = wrap_angle(target_yaw - ryaw)

        if abs(yaw_err) > self.align_yaw_tol:
            twist = Twist()
            twist.angular.z = clamp(
                self.align_kp * yaw_err, -self.align_max_omega, self.align_max_omega
            )
            self.cmd_pub.publish(twist)
            return

        dist_err = dist - self.drop_distance
        if abs(dist_err) <= self.drop_dist_tol:
            self._stop()
            self._enter("drop_open")
            self.action_t = self._clock_s()
            self._gripper.open()
            return
        twist = Twist()
        twist.linear.x = clamp(
            0.5 * dist_err, -self.park_max_speed, self.park_max_speed
        )
        twist.angular.z = 0.5 * yaw_err
        self.cmd_pub.publish(twist)

    def _tick_drop_backup(self) -> None:
        if self._clock_s() - self.action_t >= self.backup_duration:
            self._stop()
            self._enter("go_to_door")
            return
        twist = Twist()
        twist.linear.x = -self.backup_speed
        self.cmd_pub.publish(twist)

    # ===== gripper-wait substates ====================================

    def _tick_gripper_wait(self) -> None:
        self._stop()
        elapsed = self._clock_s() - self.action_t
        logger = self.get_logger()
        if self.mode == "pickup_open" and self._gripper.reached(
            GRIPPER_OPEN, elapsed, self.gripper_timeout, logger
        ):
            self._enter("pickup_align")
        elif self.mode == "pickup_close" and self._gripper.reached(
            GRIPPER_CLOSE, elapsed, self.gripper_timeout, logger
        ):
            self._gripper.attach_cube()
            self._enter("go_to_plate")
        elif self.mode == "drop_open" and self._gripper.reached(
            GRIPPER_OPEN, elapsed, self.gripper_timeout, logger
        ):
            self._gripper.detach_cube()
            self._enter("drop_backup")
            self.action_t = self._clock_s()

    # ===== state transitions =========================================

    def _enter(self, mode: str) -> None:
        self.get_logger().info(f"mode: {self.mode} → {mode}")
        self._nav.cancel()
        self.mode = mode
        if mode == "go_to_key":
            cx, cy = self.targets["cube"]
            pose = self._get_robot_pose()
            if pose is not None:
                rx, ry, _ = pose
                yaw_to_cube = math.atan2(cy - ry, cx - rx)
                sx = cx - self.pickup_standoff * math.cos(yaw_to_cube)
                sy = cy - self.pickup_standoff * math.sin(yaw_to_cube)
                self._publish_nav_goal_path(sx, sy)
                self._nav.send(sx, sy, yaw=yaw_to_cube)
            else:
                self._publish_nav_goal_path(cx, cy)
                self._nav.send(cx, cy)
        elif mode == "go_to_plate":
            px, py = self.targets["plate"]
            self._publish_nav_goal_path(px, py)
            self._nav.send(px, py)
        elif mode == "go_to_door":
            dx, dy = self._door_exit_xy()
            self._publish_nav_goal_path(dx, dy)
            self._nav.send(dx, dy)

    def _stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _publish_nav_goal_path(self, x: float, y: float) -> None:
        msg = PathMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.w = 1.0
        msg.poses = [ps]
        self.path_pub.publish(msg)

    # ===== helpers ===================================================

    def _get_robot_pose(self) -> tuple[float, float, float] | None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time(), timeout=Duration(seconds=0.1)
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2)
            )
            return float(x), float(y), float(yaw)
        except Exception:
            return None

    def _clock_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _door_exit_xy(self) -> tuple[float, float]:
        dx, dy = self.targets["door"]
        n = math.hypot(dx, dy)
        if n < 1e-3:
            return dx, dy
        return dx + self.door_push * dx / n, dy + self.door_push * dy / n

    @staticmethod
    def _find_in_tree(sim, root_h: int, alias: str) -> int:
        for h in sim.getObjectsInTree(root_h):
            if sim.getObjectAlias(int(h), 0) == alias:
                return int(h)
        raise RuntimeError(f"object not found in robot tree: {alias}")


def main(args=None) -> None:
    """Entry point."""
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
