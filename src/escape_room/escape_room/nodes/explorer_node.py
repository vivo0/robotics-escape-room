#!/usr/bin/env python3
"""Mission state machine for the escape room.

State sequence:

    explore
      → go_to_key
      → pickup_open → pickup_drive → pickup_close
      → go_to_plate
      → drop_align → drop_open → drop_backup
      → go_to_door
      → done

Pose source is the CoppeliaSim ZMQ API (exact pose, no TF lag). The
gripper is driven by calling Lua helpers (``_ext_set_target`` /
``_ext_get_state``) injected into the gripper child script by
``build_scene.py``: the native CoppeliaSim signal API is per-script
in modern versions, so writing the gripper signal from Python ZMQ
never reaches the model's own reader.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from nav_msgs.msg import Path as PathMsg

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from escape_room.exploration import find_frontiers
from escape_room.mapping.occupancy_grid import (FREE, OCCUPIED,
                                                  GridSpec, OccupancyGrid)
from escape_room.planning import PurePursuit, plan_path
from escape_room.planning.pure_pursuit import PurePursuitConfig


# Gripper state values mirror robomaster_msgs/action/GripperControl.
GRIPPER_OPEN = 1
GRIPPER_CLOSE = 2


def yaw_from_pose_matrix(mat) -> float:
    R = np.array(mat, dtype=np.float64).reshape(3, 4)[:, :3]
    return math.atan2(R[1, 0], R[0, 0])


def occupancy_msg_to_grid(msg: OccupancyGridMsg) -> OccupancyGrid:
    spec = GridSpec(
        width_m=msg.info.width * msg.info.resolution,
        height_m=msg.info.height * msg.info.resolution,
        resolution=msg.info.resolution,
        origin_x=msg.info.origin.position.x,
        origin_y=msg.info.origin.position.y,
    )
    grid = OccupancyGrid(spec)
    arr = np.array(msg.data, dtype=np.int8).reshape(
        msg.info.height, msg.info.width)
    grid.free[arr == FREE] = True
    grid.occ[arr == OCCUPIED] = True
    return grid


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class ExplorerNode(Node):

    def __init__(self) -> None:
        super().__init__('explorer_node')

        # ----- parameters --------------------------------------------
        p = self.declare_parameter
        p('robot_alias',            '/RoboMasterEP/BaseLinkFrame')
        p('cube_alias',             '/TargetCube')
        p('map_frame',              'world')
        p('control_rate_hz',        4.0)
        p('replan_period_s',        1.0)
        p('robot_radius_m',         0.20)
        p('frontier_min_size',      4)
        p('arrival_tol_m',          0.30)
        p('plate_arrival_tol_m',    0.55)
        p('door_push_m',            0.6)
        p('engage_speed_mps',       0.05)
        p('engage_duration_s',      1.6)
        p('drop_backup_speed_mps',  0.05)
        p('drop_backup_duration_s', 8.0)
        p('plate_drop_distance_m',  0.30)
        p('plate_drop_dist_tol_m',  0.04)
        p('park_max_speed_mps',     0.06)
        p('align_yaw_tol_rad',      0.08)
        p('align_kp',               1.5)
        p('align_max_omega',        0.6)
        p('gripper_timeout_s',      4.0)

        g = lambda n: self.get_parameter(n).value
        self.map_frame           = g('map_frame')
        self.robot_radius        = float(g('robot_radius_m'))
        self.replan_period       = float(g('replan_period_s'))
        self.frontier_min_size   = int(g('frontier_min_size'))
        self.arrival_tol         = float(g('arrival_tol_m'))
        self.plate_arrival_tol   = float(g('plate_arrival_tol_m'))
        self.door_push           = float(g('door_push_m'))
        self.engage_speed        = float(g('engage_speed_mps'))
        self.engage_duration     = float(g('engage_duration_s'))
        self.backup_speed        = float(g('drop_backup_speed_mps'))
        self.backup_duration     = float(g('drop_backup_duration_s'))
        self.drop_distance       = float(g('plate_drop_distance_m'))
        self.drop_dist_tol       = float(g('plate_drop_dist_tol_m'))
        self.park_max_speed      = float(g('park_max_speed_mps'))
        self.align_yaw_tol       = float(g('align_yaw_tol_rad'))
        self.align_kp            = float(g('align_kp'))
        self.align_max_omega     = float(g('align_max_omega'))
        self.gripper_timeout     = float(g('gripper_timeout_s'))

        # ----- sim handles -------------------------------------------
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')

        robot_alias = g('robot_alias')
        model_alias = '/' + robot_alias.lstrip('/').split('/')[0]
        self.robot_h = self.sim.getObject(robot_alias)
        model_h = self.sim.getObject(model_alias)
        self.cube_h = self.sim.getObject(g('cube_alias'))
        self.attach_h = self._find_in_tree(model_h, 'attachPoint')
        gripper_link_h = self._find_in_tree(model_h, 'gripper_link_respondable')
        self.gripper_script_h = self.sim.getScript(1, gripper_link_h)

        # ----- ROS pub/sub -------------------------------------------
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGridMsg, '/map', self._on_map, latched)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(
            PathMsg, '/exploration/path', 10)
        self.frontiers_pub = self.create_publisher(
            PoseArray, '/exploration/frontiers', 10)
        for name in ('cube', 'plate', 'door'):
            self.create_subscription(
                PoseStamped, f'/targets/{name}',
                lambda m, n=name: self._on_target(n, m), latched)

        # ----- mission state -----------------------------------------
        self.targets: dict[str, tuple[float, float]] = {}
        self.grid: OccupancyGrid | None = None
        self.planner: PurePursuit | None = None
        self.last_replan_t = 0.0
        self.blacklist: set[tuple[int, int]] = set()
        self.mode = 'explore'
        self.action_t = 0.0     # generic phase timer

        # Mode → handler. Built once; bound methods include ``self``.
        self._handlers = {
            'explore':       self._tick_navigate,
            'go_to_key':     self._tick_navigate,
            'go_to_plate':   self._tick_navigate,
            'go_to_door':    self._tick_navigate,
            'pickup_open':   self._tick_gripper_wait,
            'pickup_close':  self._tick_gripper_wait,
            'drop_open':     self._tick_gripper_wait,
            'pickup_drive':  self._tick_engage,
            'drop_align':    self._tick_drop_align,
            'drop_backup':   self._tick_drop_backup,
            'done':          self._stop,
        }

        self.create_timer(1.0 / float(g('control_rate_hz')), self._tick)
        self.get_logger().info(
            f'ready; robot_radius={self.robot_radius:.2f} m, '
            f'replan {self.replan_period:.1f} s')

    # ===== handle resolution =========================================

    def _find_in_tree(self, root_h: int, alias: str) -> int:
        for h in self.sim.getObjectsInTree(root_h):
            if self.sim.getObjectAlias(int(h), 0) == alias:
                return int(h)
        raise RuntimeError(f'object not found in robot tree: {alias}')

    # ===== ROS callbacks =============================================

    def _on_map(self, msg: OccupancyGridMsg) -> None:
        self.grid = occupancy_msg_to_grid(msg)

    def _on_target(self, name: str, msg: PoseStamped) -> None:
        if name in self.targets:
            return
        self.targets[name] = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(
            f"saw '{name}' at ({self.targets[name][0]:.2f}, "
            f"{self.targets[name][1]:.2f}) [{len(self.targets)}/3]")
        if self.mode == 'explore' and len(self.targets) == 3:
            self._enter('go_to_key')

    # ===== main loop =================================================

    def _tick(self) -> None:
        self._handlers[self.mode]()

    # --- shared navigation tick (explore/go_to_key/plate/door) -------

    def _tick_navigate(self) -> None:
        if self.grid is None:
            return
        pose = self._pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        if self.mode == 'go_to_key' and self._dist(rx, ry, self.targets['cube']) <= self.arrival_tol:
            self._begin_pickup()
            return
        if self.mode == 'go_to_plate' and self._dist(rx, ry, self.targets['plate']) <= self.plate_arrival_tol:
            self._begin_drop()
            return
        if self.mode == 'go_to_door' and self._dist(rx, ry, self._door_exit_xy()) <= self.arrival_tol:
            self._enter('done')
            self._stop()
            self.planner = None
            return

        now = self._clock_s()
        if self._needs_replan(now, rx, ry):
            self._replan(rx, ry)
            self.last_replan_t = now

        if self.planner is None:
            self._stop()
            return

        v, w = self.planner.step(rx, ry, ryaw)
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)

    # --- pickup substates --------------------------------------------

    def _begin_pickup(self) -> None:
        self._stop()
        self.planner = None
        self._enter('pickup_open')
        self.action_t = self._clock_s()
        self._set_gripper(GRIPPER_OPEN)

    def _tick_engage(self) -> None:
        """Slow forward creep so the cube slips between the fingers."""
        if self._clock_s() - self.action_t >= self.engage_duration:
            self._stop()
            self._enter('pickup_close')
            self.action_t = self._clock_s()
            self._set_gripper(GRIPPER_CLOSE)
            return
        twist = Twist()
        twist.linear.x = self.engage_speed
        self.cmd_pub.publish(twist)

    # --- drop substates ----------------------------------------------

    def _begin_drop(self) -> None:
        self._stop()
        self.planner = None
        self._enter('drop_align')

    def _tick_drop_align(self) -> None:
        """Two-phase parking: rotate to face the plate, then advance/
        retreat until the robot is exactly ``drop_distance`` from the
        plate (gripper offset). Then transition to ``drop_open``."""
        pose = self._pose()
        if pose is None:
            return
        rx, ry, ryaw = pose
        px, py = self.targets['plate']
        dist = math.hypot(px - rx, py - ry)
        target_yaw = math.atan2(py - ry, px - rx)
        yaw_err = wrap_angle(target_yaw - ryaw)

        if abs(yaw_err) > self.align_yaw_tol:
            twist = Twist()
            twist.angular.z = clamp(
                self.align_kp * yaw_err,
                -self.align_max_omega, self.align_max_omega)
            self.cmd_pub.publish(twist)
            return

        dist_err = dist - self.drop_distance
        if abs(dist_err) <= self.drop_dist_tol:
            self._stop()
            self._enter('drop_open')
            self.action_t = self._clock_s()
            self._set_gripper(GRIPPER_OPEN)
            return
        twist = Twist()
        twist.linear.x = clamp(
            0.5 * dist_err, -self.park_max_speed, self.park_max_speed)
        twist.angular.z = 0.5 * yaw_err
        self.cmd_pub.publish(twist)

    def _tick_drop_backup(self) -> None:
        """Reverse to clear the gripper of the cube before the rotation
        that the next state's pure-pursuit path will start with."""
        if self._clock_s() - self.action_t >= self.backup_duration:
            self._stop()
            self._enter('go_to_door')
            self.planner = None
            return
        twist = Twist()
        twist.linear.x = -self.backup_speed
        self.cmd_pub.publish(twist)

    # --- gripper state-wait (pickup_open / pickup_close / drop_open) -

    def _tick_gripper_wait(self) -> None:
        self._stop()
        if self.mode == 'pickup_open' and self._gripper_reached(GRIPPER_OPEN):
            self._enter('pickup_drive')
            self.action_t = self._clock_s()
        elif self.mode == 'pickup_close' and self._gripper_reached(GRIPPER_CLOSE):
            self._attach_cube()
            self._enter('go_to_plate')
            self.planner = None
        elif self.mode == 'drop_open' and self._gripper_reached(GRIPPER_OPEN):
            self._detach_cube()
            self._enter('drop_backup')
            self.action_t = self._clock_s()

    # ===== sim plumbing ==============================================

    def _set_gripper(self, state: int) -> None:
        self.sim.callScriptFunction(
            '_ext_set_target', self.gripper_script_h, int(state))

    def _gripper_reached(self, target_state: int) -> bool:
        if self._clock_s() - self.action_t >= self.gripper_timeout:
            self.get_logger().warn(
                f'gripper timeout waiting for {target_state}')
            return True
        cur = self.sim.callScriptFunction(
            '_ext_get_state', self.gripper_script_h)
        return cur is not None and int(cur) == target_state

    def _attach_cube(self) -> None:
        self.sim.setObjectParent(self.cube_h, self.attach_h, True)
        self.sim.resetDynamicObject(self.cube_h)

    def _detach_cube(self) -> None:
        self.sim.setObjectParent(self.cube_h, -1, True)
        self.sim.resetDynamicObject(self.cube_h)

    def _pose(self) -> tuple[float, float, float] | None:
        mat = self.sim.getObjectMatrix(self.robot_h, -1)
        return float(mat[3]), float(mat[7]), yaw_from_pose_matrix(mat)

    def _clock_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ===== planning ==================================================

    def _needs_replan(self, now: float, rx: float, ry: float) -> bool:
        return (self.planner is None
                or now - self.last_replan_t >= self.replan_period
                or self.planner.is_finished((rx, ry)))

    def _replan(self, rx: float, ry: float) -> None:
        infl_cells = max(1, int(math.ceil(
            self.robot_radius / self.grid.spec.resolution)))
        infl = self.grid.inflate(infl_cells)

        if self.mode == 'go_to_key':
            self._replan_to(self.grid, infl, rx, ry, self.targets['cube'])
        elif self.mode == 'go_to_plate':
            self._replan_to(self.grid, infl, rx, ry, self.targets['plate'])
        elif self.mode == 'go_to_door':
            self._replan_to(self.grid, infl, rx, ry, self._door_exit_xy())
        else:
            self._replan_frontier(self.grid, infl, rx, ry)

    def _replan_to(self, grid: OccupancyGrid, infl: OccupancyGrid,
                   rx: float, ry: float,
                   target: tuple[float, float]) -> None:
        """A* directly to ``target``. If the corridor still crosses
        UNKNOWN cells (treated as blocked), fall back to the closest
        reachable frontier toward the target."""
        # 1) try the exact target plus a small ring of free cells around it.
        for goal in [target] + self._free_ring(infl, target):
            path = plan_path(infl, (rx, ry), goal)
            if path is not None and len(path) >= 2:
                self._adopt_path(path)
                return

        # 2) pick the closest reachable frontier to the target.
        frontiers = find_frontiers(grid, min_size=self.frontier_min_size)
        self._publish_frontiers(frontiers)
        ranked = sorted(
            ((math.hypot(f.centroid_xy[0] - target[0],
                         f.centroid_xy[1] - target[1]), f)
             for f in frontiers if self._eligible(infl, f)),
            key=lambda x: x[0])
        for _, f in ranked:
            path = plan_path(infl, (rx, ry), f.centroid_xy)
            if path is None or len(path) < 2:
                self.blacklist.add(infl.world_to_grid(*f.centroid_xy))
                continue
            self._adopt_path(path)
            return

        self.planner = None

    def _replan_frontier(self, grid: OccupancyGrid, infl: OccupancyGrid,
                         rx: float, ry: float) -> None:
        """Frontier-driven exploration: pick the highest size/distance
        frontier the robot can reach."""
        frontiers = find_frontiers(grid, min_size=self.frontier_min_size)
        self._publish_frontiers(frontiers)
        if not frontiers:
            self._enter('done')
            self.planner = None
            return

        ranked: list[tuple[float, object]] = []
        for f in frontiers:
            if not self._eligible(infl, f):
                continue
            d = math.hypot(f.centroid_xy[0] - rx, f.centroid_xy[1] - ry)
            if d < 1e-3:
                continue
            ranked.append((f.size / d, f))
        ranked.sort(reverse=True)

        for _, f in ranked:
            path = plan_path(infl, (rx, ry), f.centroid_xy)
            if path is None or len(path) < 2:
                self.blacklist.add(infl.world_to_grid(*f.centroid_xy))
                continue
            self._adopt_path(path)
            return

        self.planner = None

    def _eligible(self, infl: OccupancyGrid, f) -> bool:
        cc, cr = infl.world_to_grid(*f.centroid_xy)
        return ((cc, cr) not in self.blacklist
                and infl.is_traversable(cc, cr))

    def _adopt_path(self, path: list[tuple[float, float]]) -> None:
        self.planner = PurePursuit(path, PurePursuitConfig())
        self._publish_path(path)

    def _free_ring(self, grid: OccupancyGrid,
                   xy: tuple[float, float]) -> list[tuple[float, float]]:
        """Free cells in a small ring around ``xy``, sorted by
        distance. Used as a fall-back goal when the exact target cell
        is in the inflation buffer."""
        cc, cr = grid.world_to_grid(*xy)
        res = grid.spec.resolution
        ox, oy = grid.spec.origin_x, grid.spec.origin_y
        out: list[tuple[float, tuple[float, float]]] = []
        for dr in range(-4, 5):
            for dc in range(-4, 5):
                if dc == 0 and dr == 0:
                    continue
                nc, nr = cc + dc, cr + dr
                if not grid.in_bounds(nc, nr):
                    continue
                if not grid.is_traversable(nc, nr):
                    continue
                wx = ox + (nc + 0.5) * res
                wy = oy + (nr + 0.5) * res
                out.append((math.hypot(wx - xy[0], wy - xy[1]), (wx, wy)))
        out.sort()
        return [w for _, w in out]

    # ===== geometric helpers =========================================

    @staticmethod
    def _dist(rx: float, ry: float, xy: tuple[float, float]) -> float:
        return math.hypot(xy[0] - rx, xy[1] - ry)

    def _door_exit_xy(self) -> tuple[float, float]:
        """Door target pushed outward from the room origin so the robot
        drives through the open gap rather than stopping at the wall."""
        dx, dy = self.targets['door']
        n = math.hypot(dx, dy)
        if n < 1e-3:
            return dx, dy
        return dx + self.door_push * dx / n, dy + self.door_push * dy / n

    # ===== state transitions / publishing ============================

    def _enter(self, mode: str) -> None:
        self.get_logger().info(f'mode: {self.mode} → {mode}')
        self.mode = mode

    def _stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _publish_path(self, path: list[tuple[float, float]]) -> None:
        msg = PathMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        for x, y in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def _publish_frontiers(self, frontiers) -> None:
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        for f in frontiers:
            p = Pose()
            p.position.x = float(f.centroid_xy[0])
            p.position.y = float(f.centroid_xy[1])
            p.orientation.w = 1.0
            msg.poses.append(p)
        self.frontiers_pub.publish(msg)


def main(args=None):
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


if __name__ == '__main__':
    main()
