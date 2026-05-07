#!/usr/bin/env python3
"""Frontier-based exploration node.

Subscribes:
    /map                    (nav_msgs/OccupancyGrid)
    /targets/{cube,plate,door}  (geometry_msgs/PoseStamped, latched)

Publishes:
    /cmd_vel               (geometry_msgs/Twist)
    /exploration/path      (nav_msgs/Path) — A* plan, for RViz
    /exploration/frontiers (geometry_msgs/PoseArray) — frontier centroids

Pose source is the CoppeliaSim ZMQ API (bypasses TF, exact pose every
tick). The node has two modes:

* ``explore`` (default) — pick frontiers, plan A* toward them.
* ``go_to_key`` — entered as soon as all three landmark poses have
  arrived on ``/targets/*``. Tries A* directly to the cube; if the
  corridor still crosses UNKNOWN cells it heads to the frontier
  closest to the cube and retries on the next replan.
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


def yaw_from_pose_matrix(mat12: list) -> float:
    """Yaw from a CoppeliaSim 3x4 row-major pose matrix."""
    R = np.array(mat12, dtype=np.float64).reshape(3, 4)[:, :3]
    return math.atan2(R[1, 0], R[0, 0])


def occupancy_msg_to_grid(msg: OccupancyGridMsg) -> OccupancyGrid:
    """Rebuild an OccupancyGrid from a nav_msgs/OccupancyGrid: re-encode
    {-1, 0, 100} into boolean ``free`` / ``occ`` masks."""
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


class ExplorerNode(Node):
    def __init__(self) -> None:
        super().__init__('explorer_node')

        # ---- parameters ---------------------------------------------
        self.declare_parameter('map_frame', 'world')
        self.declare_parameter('robot_alias', '/RoboMasterEP/BaseLinkFrame')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_rate_hz', 4.0)
        self.declare_parameter('replan_period_s', 1.0)
        self.declare_parameter('robot_radius_m', 0.20)
        self.declare_parameter('frontier_min_size', 4)
        self.declare_parameter('key_arrival_tol_m', 0.30)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.replan_period = float(
            self.get_parameter('replan_period_s').value)
        self.robot_radius = float(
            self.get_parameter('robot_radius_m').value)
        self.frontier_min_size = int(
            self.get_parameter('frontier_min_size').value)
        self._key_arrival_tol = float(
            self.get_parameter('key_arrival_tol_m').value)
        robot_alias = str(self.get_parameter('robot_alias').value)

        # ---- sim connection -----------------------------------------
        # Keep the client as a member: anonymous clients can be
        # garbage-collected, dropping the ZMQ connection.
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        try:
            self.robot_handle = self.sim.getObject(robot_alias)
        except Exception as e:
            raise RuntimeError(
                f"could not resolve robot alias '{robot_alias}'; "
                f"run build_scene first ({e})")

        # ---- ROS pub/sub --------------------------------------------
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGridMsg,
            str(self.get_parameter('map_topic').value),
            self._on_map,
            latched_qos,
        )
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self.path_pub = self.create_publisher(
            PathMsg, '/exploration/path', 10)
        self.frontiers_pub = self.create_publisher(
            PoseArray, '/exploration/frontiers', 10)

        # Landmark topics from color_detector_node; we need all three
        # before abandoning exploration.
        self._target_xy: dict[str, tuple[float, float]] = {}
        self._required_targets = ('cube', 'plate', 'door')
        self._target_subs = [
            self.create_subscription(
                PoseStamped, f'/targets/{name}',
                lambda msg, n=name: self._on_target(n, msg),
                latched_qos,
            )
            for name in self._required_targets
        ]

        # ---- state ---------------------------------------------------
        self._latest_map: OccupancyGrid | None = None
        self._planner: PurePursuit | None = None
        self._goal_xy: tuple[float, float] | None = None
        self._last_replan_time: float = 0.0
        self._blacklist: set[tuple[int, int]] = set()
        self._mode: str = 'explore'  # 'explore' | 'go_to_key' | 'done'

        # ---- control loop -------------------------------------------
        self.create_timer(
            1.0 / float(self.get_parameter('control_rate_hz').value),
            self._tick,
        )

        self.get_logger().info(
            f'ready. robot_radius={self.robot_radius:.2f} m, '
            f'replan every {self.replan_period:.1f} s')

    # ===== callbacks =====================================================

    def _on_map(self, msg: OccupancyGridMsg) -> None:
        self._latest_map = occupancy_msg_to_grid(msg)

    def _on_target(self, name: str, msg: PoseStamped) -> None:
        if name in self._target_xy:
            return
        self._target_xy[name] = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
        )
        self.get_logger().info(
            f"target '{name}' locked at "
            f"({self._target_xy[name][0]:.2f}, {self._target_xy[name][1]:.2f}) "
            f"[{len(self._target_xy)}/{len(self._required_targets)}]"
        )
        if (self._mode == 'explore'
                and all(t in self._target_xy
                        for t in self._required_targets)):
            self._mode = 'go_to_key'
            self._planner = None  # force a replan toward the key
            self._goal_xy = None
            self.get_logger().info(
                f'all landmarks seen — GO_TO_KEY '
                f'(cube @ {self._target_xy["cube"]})')

    def _tick(self) -> None:
        if self._mode == 'done':
            self._publish_stop()
            return
        if self._latest_map is None:
            return

        pose = self._lookup_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        if self._mode == 'go_to_key':
            kx, ky = self._target_xy['cube']
            if math.hypot(kx - rx, ky - ry) <= self._key_arrival_tol:
                self.get_logger().info('arrived at the key — stopping')
                self._mode = 'done'
                self._planner = None
                self._publish_stop()
                return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._needs_replan(rx, ry, now):
            self._replan(rx, ry)
            self._last_replan_time = now

        if self._planner is None:
            self._publish_stop()
            return

        v, w = self._planner.step(rx, ry, ryaw)
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)

    # ===== planning ======================================================

    def _needs_replan(self, rx: float, ry: float, now: float) -> bool:
        if self._planner is None:
            return True
        if (now - self._last_replan_time) >= self.replan_period:
            return True
        if self._planner.is_finished((rx, ry)):
            return True
        return False

    def _replan(self, rx: float, ry: float) -> None:
        grid = self._latest_map
        if grid is None:
            return
        # Inflate by robot radius so A* can plan with a point robot.
        inflate_cells = max(
            1, int(math.ceil(self.robot_radius / grid.spec.resolution)))
        inflated = grid.inflate(inflate_cells)

        if self._mode == 'go_to_key':
            self._replan_to_key(grid, inflated, rx, ry)
        else:
            self._replan_frontier(grid, inflated, rx, ry)

    def _replan_to_key(self, grid: OccupancyGrid, inflated: OccupancyGrid,
                       rx: float, ry: float) -> None:
        """A* directly to the cube. Falls back to the frontier nearest
        the key when the corridor still crosses UNKNOWN (which A*
        treats as blocked); next replan retries the direct path."""
        key_xy = self._target_xy['cube']

        # 1) Direct: try the key pose, then a free ring around it.
        for goal_xy in [key_xy] + self._nearby_free_world(inflated, key_xy):
            path = plan_path(inflated, (rx, ry), goal_xy)
            if path is not None and len(path) >= 2:
                self._adopt_path(path, goal_xy)
                self.get_logger().info(
                    f'GO_TO_KEY: direct path to ({goal_xy[0]:.2f}, '
                    f'{goal_xy[1]:.2f})')
                return

        # 2) Fall back to the closest frontier to the key.
        frontiers = find_frontiers(grid, min_size=self.frontier_min_size)
        self._publish_frontiers(frontiers)
        scored = sorted(
            ((math.hypot(f.centroid_xy[0] - key_xy[0],
                         f.centroid_xy[1] - key_xy[1]), f)
             for f in frontiers
             if self._is_frontier_eligible(inflated, f)),
            key=lambda x: x[0])

        for d_to_key, f in scored:
            path = plan_path(inflated, (rx, ry), f.centroid_xy)
            if path is None or len(path) < 2:
                self._blacklist.add(inflated.world_to_grid(*f.centroid_xy))
                continue
            self._adopt_path(path, f.centroid_xy)
            self.get_logger().info(
                f'GO_TO_KEY: heading to frontier @ '
                f'({f.centroid_xy[0]:.2f}, {f.centroid_xy[1]:.2f}) '
                f'(d_to_key={d_to_key:.2f} m)')
            return

        self.get_logger().warn(
            'GO_TO_KEY: no path to the key and no reachable frontier')
        self._planner = None

    def _replan_frontier(self, grid: OccupancyGrid,
                         inflated: OccupancyGrid,
                         rx: float, ry: float) -> None:
        frontiers = find_frontiers(grid, min_size=self.frontier_min_size)
        self._publish_frontiers(frontiers)
        if not frontiers:
            self.get_logger().info('exploration complete')
            self._mode = 'done'
            self._planner = None
            return

        # Score = size / distance: prefer big nearby frontiers.
        scored: list[tuple[float, object]] = []
        for f in frontiers:
            if not self._is_frontier_eligible(inflated, f):
                continue
            d = math.hypot(f.centroid_xy[0] - rx, f.centroid_xy[1] - ry)
            if d < 1e-3:
                continue
            scored.append((f.size / d, f))
        scored.sort(reverse=True)

        for score, f in scored:
            path = plan_path(inflated, (rx, ry), f.centroid_xy)
            if path is None or len(path) < 2:
                self._blacklist.add(inflated.world_to_grid(*f.centroid_xy))
                continue
            self._adopt_path(path, f.centroid_xy)
            self.get_logger().info(
                f'heading to frontier @ ({f.centroid_xy[0]:.2f}, '
                f'{f.centroid_xy[1]:.2f}) [size={f.size}, score={score:.2f}]')
            return

        self.get_logger().warn('no reachable frontier this tick')
        self._planner = None

    def _is_frontier_eligible(self, inflated: OccupancyGrid, f) -> bool:
        cc, cr = inflated.world_to_grid(*f.centroid_xy)
        return ((cc, cr) not in self._blacklist
                and inflated.is_traversable(cc, cr))

    def _adopt_path(self, path: list[tuple[float, float]],
                    goal_xy: tuple[float, float]) -> None:
        self._planner = PurePursuit(path, PurePursuitConfig())
        self._goal_xy = goal_xy
        self._publish_path(path)

    def _nearby_free_world(self, grid: OccupancyGrid,
                           xy: tuple[float, float]
                           ) -> list[tuple[float, float]]:
        """Free-cell world coords in a small ring around ``xy`` in the
        inflated grid, ordered by distance."""
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

    # ===== helpers =======================================================

    def _lookup_pose(self) -> tuple[float, float, float] | None:
        try:
            mat = self.sim.getObjectMatrix(self.robot_handle, -1)
        except Exception:
            return None
        return float(mat[3]), float(mat[7]), yaw_from_pose_matrix(mat)

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())  # all zeros

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
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())  # final stop
        rclpy.shutdown()


if __name__ == '__main__':
    main()
