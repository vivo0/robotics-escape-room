#!/usr/bin/env python3
"""
Frontier-based exploration node.

Subscribes:
    /map   (nav_msgs/OccupancyGrid, in `world` frame)

Publishes:
    /cmd_vel               (geometry_msgs/Twist) — drive commands
    /exploration/path      (nav_msgs/Path) — current A* plan
    /exploration/frontiers (geometry_msgs/PoseArray) — frontier centroids

Pose source: CoppeliaSim ZMQ remote API, same as mapper_node. Bypasses
TF entirely so the planner uses the exact robot pose at the moment of
the tick — deterministic, no TF lag.

Loop (default 4 Hz):
    1. Query robot pose from sim.
    2. Reconstruct an OccupancyGrid from the latest /map.
    3. Detect frontiers; if none, stop and declare exploration done.
    4. Pick the highest-scoring frontier (size / distance) whose centroid
       lies on a traversable cell of the inflated grid.
    5. A* from robot to frontier; if unreachable, blacklist and try next.
    6. Pure-pursuit one step → publish cmd_vel.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg, Path as PathMsg

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from escape_room.exploration import find_frontiers
from escape_room.mapping.occupancy_grid import GridSpec, OccupancyGrid
from escape_room.planning import PurePursuit, plan_path
from escape_room.planning.pure_pursuit import PurePursuitConfig


def yaw_from_matrix(mat12: list) -> float:
    """Extract Z-axis rotation (yaw) from a CoppeliaSim 3x4 row-major
    pose matrix (12 floats). Assumes Z is the world up axis."""
    R = np.array(mat12, dtype=np.float64).reshape(3, 4)[:, :3]
    return math.atan2(R[1, 0], R[0, 0])


def occupancy_msg_to_grid(msg: OccupancyGridMsg) -> OccupancyGrid:
    """Build an OccupancyGrid from a nav_msgs/OccupancyGrid. The publisher
    already collapsed state to {-1, 0, 100}; we only need the discrete
    state for navigation, so re-encode it on the receiver side."""
    spec = GridSpec(
        width_m=msg.info.width * msg.info.resolution,
        height_m=msg.info.height * msg.info.resolution,
        resolution=msg.info.resolution,
        origin_x=msg.info.origin.position.x,
        origin_y=msg.info.origin.position.y,
    )
    grid = OccupancyGrid(spec)
    arr = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    grid.hits[arr == 100] = OccupancyGrid.HIT_THRESHOLD
    grid.log_odds[arr == 0] = OccupancyGrid.L_MIN
    # arr == -1 stays unknown (hits=0, log_odds=0)
    return grid


class ExplorerNode(Node):
    def __init__(self):
        super().__init__('explorer_node')

        self.declare_parameter('odom_frame', 'world')
        # Pose source for the robot. Prefer the explicit chassis link so the
        # pose is at ground level / Z-up (model root sometimes isn't).
        self.declare_parameter('robot_alias', '/RoboMasterEP/BaseLinkFrame')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_rate_hz', 4.0)
        self.declare_parameter('replan_period_s', 1.0)
        self.declare_parameter('robot_radius_m', 0.20)
        self.declare_parameter('frontier_min_size', 4)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.replan_period = float(self.get_parameter('replan_period_s').value)
        self.robot_radius = float(self.get_parameter('robot_radius_m').value)
        self.frontier_min_size = int(self.get_parameter('frontier_min_size').value)
        robot_alias = self.get_parameter('robot_alias').value

        self.get_logger().info('Connecting to CoppeliaSim ZMQ remote API...')
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        try:
            self.robot_handle = self.sim.getObject(robot_alias)
        except Exception as e:
            raise RuntimeError(
                f"Could not resolve robot alias '{robot_alias}' in sim. "
                f"Run build_scene first. ({e})"
            )

        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(
            OccupancyGridMsg,
            self.get_parameter('map_topic').value,
            self.on_map,
            latched_qos,
        )
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.path_pub = self.create_publisher(PathMsg, '/exploration/path', 10)
        self.frontiers_pub = self.create_publisher(
            PoseArray, '/exploration/frontiers', 10)

        self._latest_map: OccupancyGrid | None = None
        self._planner: PurePursuit | None = None
        self._current_goal_xy: tuple[float, float] | None = None
        self._last_replan_time: float = 0.0
        self._blacklist: set[tuple[int, int]] = set()  # frontier centroid grid cells
        self._exploration_done = False

        rate = float(self.get_parameter('control_rate_hz').value)
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f'explorer_node ready. frame={self.odom_frame}, '
            f'pose source = sim direct ({robot_alias}), '
            f'robot_radius={self.robot_radius:.2f} m, '
            f'replan every {self.replan_period:.1f} s'
        )

    # ---------- callbacks ----------

    def on_map(self, msg: OccupancyGridMsg) -> None:
        self._latest_map = occupancy_msg_to_grid(msg)

    def tick(self) -> None:
        if self._exploration_done:
            self._publish_stop()
            return
        if self._latest_map is None:
            return

        pose = self._lookup_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        now = self.get_clock().now().nanoseconds * 1e-9
        need_replan = (
            self._planner is None
            or (now - self._last_replan_time) >= self.replan_period
            or (self._planner is not None
                and self._planner.is_finished((rx, ry)))
        )

        if need_replan:
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

    # ---------- planning ----------

    def _replan(self, rx: float, ry: float) -> None:
        grid = self._latest_map
        if grid is None:
            return

        # Inflate by robot radius (rounded up to whole cells).
        inflate_cells = max(1, int(math.ceil(self.robot_radius / grid.spec.resolution)))
        inflated = grid.inflate(inflate_cells)

        frontiers = find_frontiers(grid, min_size=self.frontier_min_size)
        self._publish_frontiers(frontiers)

        if not frontiers:
            self.get_logger().info('No frontiers left — exploration complete.')
            self._exploration_done = True
            self._planner = None
            return

        # Score: size / distance, preferring big nearby frontiers. Skip
        # blacklisted goals and frontiers whose centroid is non-traversable
        # in the inflated grid (we'd never reach the centre, but the path's
        # last waypoint can be the centroid only if it's free post-inflation).
        scored = []
        for f in frontiers:
            cc, cr = inflated.world_to_grid(*f.centroid_xy)
            if (cc, cr) in self._blacklist:
                continue
            if not inflated.is_traversable(cc, cr):
                continue
            d = math.hypot(f.centroid_xy[0] - rx, f.centroid_xy[1] - ry)
            if d < 1e-3:
                continue
            scored.append((f.size / d, f))
        scored.sort(reverse=True)

        for _score, f in scored:
            path = plan_path(inflated, (rx, ry), f.centroid_xy)
            if path is None or len(path) < 2:
                cc, cr = inflated.world_to_grid(*f.centroid_xy)
                self._blacklist.add((cc, cr))
                continue
            self._planner = PurePursuit(path, PurePursuitConfig())
            self._current_goal_xy = f.centroid_xy
            self._publish_path(path)
            self.get_logger().info(
                f'Heading to frontier @ ({f.centroid_xy[0]:.2f},'
                f'{f.centroid_xy[1]:.2f}) [size={f.size}, d={_score:.2f}]'
            )
            return

        self.get_logger().warn('No reachable frontier this tick.')
        self._planner = None

    # ---------- helpers ----------

    def _lookup_pose(self) -> tuple[float, float, float] | None:
        try:
            mat = self.sim.getObjectMatrix(self.robot_handle, -1)
        except Exception:
            return None
        # mat is 12 floats (3x4 row-major); translation is in column 3.
        x = float(mat[3])
        y = float(mat[7])
        return (x, y, yaw_from_matrix(mat))

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())  # all zeros

    def _publish_path(self, path: list[tuple[float, float]]) -> None:
        msg = PathMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
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
        msg.header.frame_id = self.odom_frame
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
            try:
                node.cmd_pub.publish(Twist())  # final stop
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
