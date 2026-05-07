#!/usr/bin/env python3
"""
ROS2 node that paints a 2D occupancy grid by querying CoppeliaSim
directly for the obstacle layout, instead of ray-casting a published
LiDAR cloud.

Why not the LiDAR
-----------------
In simulation we know the position of every wall and obstacle exactly:
they were placed by ``build_scene.py``. The Velodyne cloud, on the
other hand, doesn't synchronise cleanly with sim physics steps; if we
ray-cast it as the robot moves, walls smear because the same cell gets
re-marked at slightly different positions across scans. Reading the
scene state directly produces a deterministic, perfect map: every
obstacle is painted exactly once, in the right cells.

Discovery semantics are preserved
---------------------------------
The map is not pre-filled. Each tick, only cells within ``max_range``
of the robot's *current* position are updated. A virtual scan of N
rays from the robot through the obstacle mask ensures cells behind a
wall stay UNKNOWN until the robot moves to a position where the wall
no longer occludes them. So frontier exploration still does its job:
the robot has to physically move to discover.

Subscribes:
    (none — pose and obstacle layout both come from CoppeliaSim ZMQ.)

Publishes:
    /map        (nav_msgs/OccupancyGrid, in ``world`` frame, ~2 Hz)
    /tf_static  (one-shot ``world -> odom`` transform)
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from tf2_ros import StaticTransformBroadcaster

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from escape_room.mapping.occupancy_grid import GridSpec, OccupancyGrid


# Object-alias prefixes treated as obstacles when enumerating the scene.
# These match the names build_scene.py assigns to its primitives.
_OBSTACLE_PREFIXES = ('Wall', 'Obstacle_', 'Door', 'PressurePlate')


class MapperNode(Node):
    def __init__(self) -> None:
        super().__init__('mapper_node')

        # ---- parameters ---------------------------------------------
        self.declare_parameter('grid_width_m', 10.0)
        self.declare_parameter('grid_height_m', 10.0)
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('origin_x', -5.0)
        self.declare_parameter('origin_y', -5.0)
        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('process_rate_hz', 5.0)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'world')
        self.declare_parameter('robot_alias', '/RoboMasterEP/BaseLinkFrame')
        # 720 rays = every 0.5°. Higher = finer occlusion detection.
        self.declare_parameter('scan_rays', 720)

        self.map_frame = str(self.get_parameter('map_frame').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.scan_rays = int(self.get_parameter('scan_rays').value)
        robot_alias = str(self.get_parameter('robot_alias').value)

        spec = GridSpec(
            width_m=float(self.get_parameter('grid_width_m').value),
            height_m=float(self.get_parameter('grid_height_m').value),
            resolution=float(self.get_parameter('resolution').value),
            origin_x=float(self.get_parameter('origin_x').value),
            origin_y=float(self.get_parameter('origin_y').value),
        )
        self.grid = OccupancyGrid(spec)

        # ---- sim connection -----------------------------------------
        self.get_logger().info('Connecting to CoppeliaSim ZMQ remote API...')
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        try:
            self.robot_handle = self.sim.getObject(robot_alias)
        except Exception as e:
            raise RuntimeError(
                f"Could not resolve robot alias '{robot_alias}'. "
                f"Run build_scene first. ({e})"
            )

        # ---- one-shot setup -----------------------------------------
        self._publish_world_to_odom_tf()
        self.obstacles = self._enumerate_obstacles()
        self.get_logger().info(
            f'Found {len(self.obstacles)} obstacle(s) in the scene.')

        self._cell_x, self._cell_y = self._grid_cell_centers(spec)
        self._obstacle_mask = self._compute_obstacle_mask()

        angles = np.linspace(0.0, 2.0 * np.pi, self.scan_rays, endpoint=False)
        self._ray_dx = np.cos(angles)
        self._ray_dy = np.sin(angles)

        # ---- ROS publishers -----------------------------------------
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(
            OccupancyGridMsg,
            str(self.get_parameter('map_topic').value),
            latched_qos,
        )

        # ---- timers --------------------------------------------------
        self.create_timer(
            1.0 / float(self.get_parameter('process_rate_hz').value),
            self._tick,
        )
        self.create_timer(
            1.0 / float(self.get_parameter('publish_rate_hz').value),
            self._publish_map,
        )

        self._has_data = False
        self.get_logger().info(
            f'mapper_node ready. '
            f'grid={spec.width_m}x{spec.height_m} m '
            f'@ {spec.resolution} m/cell ({self.grid.cols}x{self.grid.rows}); '
            f'max_range={self.max_range:.2f} m; '
            f'pose source = sim direct ({robot_alias})'
        )

    # ===== one-shot setup =================================================

    def _publish_world_to_odom_tf(self) -> None:
        """Bridge the sim's ``world`` frame into the robomaster_ros TF tree.

        robomaster_ros publishes /odom and tf as ``odom -> ... ->
        chassis_base_link``, with odom initialised at the robot's pose
        when the driver started. We're starting now and the robot has
        not moved yet, so the world→odom transform is just the robot's
        current world pose (xy + yaw).
        """
        try:
            mat = self.sim.getObjectMatrix(self.robot_handle, -1)
        except Exception as e:
            self.get_logger().warn(
                f'Could not read robot pose for world->odom TF: {e}')
            return

        self._static_tf = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'odom'
        t.transform.translation.x = float(mat[3])
        t.transform.translation.y = float(mat[7])
        t.transform.translation.z = float(mat[11])
        yaw = math.atan2(mat[4], mat[0])
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self._static_tf.sendTransform(t)
        self.get_logger().info(
            f'Published static TF world -> odom @ '
            f'({mat[3]:.2f}, {mat[7]:.2f}, yaw={math.degrees(yaw):.1f}°)'
        )

    def _enumerate_obstacles(self) -> list[dict]:
        """List Wall*/Obstacle_*/Door/PressurePlate objects with their
        world-axis-aligned XY bounding box."""
        out: list[dict] = []
        for handle in self.sim.getObjectsInTree(self.sim.handle_scene):
            try:
                alias = self.sim.getObjectAlias(handle, 0)
            except Exception:
                continue
            if not alias.startswith(_OBSTACLE_PREFIXES):
                continue
            try:
                aabb = self._world_xy_aabb(handle)
            except Exception as e:
                self.get_logger().warn(
                    f"Could not compute AABB for '{alias}': {e}")
                continue
            out.append({'alias': alias, 'aabb': aabb})
        return out

    def _world_xy_aabb(self, handle: int
                       ) -> tuple[float, float, float, float]:
        """World-frame XY axis-aligned bounding box for an object,
        accounting for its yaw. Exact for axis-aligned cuboids and
        cylinders — the only primitives build_scene.py creates."""
        sim = self.sim
        pos = sim.getObjectPosition(handle, -1)
        yaw = sim.getObjectOrientation(handle, -1)[2]
        x0 = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_x)
        y0 = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_y)
        x1 = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_x)
        y1 = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_y)
        c, s = math.cos(yaw), math.sin(yaw)
        local_corners = [(x0, y0), (x0, y1), (x1, y0), (x1, y1)]
        wx = [pos[0] + c * x - s * y for x, y in local_corners]
        wy = [pos[1] + s * x + c * y for x, y in local_corners]
        return min(wx), min(wy), max(wx), max(wy)

    def _grid_cell_centers(self, spec: GridSpec
                           ) -> tuple[np.ndarray, np.ndarray]:
        """World XY of every cell center, returned as (rows, cols) arrays."""
        cols = np.arange(self.grid.cols)
        rows = np.arange(self.grid.rows)
        cx = spec.origin_x + (cols + 0.5) * spec.resolution
        cy = spec.origin_y + (rows + 0.5) * spec.resolution
        cell_x, cell_y = np.meshgrid(cx, cy)
        return cell_x.astype(np.float32), cell_y.astype(np.float32)

    def _compute_obstacle_mask(self) -> np.ndarray:
        """Boolean (rows, cols) mask: True where a cell center lies
        inside any obstacle's world AABB."""
        mask = np.zeros((self.grid.rows, self.grid.cols), dtype=bool)
        for obs in self.obstacles:
            xmin, ymin, xmax, ymax = obs['aabb']
            inside = ((self._cell_x >= xmin) & (self._cell_x <= xmax) &
                      (self._cell_y >= ymin) & (self._cell_y <= ymax))
            mask |= inside
        return mask

    # ===== per-tick virtual scan ==========================================

    def _tick(self) -> None:
        try:
            mat = self.sim.getObjectMatrix(self.robot_handle, -1)
        except Exception as e:
            self.get_logger().warn(
                f'sim.getObjectMatrix failed: {e}',
                throttle_duration_sec=2.0,
            )
            return
        rx, ry = float(mat[3]), float(mat[7])
        self._virtual_scan(rx, ry)
        self._has_data = True

    def _virtual_scan(self, rx: float, ry: float) -> None:
        """Cast ``scan_rays`` rays from the robot through the obstacle
        mask. Cells along each ray become FREE up to the first
        obstacle (which becomes OCC). Cells beyond ``max_range`` or
        behind an obstacle stay UNKNOWN — that's how movement-driven
        discovery is preserved."""
        spec = self.grid.spec
        res = spec.resolution
        max_steps = int(math.ceil(self.max_range / res))
        n_rays = self._ray_dx.shape[0]

        # Pre-compute (col, row) along every ray, every step.
        steps = (np.arange(1, max_steps + 1) * res).astype(np.float32)
        wx = rx + np.outer(self._ray_dx, steps)
        wy = ry + np.outer(self._ray_dy, steps)
        cols = ((wx - spec.origin_x) / res).astype(np.int32)
        rows = ((wy - spec.origin_y) / res).astype(np.int32)
        in_bounds = ((cols >= 0) & (cols < self.grid.cols) &
                     (rows >= 0) & (rows < self.grid.rows))

        free = self.grid.free
        occ = self.grid.occ
        obstacle = self._obstacle_mask

        # The early-exit on each ray is what makes occlusion behave
        # correctly; vectorising this requires a bit of trickery and
        # the loop is fast enough at 5 Hz on this 100×100 grid.
        for r in range(n_rays):
            for s in range(max_steps):
                if not in_bounds[r, s]:
                    break
                c = int(cols[r, s])
                rr = int(rows[r, s])
                if obstacle[rr, c]:
                    occ[rr, c] = True
                    break
                free[rr, c] = True

        # The robot's own cell is observed-free.
        rcol, rrow = self.grid.world_to_grid(rx, ry)
        if self.grid.in_bounds(rcol, rrow):
            free[rrow, rcol] = True

    # ===== publishing =====================================================

    def _publish_map(self) -> None:
        if not self._has_data:
            return
        spec = self.grid.spec
        msg = OccupancyGridMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = spec.resolution
        msg.info.width = self.grid.cols
        msg.info.height = self.grid.rows
        msg.info.origin.position.x = spec.origin_x
        msg.info.origin.position.y = spec.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid.data.flatten().astype(np.int8).tolist()
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MapperNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
