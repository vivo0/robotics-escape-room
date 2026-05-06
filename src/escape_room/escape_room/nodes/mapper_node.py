#!/usr/bin/env python3
"""
ROS2 node that builds a 2D occupancy grid by querying CoppeliaSim
directly for the obstacle layout, instead of ray-casting LiDAR points.

Why direct from sim
-------------------
The CoppeliaSim Velodyne plugin's per-scan timestamp doesn't line up
with sim physics steps cleanly, so any moving-robot mapping based on
the published cloud ends up smearing walls (the same cell gets re-hit
at slightly different positions across scans). In simulation we *know*
where every wall and obstacle is — they were placed by `build_scene.py`
— so we can paint a deterministic, perfect map.

Discovery semantics are preserved by simulating the sensor's range:
only cells within `max_range` of the robot are updated; the rest stay
UNKNOWN. A virtual ray-cast (sweeping 720 rays through the obstacle
mask) ensures cells *behind* a wall stay UNKNOWN until the robot
physically moves to a position where the wall no longer occludes them.

Subscribes:
    (none — pose comes from sim, layout comes from sim)

Publishes:
    /map  (nav_msgs/OccupancyGrid, in `world` frame, ~2 Hz)
"""
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

from escape_room.mapping import OccupancyGrid
from escape_room.mapping.occupancy_grid import GridSpec


# Aliases we consider obstacles when enumerating the scene tree.
_OBSTACLE_PREFIXES = ('Wall', 'Obstacle_', 'Door', 'PressurePlate')


class MapperNode(Node):
    def __init__(self):
        super().__init__('mapper_node')

        self.declare_parameter('grid_width_m', 10.0)
        self.declare_parameter('grid_height_m', 10.0)
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('origin_x', -5.0)
        self.declare_parameter('origin_y', -5.0)
        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('process_rate_hz', 5.0)
        self.declare_parameter('odom_frame', 'world')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_alias', '/RoboMasterEP/BaseLinkFrame')
        # Angular resolution of the virtual scan: 720 rays = every 0.5°.
        self.declare_parameter('scan_rays', 720)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.max_range = float(self.get_parameter('max_range').value)
        robot_alias = self.get_parameter('robot_alias').value
        self.scan_rays = int(self.get_parameter('scan_rays').value)

        spec = GridSpec(
            width_m=float(self.get_parameter('grid_width_m').value),
            height_m=float(self.get_parameter('grid_height_m').value),
            resolution=float(self.get_parameter('resolution').value),
            origin_x=float(self.get_parameter('origin_x').value),
            origin_y=float(self.get_parameter('origin_y').value),
        )
        self.grid = OccupancyGrid(spec)

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

        # Publish a static TF connecting `world` (sim's global frame) to
        # the robomaster_ros TF tree's `odom`. odom origin equals the
        # robot's pose at the moment the driver started, which is its
        # current world pose now (robot hasn't moved yet).
        self._publish_world_to_odom_tf()

        # Enumerate static obstacles once (their AABB doesn't change).
        self.obstacles = self._enumerate_obstacles()
        self.get_logger().info(
            f'Found {len(self.obstacles)} obstacles in scene.'
        )

        # Pre-compute the world-frame XY of every cell center, and the
        # boolean obstacle mask. Static; never recomputed.
        self._cell_x, self._cell_y = self._grid_cell_centers(spec)
        self._obstacle_mask = self._compute_obstacle_mask()

        # Pre-compute ray directions for the virtual scan.
        angles = np.linspace(0.0, 2.0 * np.pi, self.scan_rays, endpoint=False)
        self._ray_dx = np.cos(angles)
        self._ray_dy = np.sin(angles)

        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(
            OccupancyGridMsg,
            self.get_parameter('map_topic').value,
            latched_qos,
        )

        process_rate = float(self.get_parameter('process_rate_hz').value)
        self.create_timer(1.0 / process_rate, self.tick)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / publish_rate, self.publish_map)

        self._has_data = False
        self.get_logger().info(
            f'mapper_node ready. grid={spec.width_m}x{spec.height_m} m '
            f'@ {spec.resolution} m/cell ({self.grid.cols}x{self.grid.rows}); '
            f'max_range={self.max_range:.2f} m; '
            f'pose source = sim direct ({robot_alias}); '
            f'mapping = deterministic obstacle paint'
        )

    def _publish_world_to_odom_tf(self) -> None:
        """world -> odom static TF: odom origin = robot's current world
        pose (robomaster_ros initialised odom there at driver startup).
        """
        try:
            mat = self.sim.getObjectMatrix(self.robot_handle, -1)
        except Exception as e:
            self.get_logger().warn(
                f'Could not read robot pose for world->odom TF: {e}'
            )
            return

        self._static_tf_broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'odom'
        t.transform.translation.x = float(mat[3])
        t.transform.translation.y = float(mat[7])
        t.transform.translation.z = float(mat[11])
        # Yaw from rotation matrix (Z-up)
        yaw = math.atan2(mat[4], mat[0])
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self._static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f'Published static TF world -> odom @ '
            f'({mat[3]:.2f}, {mat[7]:.2f}, yaw={math.degrees(yaw):.1f}°)'
        )

    # ---------- one-shot setup ---------------------------------------

    def _enumerate_obstacles(self) -> list[dict]:
        """List every Wall/Obstacle/Door/Plate in the scene with its
        world-axis-aligned XY bounding box, computed once at startup.

        We rotate the local bbox corners by the object's yaw and take
        the AABB of the rotated corners — exact for axis-aligned
        boxes (the only primitives `build_scene.py` creates).
        """
        out = []
        all_handles = self.sim.getObjectsInTree(self.sim.handle_scene)
        for h in all_handles:
            try:
                alias = self.sim.getObjectAlias(h, 0)
            except Exception:
                continue
            if not alias.startswith(_OBSTACLE_PREFIXES):
                continue
            try:
                aabb = self._world_xy_aabb(h)
            except Exception as e:
                self.get_logger().warn(
                    f"Could not compute AABB for {alias}: {e}"
                )
                continue
            out.append({'alias': alias, 'aabb': aabb})
        return out

    def _world_xy_aabb(self, handle: int) -> tuple[float, float, float, float]:
        sim = self.sim
        pos = sim.getObjectPosition(handle, -1)
        orient = sim.getObjectOrientation(handle, -1)
        yaw = orient[2]
        x0 = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_x)
        y0 = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_y)
        x1 = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_x)
        y1 = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_y)
        c, s = math.cos(yaw), math.sin(yaw)
        corners = [(x0, y0), (x0, y1), (x1, y0), (x1, y1)]
        wx = [pos[0] + c * x - s * y for x, y in corners]
        wy = [pos[1] + s * x + c * y for x, y in corners]
        return min(wx), min(wy), max(wx), max(wy)

    def _grid_cell_centers(self, spec: GridSpec) -> tuple[np.ndarray, np.ndarray]:
        cols = np.arange(self.grid.cols)
        rows = np.arange(self.grid.rows)
        cx = spec.origin_x + (cols + 0.5) * spec.resolution
        cy = spec.origin_y + (rows + 0.5) * spec.resolution
        cell_x, cell_y = np.meshgrid(cx, cy)
        return cell_x.astype(np.float32), cell_y.astype(np.float32)

    def _compute_obstacle_mask(self) -> np.ndarray:
        mask = np.zeros((self.grid.rows, self.grid.cols), dtype=bool)
        for obs in self.obstacles:
            xmin, ymin, xmax, ymax = obs['aabb']
            inside = ((self._cell_x >= xmin) & (self._cell_x <= xmax) &
                      (self._cell_y >= ymin) & (self._cell_y <= ymax))
            mask |= inside
        return mask

    # ---------- per-tick virtual scan --------------------------------

    def tick(self) -> None:
        try:
            mat = self.sim.getObjectMatrix(self.robot_handle, -1)
        except Exception as e:
            self.get_logger().warn(f'sim.getObjectMatrix failed: {e}',
                                   throttle_duration_sec=2.0)
            return

        rx = float(mat[3])
        ry = float(mat[7])
        self._virtual_scan(rx, ry)
        self._has_data = True

    def _virtual_scan(self, rx: float, ry: float) -> None:
        """Cast `scan_rays` rays from the robot through the obstacle
        mask. Cells along each ray become FREE up to the first
        obstacle; the obstacle cell becomes OCC. Cells beyond
        max_range, or occluded by an earlier ray hit, stay UNKNOWN.
        """
        spec = self.grid.spec
        res = spec.resolution
        rcol, rrow = self.grid.world_to_grid(rx, ry)
        max_steps = int(math.ceil(self.max_range / res))

        # Pre-step distances along each ray, in metres. Half-cell start
        # so the first sample is the *next* cell, not the robot's cell.
        steps = (np.arange(1, max_steps + 1) * res).astype(np.float32)
        # (rays, steps) -> world XY along each ray
        wx = rx + np.outer(self._ray_dx, steps)
        wy = ry + np.outer(self._ray_dy, steps)

        # World -> grid indices, vectorised
        cols = ((wx - spec.origin_x) / res).astype(np.int32)
        rows = ((wy - spec.origin_y) / res).astype(np.int32)

        in_bounds = (cols >= 0) & (cols < self.grid.cols) & \
                    (rows >= 0) & (rows < self.grid.rows)

        rays = self._ray_dx.shape[0]
        for r in range(rays):
            for s in range(max_steps):
                if not in_bounds[r, s]:
                    break
                c = int(cols[r, s])
                rr = int(rows[r, s])
                if self._obstacle_mask[rr, c]:
                    # First obstacle along the ray -> mark and stop
                    if self.grid.hits[rr, c] < OccupancyGrid.HIT_THRESHOLD:
                        self.grid.hits[rr, c] = OccupancyGrid.HIT_THRESHOLD
                    break
                # Free cell along the ray
                self.grid.log_odds[rr, c] = OccupancyGrid.L_MIN

        # Robot's own cell is free.
        if self.grid.in_bounds(rcol, rrow):
            self.grid.log_odds[rrow, rcol] = OccupancyGrid.L_MIN

    # ---------- publishing -------------------------------------------

    def publish_map(self) -> None:
        if not self._has_data:
            return
        spec = self.grid.spec
        msg = OccupancyGridMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
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
