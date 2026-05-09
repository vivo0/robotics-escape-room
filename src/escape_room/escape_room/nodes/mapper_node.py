#!/usr/bin/env python3
"""2D occupancy-grid mapper backed by direct CoppeliaSim queries.

In simulation we know every wall and obstacle pose exactly (they
were placed by ``build_scene.py``), so we don't need to ray-cast a
real LiDAR cloud — those scans don't synchronise cleanly with the
physics step and walls smear across cells.

Discovery semantics are still preserved by a virtual scan: each
tick, only cells within ``max_range`` of the robot's current pose
are updated by ``scan_rays`` rays. Cells behind a wall stay UNKNOWN
until the robot moves to a position that exposes them, so frontier
exploration still does its job.

Subscribes:
    (none — pose and obstacle layout both come from CoppeliaSim ZMQ.)

Publishes:
    /map        nav_msgs/OccupancyGrid (in ``world`` frame, ~2 Hz)
    /tf_static  one-shot ``world -> odom`` transform
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
# Match the names build_scene.py assigns to its primitives.
_OBSTACLE_PREFIXES = ('Wall', 'Obstacle_', 'Door', 'PressurePlate')

# Z-threshold below which an obstacle is considered "removed". The
# escape-room door slides under the floor when opened (z ≈ -0.30 m);
# excluding it from the mask + clearing OCC on ray pass-through is
# what lets A* plan through after the door opens.
_BELOW_FLOOR_Z = -0.05


class MapperNode(Node):

    def __init__(self) -> None:
        super().__init__('mapper_node')

        # ----- parameters --------------------------------------------
        p = self.declare_parameter
        p('grid_width_m',     10.0)
        p('grid_height_m',    10.0)
        p('resolution',       0.10)
        p('origin_x',         -5.0)
        p('origin_y',         -5.0)
        p('max_range',        6.0)
        p('publish_rate_hz',  2.0)
        p('process_rate_hz',  5.0)
        p('map_topic',        '/map')
        p('map_frame',        'world')
        p('robot_alias',      '/RoboMasterEP/BaseLinkFrame')
        p('scan_rays',        720)   # 720 = every 0.5°

        g = lambda n: self.get_parameter(n).value
        self.map_frame = g('map_frame')
        self.max_range = float(g('max_range'))
        self.scan_rays = int(g('scan_rays'))

        spec = GridSpec(
            width_m=float(g('grid_width_m')),
            height_m=float(g('grid_height_m')),
            resolution=float(g('resolution')),
            origin_x=float(g('origin_x')),
            origin_y=float(g('origin_y')),
        )
        self.grid = OccupancyGrid(spec)

        # ----- sim connection ----------------------------------------
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        self.robot_handle = self.sim.getObject(g('robot_alias'))

        # ----- one-shot setup ----------------------------------------
        self._publish_world_to_odom_tf()

        # Cache obstacle handles + local bbox extents at init; recompute
        # world AABBs every tick so the door (which slides underground
        # when opened) drops out of the mask.
        self.obstacle_specs = self._enumerate_obstacles()
        self.get_logger().info(
            f'found {len(self.obstacle_specs)} obstacle(s) in the scene')
        self.obstacles = self._refresh_obstacle_aabbs()

        self._cell_x, self._cell_y = self._grid_cell_centres(spec)
        self._obstacle_mask = self._compute_obstacle_mask()

        angles = np.linspace(0.0, 2.0 * np.pi, self.scan_rays, endpoint=False)
        self._ray_dx = np.cos(angles)
        self._ray_dy = np.sin(angles)

        # ----- ROS publisher -----------------------------------------
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(
            OccupancyGridMsg, g('map_topic'), latched)

        self.create_timer(1.0 / float(g('process_rate_hz')), self._tick)
        self.create_timer(1.0 / float(g('publish_rate_hz')), self._publish_map)

        self._has_data = False
        self.get_logger().info(
            f'ready; grid={spec.width_m}×{spec.height_m} m '
            f'@ {spec.resolution} m/cell, max_range={self.max_range:.2f} m')

    # ===== one-shot setup ============================================

    def _publish_world_to_odom_tf(self) -> None:
        """Bridge the sim ``world`` frame into the robomaster_ros TF
        tree. The driver initialises ``odom`` at the robot's pose at
        startup, so world→odom is just that pose (xy + yaw)."""
        mat = self.sim.getObjectMatrix(self.robot_handle, -1)

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
            f'world→odom @ ({mat[3]:.2f}, {mat[7]:.2f}, '
            f'yaw={math.degrees(yaw):.1f}°)')

    def _enumerate_obstacles(self) -> list[dict]:
        """Once at init: list obstacle handles and their local bbox
        extents (which never change)."""
        sim = self.sim
        out: list[dict] = []
        for handle in sim.getObjectsInTree(sim.handle_scene):
            alias = sim.getObjectAlias(handle, 0)
            if not alias.startswith(_OBSTACLE_PREFIXES):
                continue
            extents = (
                sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_x),
                sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_y),
                sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_x),
                sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_y),
            )
            out.append({'handle': int(handle), 'alias': alias,
                        'extents': extents})
        return out

    def _refresh_obstacle_aabbs(self) -> list[dict]:
        """Per tick: rebuild each obstacle's world XY AABB from current
        sim pose. Objects below the floor (e.g. the open door) are
        skipped so they no longer block A*."""
        out: list[dict] = []
        for spec in self.obstacle_specs:
            pos = self.sim.getObjectPosition(spec['handle'], -1)
            if pos[2] < _BELOW_FLOOR_Z:
                continue
            yaw = self.sim.getObjectOrientation(spec['handle'], -1)[2]
            x0, y0, x1, y1 = spec['extents']
            c, s = math.cos(yaw), math.sin(yaw)
            corners = ((x0, y0), (x0, y1), (x1, y0), (x1, y1))
            wx = [pos[0] + c * x - s * y for x, y in corners]
            wy = [pos[1] + s * x + c * y for x, y in corners]
            out.append({'alias': spec['alias'],
                        'aabb': (min(wx), min(wy), max(wx), max(wy))})
        return out

    def _grid_cell_centres(self, spec: GridSpec
                           ) -> tuple[np.ndarray, np.ndarray]:
        cols = np.arange(self.grid.cols)
        rows = np.arange(self.grid.rows)
        cx = spec.origin_x + (cols + 0.5) * spec.resolution
        cy = spec.origin_y + (rows + 0.5) * spec.resolution
        cell_x, cell_y = np.meshgrid(cx, cy)
        return cell_x.astype(np.float32), cell_y.astype(np.float32)

    def _compute_obstacle_mask(self) -> np.ndarray:
        """True where the cell centre lies inside any obstacle AABB."""
        mask = np.zeros((self.grid.rows, self.grid.cols), dtype=bool)
        for obs in self.obstacles:
            xmin, ymin, xmax, ymax = obs['aabb']
            mask |= ((self._cell_x >= xmin) & (self._cell_x <= xmax) &
                     (self._cell_y >= ymin) & (self._cell_y <= ymax))
        return mask

    # ===== per-tick virtual scan =====================================

    def _tick(self) -> None:
        mat = self.sim.getObjectMatrix(self.robot_handle, -1)
        rx, ry = float(mat[3]), float(mat[7])
        # Re-poll obstacle poses every tick (door slides underground
        # when opened) and rebuild the mask from the survivors.
        self.obstacles = self._refresh_obstacle_aabbs()
        self._obstacle_mask = self._compute_obstacle_mask()
        self._virtual_scan(rx, ry)
        self._has_data = True

    def _virtual_scan(self, rx: float, ry: float) -> None:
        """Cast ``scan_rays`` rays from the robot through the obstacle
        mask. Cells along each ray become FREE up to the first
        obstacle (which becomes OCC). When a ray passes through a cell
        that is no longer in the mask, OCC is also cleared so A* can
        plan through openings the door used to block."""
        spec = self.grid.spec
        res = spec.resolution
        max_steps = int(math.ceil(self.max_range / res))

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

        # Per-ray loop: vectorising the early-exit on hit isn't
        # worth the complexity at 5 Hz on a 100×100 grid.
        for r in range(self._ray_dx.shape[0]):
            for s in range(max_steps):
                if not in_bounds[r, s]:
                    break
                c = int(cols[r, s])
                rr = int(rows[r, s])
                if obstacle[rr, c]:
                    occ[rr, c] = True
                    break
                free[rr, c] = True
                occ[rr, c] = False

        # The robot's own cell is always observed-free.
        rcol, rrow = self.grid.world_to_grid(rx, ry)
        if self.grid.in_bounds(rcol, rrow):
            free[rrow, rcol] = True

    # ===== publishing ================================================

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
    node = MapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
