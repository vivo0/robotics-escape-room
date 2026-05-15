#!/usr/bin/env python3
"""Simulated 2D LiDAR node backed by CoppeliaSim ground-truth geometry.

Casts rays against known obstacle AABBs (walls, obstacles, door) to produce
sensor_msgs/LaserScan on /scan.  Also publishes the static base_link→laser
TF so slam_toolbox can localise the scan in the robot's frame.

The door AABB is excluded once it drops below the floor (z < -0.05 m), so
the robot's map opens at that point and Nav2 can plan through.

Publishes:
    /scan       sensor_msgs/LaserScan  (10 Hz, frame 'laser')
    /tf_static  base_link → laser      (static, once at startup)
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


_OBSTACLE_PREFIXES = ('Wall', 'Obstacle_', 'Door', 'PressurePlate')
_BELOW_FLOOR_Z = -0.05


def _ray_aabb_dist(ox: float, oy: float, dx: float, dy: float,
                   xmin: float, ymin: float, xmax: float, ymax: float,
                   max_r: float) -> float:
    """Ray-AABB slab intersection. Returns hit distance, or max_r if no hit."""
    t_min, t_max = 0.0, max_r
    if abs(dx) > 1e-9:
        t1 = (xmin - ox) / dx
        t2 = (xmax - ox) / dx
        t_min = max(t_min, min(t1, t2))
        t_max = min(t_max, max(t1, t2))
    elif not (xmin <= ox <= xmax):
        return max_r
    if abs(dy) > 1e-9:
        t1 = (ymin - oy) / dy
        t2 = (ymax - oy) / dy
        t_min = max(t_min, min(t1, t2))
        t_max = min(t_max, max(t1, t2))
    elif not (ymin <= oy <= ymax):
        return max_r
    if t_max < t_min or t_min <= 0.0:
        return max_r
    return t_min


class LidarNode(Node):

    def __init__(self) -> None:
        super().__init__('lidar_node')

        p = self.declare_parameter
        p('robot_alias',  '/RoboMasterEP/BaseLinkFrame')
        p('scan_rate_hz', 10.0)
        p('n_rays',       360)
        p('max_range',    5.0)
        p('laser_frame',  'laser')
        p('base_frame',   'base_link')
        p('odom_frame',   'odom')
        p('laser_height', 0.12)

        g = lambda n: self.get_parameter(n).value
        self._max_range   = float(g('max_range'))
        self._laser_frame = str(g('laser_frame'))
        self._base_frame  = str(g('base_frame'))
        self._odom_frame  = str(g('odom_frame'))
        self._laser_z     = float(g('laser_height'))
        n_rays            = int(g('n_rays'))
        rate              = float(g('scan_rate_hz'))

        angles = np.linspace(-math.pi, math.pi, n_rays, endpoint=False)
        self._ray_dx = np.cos(angles).tolist()
        self._ray_dy = np.sin(angles).tolist()
        self._angle_min = float(angles[0])
        self._angle_inc = float(angles[1] - angles[0])

        self._client = RemoteAPIClient()
        self._sim    = self._client.require('sim')
        self._robot_h = self._sim.getObject(g('robot_alias'))

        self._obstacle_specs = self._enumerate_obstacles()
        self.get_logger().info(
            f'found {len(self._obstacle_specs)} obstacle(s) in scene')

        self._scan_pub  = self.create_publisher(LaserScan, '/scan', 10)
        self._static_tf = StaticTransformBroadcaster(self)
        self._tf_pub    = TransformBroadcaster(self)
        self._publish_static_tf()

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'ready; {n_rays} rays, {self._max_range} m range, {rate} Hz')

    # ===== init =========================================================

    def _enumerate_obstacles(self) -> list[dict]:
        sim = self._sim
        out: list[dict] = []
        for h in sim.getObjectsInTree(sim.handle_scene):
            alias = sim.getObjectAlias(h, 0)
            if not alias.startswith(_OBSTACLE_PREFIXES):
                continue
            extents = (
                sim.getObjectFloatParam(h, sim.objfloatparam_objbbox_min_x),
                sim.getObjectFloatParam(h, sim.objfloatparam_objbbox_min_y),
                sim.getObjectFloatParam(h, sim.objfloatparam_objbbox_max_x),
                sim.getObjectFloatParam(h, sim.objfloatparam_objbbox_max_y),
            )
            out.append({'handle': int(h), 'extents': extents})
        return out

    def _publish_static_tf(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._base_frame
        t.child_frame_id  = self._laser_frame
        t.transform.translation.z = self._laser_z
        t.transform.rotation.w    = 1.0
        self._static_tf.sendTransform(t)

    def _publish_odom_tf(self, x: float, y: float, z: float,
                         yaw: float) -> None:
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = self._odom_frame
        t.child_frame_id  = self._base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self._tf_pub.sendTransform(t)

    # ===== per-tick =====================================================

    def _obstacle_aabbs(self) -> list[tuple[float, float, float, float]]:
        sim = self._sim
        aabbs = []
        for spec in self._obstacle_specs:
            pos = sim.getObjectPosition(spec['handle'], -1)
            if pos[2] < _BELOW_FLOOR_Z:
                continue
            yaw = sim.getObjectOrientation(spec['handle'], -1)[2]
            x0, y0, x1, y1 = spec['extents']
            c, s = math.cos(yaw), math.sin(yaw)
            corners = [(x0, y0), (x0, y1), (x1, y0), (x1, y1)]
            wx = [pos[0] + c * x - s * y for x, y in corners]
            wy = [pos[1] + s * x + c * y for x, y in corners]
            aabbs.append((min(wx), min(wy), max(wx), max(wy)))
        return aabbs

    def _tick(self) -> None:
        mat = self._sim.getObjectMatrix(self._robot_h, -1)
        rx, ry, rz = float(mat[3]), float(mat[7]), float(mat[11])
        yaw = math.atan2(float(mat[4]), float(mat[0]))
        self._publish_odom_tf(rx, ry, rz, yaw)

        aabbs = self._obstacle_aabbs()
        max_r   = self._max_range
        ranges: list[float] = []

        for dx, dy in zip(self._ray_dx, self._ray_dy):
            d = max_r
            for xmin, ymin, xmax, ymax in aabbs:
                hit = _ray_aabb_dist(rx, ry, dx, dy, xmin, ymin, xmax, ymax, d)
                if hit < d:
                    d = hit
            ranges.append(d)

        msg = LaserScan()
        msg.header.stamp      = self.get_clock().now().to_msg()
        msg.header.frame_id   = self._laser_frame
        msg.angle_min         = self._angle_min
        msg.angle_max         = -self._angle_min
        msg.angle_increment   = self._angle_inc
        msg.time_increment    = 0.0
        msg.scan_time         = 1.0 / 10.0
        msg.range_min         = 0.05
        msg.range_max         = max_r
        msg.ranges            = ranges
        self._scan_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarNode()
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
