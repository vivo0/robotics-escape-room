#!/usr/bin/env python3
"""
ROS2 node that builds a 2D occupancy grid by ray-casting the Velodyne
point cloud onto a top-down map in the `odom` frame.

Subscribes:
    /velodyne_points  (sensor_msgs/PointCloud2, in `velodyne` frame)

Publishes:
    /map              (nav_msgs/OccupancyGrid, in `odom` frame, ~2 Hz)

The grid lives in odom; we rely on TF (velodyne -> odom) to bring the
points into a consistent frame. Cells along each ray are marked free,
the ray endpoint is marked occupied. Cells never seen stay unknown.
"""
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time as RclpyTime

from nav_msgs.msg import OccupancyGrid as OccupancyGridMsg
from sensor_msgs.msg import PointCloud2

import tf2_ros

from escape_room.mapping import OccupancyGrid, pointcloud2_to_xyz
from escape_room.mapping.occupancy_grid import GridSpec


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Quaternion (xyzw) to 3x3 rotation matrix."""
    n = qx*qx + qy*qy + qz*qz + qw*qw
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s*(qy*qy + qz*qz), s*(qx*qy - qz*qw),     s*(qx*qz + qy*qw)],
        [s*(qx*qy + qz*qw),     1 - s*(qx*qx + qz*qz), s*(qy*qz - qx*qw)],
        [s*(qx*qz - qy*qw),     s*(qy*qz + qx*qw),     1 - s*(qx*qx + qy*qy)],
    ])


def transform_to_matrix(tf_msg) -> np.ndarray:
    """geometry_msgs/TransformStamped -> 4x4 homogeneous matrix."""
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    R = quat_to_rot(q.x, q.y, q.z, q.w)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = (t.x, t.y, t.z)
    return T


class MapperNode(Node):
    def __init__(self):
        super().__init__('mapper_node')

        # Grid is in `odom` frame; origin = robot spawn. Default 10x10 m
        # centered on the spawn covers a 5x4 room with the robot anywhere.
        self.declare_parameter('grid_width_m', 10.0)
        self.declare_parameter('grid_height_m', 10.0)
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('origin_x', -5.0)
        self.declare_parameter('origin_y', -5.0)
        # z slice in odom: keep enough vertical band that beams at small
        # tilt still hit walls even far from the sensor.
        self.declare_parameter('z_min', 0.03)
        self.declare_parameter('z_max', 0.55)
        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('cloud_topic', '/velodyne_points')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('tf_lookup_timeout_s', 0.1)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.tf_timeout = Duration(
            seconds=float(self.get_parameter('tf_lookup_timeout_s').value))

        spec = GridSpec(
            width_m=float(self.get_parameter('grid_width_m').value),
            height_m=float(self.get_parameter('grid_height_m').value),
            resolution=float(self.get_parameter('resolution').value),
            origin_x=float(self.get_parameter('origin_x').value),
            origin_y=float(self.get_parameter('origin_y').value),
        )
        self.grid = OccupancyGrid(spec)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.get_parameter('cloud_topic').value,
            self.on_cloud,
            10,
        )

        # Latched-style QoS so a late-joining RViz still receives the map.
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

        publish_rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / publish_rate, self.publish_map)

        self._cloud_count = 0
        self._last_warn = 0.0

        self.get_logger().info(
            f'mapper_node ready. grid={spec.width_m}x{spec.height_m} m '
            f'@ {spec.resolution} m/cell ({self.grid.cols}x{self.grid.rows}); '
            f'z slice [{self.z_min:.2f}, {self.z_max:.2f}]; '
            f'max_range={self.max_range} m; odom_frame={self.odom_frame}'
        )

    def on_cloud(self, msg: PointCloud2) -> None:
        sensor_frame = msg.header.frame_id
        try:
            # Use Time() (= latest available TF) instead of the cloud's exact
            # stamp. Velodyne and robomaster_ros publish with different
            # clocks (real-time vs sim-time when the sim runs slow), so a
            # stamp-exact lookup explodes with "extrapolation into the
            # future". The robot moves slowly so the spatial error from
            # using the latest TF is negligible.
            tf_msg = self.tf_buffer.lookup_transform(
                self.odom_frame,
                sensor_frame,
                RclpyTime(),
                timeout=self.tf_timeout,
            )
        except tf2_ros.TransformException as e:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_warn > 2.0:
                self.get_logger().warn(
                    f'TF {sensor_frame} -> {self.odom_frame} unavailable: {e}'
                )
                self._last_warn = now
            return

        pts = pointcloud2_to_xyz(msg)
        if pts.shape[0] == 0:
            return

        T = transform_to_matrix(tf_msg)
        pts_h = np.column_stack([pts, np.ones(pts.shape[0], dtype=np.float32)])
        pts_odom = pts_h @ T.T  # (N, 4)

        # filter to horizontal slice
        mask = (pts_odom[:, 2] >= self.z_min) & (pts_odom[:, 2] <= self.z_max)
        slice_xy = pts_odom[mask, :2]

        # also drop NaNs / huge points
        finite = np.isfinite(slice_xy).all(axis=1)
        slice_xy = slice_xy[finite]

        sensor_xy = (T[0, 3], T[1, 3])
        self.grid.update_from_scan(sensor_xy, slice_xy, self.max_range)
        self._cloud_count += 1

    def publish_map(self) -> None:
        if self._cloud_count == 0:
            return  # nothing observed yet
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
        # row-major, x varies fastest
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
