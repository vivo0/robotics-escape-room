#!/usr/bin/env python3
"""Color landmark detector for the discovery phase.

Looks for three coloured landmarks in the robot's camera image:

    cube  (magenta cuboid) → sim alias /TargetCube
    plate (green square)   → sim alias /PressurePlate
    door  (blue rectangle) → sim alias /Door_0

The largest HSV-matching connected component is the detection. The
first time a target clears its pixel threshold we publish:

* the sim-truth pose on ``/targets/<name>`` — used by ``explorer_node``
  for navigation;
* a marker at the *camera-estimated* position on ``/targets/markers``,
  i.e. the blob centroid back-projected through the pinhole and
  pushed to the depth implied by the known target size. So the user
  can see in RViz where the robot *thinks* it saw the target.

Subscribes:
    /camera/image_color    (sensor_msgs/Image)

Publishes (latched):
    /targets/{cube,plate,door}  (geometry_msgs/PoseStamped)
    /targets/markers            (visualization_msgs/MarkerArray)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def _hsv_range(h_lo: int, h_hi: int,
               s_lo: int = 60, s_hi: int = 255,
               v_lo: int = 60, v_hi: int = 255
               ) -> tuple[np.ndarray, np.ndarray]:
    """OpenCV HSV (lo, hi) pair. H is in [0, 179]."""
    return (np.array([h_lo, s_lo, v_lo], dtype=np.uint8),
            np.array([h_hi, s_hi, v_hi], dtype=np.uint8))


@dataclass
class ColorTarget:
    name: str
    sim_alias: str
    min_pixels: int
    rgb: tuple[float, float, float]   # marker colour in RViz
    real_size_m: float                # used for monocular depth recovery
    hsv_ranges: list = field(default_factory=list)


# Cube is magenta (not red, which the Velodyne beams paint; not yellow,
# which the default floor pattern uses).
_TARGETS: tuple[ColorTarget, ...] = (
    ColorTarget('cube',  '/TargetCube',     80, (0.9, 0.1, 0.9), 0.12,
                [_hsv_range(140, 170)]),
    ColorTarget('plate', '/PressurePlate', 200, (0.1, 1.0, 0.1), 0.30,
                [_hsv_range(40, 80)]),
    ColorTarget('door',  '/Door_0',        300, (0.1, 0.3, 1.0), 0.80,
                [_hsv_range(100, 130)]),
)


class ColorDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('color_detector_node')

        self.declare_parameter('image_topic', '/camera/image_color')
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('camera_alias', '/RoboMasterEP/Camera')
        image_topic = str(self.get_parameter('image_topic').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        camera_alias = str(self.get_parameter('camera_alias').value)

        # ---- sim handles --------------------------------------------
        # Keep the client as a member: anonymous clients can be
        # garbage-collected, dropping the ZMQ connection.
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')

        self._handles: dict[str, int | None] = {}
        for t in _TARGETS:
            try:
                self._handles[t.name] = self.sim.getObject(t.sim_alias)
            except Exception:
                self.get_logger().warn(
                    f"could not resolve '{t.sim_alias}'; "
                    f"'{t.name}' will be ignored")
                self._handles[t.name] = None

        self._camera_handle, self._cam_res, self._cam_fov = (
            self._resolve_camera(camera_alias))

        # ---- ROS pub/sub --------------------------------------------
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._target_pubs = {
            t.name: self.create_publisher(
                PoseStamped, f'/targets/{t.name}', latched)
            for t in _TARGETS
        }
        self._marker_pub = self.create_publisher(
            MarkerArray, '/targets/markers', latched)
        self._published: set[str] = set()
        self._marker_poses: dict[str, tuple[float, float, float]] = {}

        self._bridge = CvBridge()
        self.create_subscription(Image, image_topic, self._on_image, 10)
        self.get_logger().info(f'ready, listening on {image_topic}')

    def _resolve_camera(self, hint: str
                        ) -> tuple[int | None,
                                   tuple[int, int] | None,
                                   float | None]:
        """Resolve the vision sensor for camera-side marker estimates.
        Tries ``hint`` first; on failure falls back to the first
        vision sensor in the scene. Both attempts are guarded — if
        nothing resolves, markers transparently use sim-truth poses."""
        sim = self.sim
        try:
            handle = sim.getObject(hint)
        except Exception:
            handle = self._first_vision_sensor()
            if handle is None:
                self.get_logger().warn(
                    'no vision sensor found; markers will use sim poses')
                return None, None, None
        try:
            res = (
                int(sim.getObjectInt32Param(
                    handle, sim.visionintparam_resolution_x)),
                int(sim.getObjectInt32Param(
                    handle, sim.visionintparam_resolution_y)),
            )
            fov = float(sim.getObjectFloatParam(
                handle, sim.visionfloatparam_perspective_angle))
        except Exception as e:
            self.get_logger().warn(
                f'could not read camera intrinsics: {e}; '
                f'markers will use sim poses')
            return None, None, None
        self.get_logger().info(
            f'camera resolved: res={res}, fov={math.degrees(fov):.1f}°')
        return handle, res, fov

    def _first_vision_sensor(self) -> int | None:
        """Walk the scene and return the first vision sensor handle.
        Object-type filter API names vary across CoppeliaSim versions,
        so we filter by per-object type instead of passing a constant."""
        sim = self.sim
        try:
            objs = sim.getObjectsInTree(sim.handle_scene)
        except Exception:
            return None
        # Vision sensor type integer = 9 in CoppeliaSim's object type
        # enum (sim_object_visionsensor_type).
        for h in objs:
            try:
                if int(sim.getObjectType(int(h))) == 9:
                    return int(h)
            except Exception:
                continue
        return None

    # ===== callbacks ====================================================

    def _on_image(self, msg: Image) -> None:
        if len(self._published) == len(_TARGETS):
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}',
                                   throttle_duration_sec=2.0)
            return
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        for t in _TARGETS:
            if t.name in self._published:
                continue
            det = self._detect(hsv, t)
            if det is not None:
                self._publish_target(t, det)

    # ===== detection / publishing =======================================

    def _detect(self, hsv: np.ndarray, target: ColorTarget
                ) -> tuple[float, float, int] | None:
        """Largest connected component matching the HSV windows.
        Returns (centroid_x_px, centroid_y_px, largest_bbox_dim_px), or
        ``None`` if no blob clears ``target.min_pixels``."""
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in target.hsv_ranges:
            mask |= cv2.inRange(hsv, lo, hi)
        n, _, stats, centroids = cv2.connectedComponentsWithStats(
            mask, connectivity=8)
        if n < 2:
            return None
        idx = int(stats[1:, cv2.CC_STAT_AREA].argmax()) + 1
        if int(stats[idx, cv2.CC_STAT_AREA]) < target.min_pixels:
            return None
        cx = float(centroids[idx, 0])
        cy = float(centroids[idx, 1])
        size_px = max(int(stats[idx, cv2.CC_STAT_WIDTH]),
                      int(stats[idx, cv2.CC_STAT_HEIGHT]))
        return cx, cy, size_px

    def _publish_target(self, target: ColorTarget,
                        detection: tuple[float, float, int]) -> None:
        handle = self._handles.get(target.name)
        if handle is None:
            return
        pos = self.sim.getObjectPosition(handle, -1)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.w = 1.0
        self._target_pubs[target.name].publish(msg)
        self._published.add(target.name)

        cam = self._estimate_world_xyz(target, *detection)
        marker_xyz = cam if cam is not None else (
            float(pos[0]), float(pos[1]), float(pos[2]))
        self._marker_poses[target.name] = marker_xyz
        self._publish_markers()

        if cam is not None:
            err = math.hypot(cam[0] - pos[0], cam[1] - pos[1])
            self.get_logger().info(
                f'[{target.name}] sim ({pos[0]:.2f}, {pos[1]:.2f}) → '
                f'cam ({cam[0]:.2f}, {cam[1]:.2f}); err={err:.2f} m')
        else:
            self.get_logger().info(
                f'[{target.name}] @ ({pos[0]:.2f}, {pos[1]:.2f})')

    def _estimate_world_xyz(self, target: ColorTarget,
                            cx: float, cy: float, size_px: int
                            ) -> tuple[float, float, float] | None:
        """Pinhole back-projection of the blob centroid plus monocular
        depth from the known target size:
        ``depth = f_pix * real_size / pixel_size``."""
        if (self._camera_handle is None or self._cam_res is None
                or self._cam_fov is None or size_px <= 0):
            return None
        W, H = self._cam_res
        # CoppeliaSim's perspective_angle spans the longer image axis.
        f_pix = max(W, H) / (2.0 * math.tan(self._cam_fov / 2.0))

        nx = (cx - W / 2.0) / f_pix
        ny = (cy - H / 2.0) / f_pix
        ray_cam = np.array([nx, ny, 1.0])
        ray_cam /= np.linalg.norm(ray_cam)

        mat = self.sim.getObjectMatrix(self._camera_handle, -1)
        M = np.array(mat).reshape(3, 4)
        ray_world = M[:, :3] @ ray_cam
        depth = (target.real_size_m * f_pix) / size_px
        point = M[:, 3] + ray_world * depth
        return float(point[0]), float(point[1]), float(point[2])

    def _publish_markers(self) -> None:
        """Sphere + label per detected target. Re-published on every
        new detection so a late-joining RViz still gets the latched
        snapshot."""
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        for t in _TARGETS:
            xyz = self._marker_poses.get(t.name)
            if xyz is None:
                continue
            arr.markers.append(self._sphere_marker(t, xyz, stamp))
            arr.markers.append(self._label_marker(t, xyz, stamp))
        self._marker_pub.publish(arr)

    def _sphere_marker(self, t: ColorTarget,
                       xyz: tuple[float, float, float], stamp) -> Marker:
        m = Marker()
        m.header.frame_id = self.target_frame
        m.header.stamp = stamp
        m.ns = 'targets'
        m.id = hash(t.name) & 0xFFFF
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.18
        m.color.r, m.color.g, m.color.b = t.rgb
        m.color.a = 0.85
        return m

    def _label_marker(self, t: ColorTarget,
                      xyz: tuple[float, float, float], stamp) -> Marker:
        m = Marker()
        m.header.frame_id = self.target_frame
        m.header.stamp = stamp
        m.ns = 'target_labels'
        m.id = hash(t.name) & 0xFFFF
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = xyz[0]
        m.pose.position.y = xyz[1]
        m.pose.position.z = xyz[2] + 0.30
        m.pose.orientation.w = 1.0
        m.scale.z = 0.18
        m.color.r = m.color.g = m.color.b = 1.0
        m.color.a = 1.0
        m.text = t.name
        return m


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(ColorDetectorNode())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
