#!/usr/bin/env python3
"""Perception-only colour landmark detector.

Finds three coloured landmarks in the camera image and publishes their map
poses on ``/targets/{cube,plate,door}`` — no object pose is read from the
simulator. Localisation is either monocular depth from the known object
HEIGHT (cube, door) or a camera-ray / floor-plane intersection (plate, which
is flat on the ground). The cube is also streamed in ``base_link`` on
``/targets/cube_live`` for the explorer's visual-servoing pickup.

The only sim read is the camera FOV (intrinsic calibration, not a pose).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
import rclpy.time
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker, MarkerArray

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

_CAMERA_FRAME = 'camera_optical_link'
_BASE_FRAME = 'base_link'


def _hsv(h_lo, h_hi, s_lo=60, v_lo=60):
    """OpenCV HSV (lo, hi) pair. H is in [0, 179]."""
    return (np.array([h_lo, s_lo, v_lo], np.uint8),
            np.array([h_hi, 255, 255], np.uint8))


@dataclass
class Target:
    name: str
    min_pixels: int
    rgb: tuple
    hsv: tuple              # (lo, hi) HSV bounds
    localize: str           # 'mono' (depth from height) or 'ground'
    height_m: float = 0.0   # mono: real object height (m)
    floor_z: float = 0.0    # ground: support-plane height (m)


# Cube/door: vertical → monocular depth from height. Plate: flat → floor ray.
_CUBE = Target('cube', 80, (0.9, 0.1, 0.9), _hsv(140, 170), 'mono', height_m=0.20)
_LANDMARKS = (
    Target('plate', 200, (0.1, 1.0, 0.1), _hsv(40, 80), 'ground', floor_z=0.0),
    Target('door', 300, (0.1, 0.3, 1.0), _hsv(100, 130), 'mono', height_m=0.50),
)
_TARGETS = (_CUBE, *_LANDMARKS)


class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')
        image_topic = self.declare_parameter(
            'image_topic', '/camera/image_color').value
        self._map = self.declare_parameter('map_frame', 'map').value

        self._tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf, self)

        self._client = RemoteAPIClient()            # kept alive: GC drops ZMQ
        self._fov = self._camera_fov()

        latched = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._pubs = {
            t.name: self.create_publisher(PoseStamped, f'/targets/{t.name}', latched)
            for t in _TARGETS}
        self._cube_live = self.create_publisher(
            PoseStamped, '/targets/cube_live', 10)
        self._markers = self.create_publisher(
            MarkerArray, '/targets/markers', latched)
        self._poses: dict[str, tuple] = {}

        self._bridge = CvBridge()
        self.create_subscription(Image, image_topic, self._on_image, 10)
        self.get_logger().info(f'ready, listening on {image_topic}')

    def _camera_fov(self) -> float:
        """Perspective angle of the scene's vision sensor (type 9)."""
        sim = self._client.require('sim')
        for h in sim.getObjectsInTree(sim.handle_scene):
            if int(sim.getObjectType(int(h))) == 9:
                fov = float(sim.getObjectFloatParam(
                    int(h), sim.visionfloatparam_perspective_angle))
                self.get_logger().info(f'camera fov={math.degrees(fov):.1f}deg')
                return fov
        raise RuntimeError('no vision sensor in scene')

    # ===== detection ====================================================

    def _on_image(self, msg: Image) -> None:
        hsv = cv2.cvtColor(
            self._bridge.imgmsg_to_cv2(msg, 'bgr8'), cv2.COLOR_BGR2HSV)
        h, w = hsv.shape[:2]
        cube = self._detect(hsv, _CUBE)
        if cube:
            self._publish_cube(cube, w, h)
        for t in _LANDMARKS:
            det = self._detect(hsv, t)
            if det:
                self._publish_landmark(t, det, w, h)

    @staticmethod
    def _detect(hsv, t: Target):
        """Largest HSV blob → (cx, cy, w_px, h_px), or None below threshold."""
        mask = cv2.inRange(hsv, *t.hsv)
        n, _, stats, cent = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if n < 2:
            return None
        i = int(stats[1:, cv2.CC_STAT_AREA].argmax()) + 1
        if int(stats[i, cv2.CC_STAT_AREA]) < t.min_pixels:
            return None
        return (float(cent[i, 0]), float(cent[i, 1]),
                int(stats[i, cv2.CC_STAT_WIDTH]), int(stats[i, cv2.CC_STAT_HEIGHT]))

    # ===== localisation (perception only) ===============================

    def _f_pix(self, w, h) -> float:
        # CoppeliaSim's perspective angle spans the longer image axis.
        return max(w, h) / (2.0 * math.tan(self._fov / 2.0))

    def _mono(self, cx, cy, h_px, height_m, w, h) -> tuple:
        """Object centre in camera_optical_link from monocular height depth."""
        f = self._f_pix(w, h)
        z = height_m * f / h_px
        return (cx - w / 2.0) / f * z, (cy - h / 2.0) / f * z, z

    def _floor(self, cx, cy, z_plane, w, h):
        """Map (x, y) where the centroid ray meets the floor plane."""
        f = self._f_pix(w, h)
        tf = self._lookup(self._map, _CAMERA_FRAME)
        if tf is None:
            return None
        o = do_transform_point(_ps(0.0, 0.0, 0.0), tf).point
        p = do_transform_point(_ps((cx - w / 2.0) / f, (cy - h / 2.0) / f, 1.0), tf).point
        if p.z - o.z >= -1e-6:            # ray not pointing down → no floor hit
            return None
        s = (z_plane - o.z) / (p.z - o.z)
        return o.x + s * (p.x - o.x), o.y + s * (p.y - o.y)

    @staticmethod
    def _clipped(cy, h_px, img_h) -> bool:
        """Blob touches the top/bottom edge → its height (depth) is unreliable."""
        return cy - h_px / 2.0 <= 2.0 or cy + h_px / 2.0 >= img_h - 2.0

    # ===== publishing ===================================================

    def _publish_cube(self, det, w, h) -> None:
        cx, cy, _, h_px = det
        ps = _ps(*self._mono(cx, cy, h_px, _CUBE.height_m, w, h))
        bl = self._transform(ps, _BASE_FRAME)
        if bl is None:                    # TF not ready yet
            return
        self._cube_live.publish(_pose(bl.point.x, bl.point.y, 0.0, _BASE_FRAME,
                                      self.get_clock().now().to_msg()))
        if not self._clipped(cy, h_px, h):
            mp = self._transform(ps, self._map)
            if mp is not None:
                self._publish_map('cube', mp.point.x, mp.point.y, mp.point.z)

    def _publish_landmark(self, t: Target, det, w, h) -> None:
        cx, cy, _, h_px = det
        if t.localize == 'ground':
            gp = self._floor(cx, cy, t.floor_z, w, h)
            if gp is not None:
                self._publish_map(t.name, gp[0], gp[1], t.floor_z)
            return
        if self._clipped(cy, h_px, h):
            return
        mp = self._transform(_ps(*self._mono(cx, cy, h_px, t.height_m, w, h)), self._map)
        if mp is not None:
            self._publish_map(t.name, mp.point.x, mp.point.y, mp.point.z)

    def _publish_map(self, name, x, y, z) -> None:
        self._pubs[name].publish(
            _pose(x, y, z, self._map, self.get_clock().now().to_msg()))
        prev = self._poses.get(name)
        if prev is None or math.hypot(x - prev[0], y - prev[1]) > 0.05:
            self._poses[name] = (x, y, z)
            self._publish_markers()

    # ===== TF + markers =================================================

    def _lookup(self, target, source):
        try:
            return self._tf.lookup_transform(target, source, rclpy.time.Time())
        except Exception:
            return None

    def _transform(self, ps, frame):
        tf = self._lookup(frame, ps.header.frame_id)
        return do_transform_point(ps, tf) if tf is not None else None

    def _publish_markers(self) -> None:
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        for t in _TARGETS:
            xyz = self._poses.get(t.name)
            if xyz is None:
                continue
            m = Marker()
            m.header.frame_id = self._map
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
            arr.markers.append(m)
        self._markers.publish(arr)


def _ps(x, y, z) -> PointStamped:
    ps = PointStamped()
    ps.header.frame_id = _CAMERA_FRAME
    ps.point.x, ps.point.y, ps.point.z = x, y, z
    return ps


def _pose(x, y, z, frame, stamp) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.header.stamp = stamp
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
    msg.pose.orientation.w = 1.0
    return msg


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
