#!/usr/bin/env python3
"""Finds the cube/plate/door by colour and publishes their map poses.

Everything is from the camera + TF (no object position is read from the sim).
Cube and door (tall) use monocular depth from their known height; the plate
(flat) uses a camera-ray / floor-plane intersection.
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

# the camera image frame (z forward, x right, y down) and the robot base frame
_CAMERA_FRAME = 'camera_optical_link'
_BASE_FRAME = 'base_link'


def _hsv(h_lo, h_hi, s_lo=60, v_lo=60):
    # build an OpenCV HSV lower/upper bound; OpenCV hue is 0..179, not 0..359
    return (np.array([h_lo, s_lo, v_lo], np.uint8),
            np.array([h_hi, 255, 255], np.uint8))


@dataclass
class Target:
    name: str
    min_pixels: int         # ignore blobs smaller than this (noise)
    rgb: tuple              # colour of the RViz marker
    hsv: tuple              # (lo, hi) HSV bounds for the mask
    localize: str           # 'mono' (depth from height) or 'ground'
    height_m: float = 0.0   # mono: real object height (m)
    floor_z: float = 0.0    # ground: support-plane height (m)


# magenta cube / blue door are upright → monocular; green plate is flat → floor ray
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
        self._fov = self._camera_fov()              # camera field of view (rad)

        # latched so a late RViz / explorer still gets the last published pose
        latched = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # one /targets/<name> publisher per landmark (pose in the map frame)
        self._pubs = {
            t.name: self.create_publisher(PoseStamped, f'/targets/{t.name}', latched)
            for t in _TARGETS}
        # extra stream: cube pose in base_link every frame, for the pickup
        self._cube_live = self.create_publisher(
            PoseStamped, '/targets/cube_live', 10)
        self._markers = self.create_publisher(
            MarkerArray, '/targets/markers', latched)
        self._poses: dict[str, tuple] = {}   # last published pose per landmark

        self._bridge = CvBridge()
        self.create_subscription(Image, image_topic, self._on_image, 10)
        self.get_logger().info(f'ready, listening on {image_topic}')

    def _camera_fov(self) -> float:
        # read the camera's field of view from the sim (type 9 = vision sensor).
        # this is calibration (a constant), not the camera's position.
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
        # work in HSV: colour thresholding is much easier than in RGB
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
        # threshold the colour, then keep the biggest blob of that colour
        mask = cv2.inRange(hsv, *t.hsv)
        n, _, stats, cent = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if n < 2:                          # label 0 is the background
            return None
        # argmax over labels 1.. (skip background); +1 to undo the [1:] offset
        i = int(stats[1:, cv2.CC_STAT_AREA].argmax()) + 1
        if int(stats[i, cv2.CC_STAT_AREA]) < t.min_pixels:
            return None
        # centroid + bounding-box width/height in pixels
        return (float(cent[i, 0]), float(cent[i, 1]),
                int(stats[i, cv2.CC_STAT_WIDTH]), int(stats[i, cv2.CC_STAT_HEIGHT]))

    # ===== localisation ===============================

    def _f_pix(self, w, h) -> float:
        # CoppeliaSim's perspective angle spans the longer image axis.
        return max(w, h) / (2.0 * math.tan(self._fov / 2.0))

    def _mono(self, cx, cy, h_px, height_m, w, h) -> tuple:
        # pinhole + known height: a real height of height_m showing as h_px
        # pixels is at depth z = f * height / h_px. Then back-project the
        # centroid to that depth to get the 3D point in the camera frame.
        f = self._f_pix(w, h)
        z = height_m * f / h_px
        return (cx - w / 2.0) / f * z, (cy - h / 2.0) / f * z, z

    def _floor(self, cx, cy, z_plane, w, h):
        # for a flat object we can't use height, so shoot a ray through the
        # pixel and intersect it with the floor plane (no size needed).
        f = self._f_pix(w, h)
        tf = self._lookup(self._map, _CAMERA_FRAME)
        if tf is None:
            return None
        # camera origin and a point one unit down the ray, both in map frame
        o = do_transform_point(_ps(0.0, 0.0, 0.0), tf).point
        p = do_transform_point(_ps((cx - w / 2.0) / f, (cy - h / 2.0) / f, 1.0), tf).point
        if p.z - o.z >= -1e-6:            # ray not pointing down → no floor hit
            return None
        # walk along the ray until its z reaches the floor, return that (x, y)
        s = (z_plane - o.z) / (p.z - o.z)
        return o.x + s * (p.x - o.x), o.y + s * (p.y - o.y)

    @staticmethod
    def _clipped(cy, h_px, img_h) -> bool:
        # if the blob touches the top/bottom edge its true height is cut off,
        # so the monocular depth would be wrong: skip those frames
        return cy - h_px / 2.0 <= 2.0 or cy + h_px / 2.0 >= img_h - 2.0

    # ===== publishing ===================================================

    def _publish_cube(self, det, w, h) -> None:
        cx, cy, _, h_px = det
        ps = _ps(*self._mono(cx, cy, h_px, _CUBE.height_m, w, h))  # point in cam frame
        bl = self._transform(ps, _BASE_FRAME)
        if bl is None:                    # TF not ready yet
            return
        # live stream for the pickup: only x/y bearing matters, z unused
        self._cube_live.publish(_pose(bl.point.x, bl.point.y, 0.0, _BASE_FRAME,
                                      self.get_clock().now().to_msg()))
        # the map pose is only published while the height is reliable (not clipped)
        if not self._clipped(cy, h_px, h):
            mp = self._transform(ps, self._map)
            if mp is not None:
                self._publish_map('cube', mp.point.x, mp.point.y, mp.point.z)

    def _publish_landmark(self, t: Target, det, w, h) -> None:
        cx, cy, _, h_px = det
        if t.localize == 'ground':        # plate: floor-ray intersection
            gp = self._floor(cx, cy, t.floor_z, w, h)
            if gp is not None:
                self._publish_map(t.name, gp[0], gp[1], t.floor_z)
            return
        if self._clipped(cy, h_px, h):    # door: monocular, skip if cut off
            return
        mp = self._transform(_ps(*self._mono(cx, cy, h_px, t.height_m, w, h)), self._map)
        if mp is not None:
            self._publish_map(t.name, mp.point.x, mp.point.y, mp.point.z)

    def _publish_map(self, name, x, y, z) -> None:
        # publish the (refined) pose every frame; only redraw the marker when
        # it actually moved, to avoid spamming RViz
        self._pubs[name].publish(
            _pose(x, y, z, self._map, self.get_clock().now().to_msg()))
        prev = self._poses.get(name)
        if prev is None or math.hypot(x - prev[0], y - prev[1]) > 0.05:
            self._poses[name] = (x, y, z)
            self._publish_markers()

    # ===== TF + markers =================================================

    def _lookup(self, target, source):
        # latest transform target<-source, or None if TF isn't ready
        try:
            return self._tf.lookup_transform(target, source, rclpy.time.Time())
        except Exception:
            return None

    def _transform(self, ps, frame):
        # move a point from its own frame into `frame` using TF
        tf = self._lookup(frame, ps.header.frame_id)
        return do_transform_point(ps, tf) if tf is not None else None

    def _publish_markers(self) -> None:
        # one coloured sphere per known landmark, for RViz
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
            m.id = hash(t.name) & 0xFFFF   # stable id per name so it updates in place
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
    # small helper: a point in the camera frame, ready to be TF-transformed
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
