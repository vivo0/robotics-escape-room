#!/usr/bin/env python3
"""Color landmark detector for the discovery phase.

Looks for three coloured landmarks in the robot's camera image (cube =
magenta, plate = green, door = blue). The largest HSV-matching connected
component is the detection. All three are localised **purely from
perception** — no object pose is ever read from the simulator:

  * **cube** — a thin upright column. Monocular depth from its known HEIGHT
    back-projects the centroid in ``camera_optical_link``, transformed to
    map/base_link via the live TF tree. Published every frame in
    ``base_link`` on ``/targets/cube_live`` (for the explorer's
    visual-servoing pickup) and as a continuously-refined map pose.
  * **door** — a vertical wall panel. Same monocular-height method as the
    cube (height is horizontally angle-invariant and the panel is fully in
    frame from across the room).
  * **plate** — a flat square on the floor (≈0 pixel height). Its centroid
    ray is intersected with the floor plane (z=0) via the camera pose from
    TF — no size estimate needed.

Each map pose is re-published while the landmark is visible, so the estimate
sharpens as the robot approaches. The only sim read is the camera
intrinsics (resolution/FOV — calibration, not a pose) and, behind a
diagnostic log, the cube's ground-truth pose used solely to report error.

Subscribes:
    /camera/image_color    (sensor_msgs/Image)

Publishes:
    /targets/cube_live          (geometry_msgs/PoseStamped, frame: base_link)
    /targets/{cube,plate,door}  (geometry_msgs/PoseStamped, frame: map, latched)
    /targets/markers            (visualization_msgs/MarkerArray,  frame: map)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np
import rclpy
import rclpy.time
import tf2_ros
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Image
from tf2_geometry_msgs import do_transform_point
from visualization_msgs.msg import Marker, MarkerArray

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def _hsv_range(h_lo: int, h_hi: int,
               s_lo: int = 60, s_hi: int = 255,
               v_lo: int = 60, v_hi: int = 255
               ) -> tuple[np.ndarray, np.ndarray]:
    """OpenCV HSV (lo, hi) pair. H is in [0, 179]."""
    return (np.array([h_lo, s_lo, v_lo], dtype=np.uint8),
            np.array([h_hi, s_hi, v_hi], dtype=np.uint8))


def _yaw_from_quat(q) -> float:
    """CoppeliaSim quaternion [x, y, z, w] → yaw (rad)."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


@dataclass
class ColorTarget:
    name: str
    min_pixels: int
    rgb: tuple[float, float, float]    # marker colour in RViz
    hsv_ranges: list = field(default_factory=list)
    # Localisation method (all perception, no ground truth):
    #   'mono'   — monocular depth from the object's known HEIGHT.
    #   'ground' — intersect the pixel ray with the floor plane.
    localize: str = 'mono'
    real_size_m: float = 0.0           # mono: real object height (m)
    support_z: float = 0.0             # ground: floor-plane height (m)
    ref: str = 'centroid'              # ground: 'centroid' or 'bottom' pixel


# Cube is magenta (not red, which causes artefacts; not yellow, which the
# default floor pattern uses). The "cube" is a thin upright column
# (0.03 x 0.03 x 0.20 m); monocular depth uses its 0.20 m HEIGHT, which
# projects to a far larger, more stable pixel extent than its 0.03 m width.
_CUBE: ColorTarget = ColorTarget(
    'cube', 80, (0.9, 0.1, 0.9), [_hsv_range(140, 170)],
    localize='mono', real_size_m=0.20)

# Plate and door, also localised purely from perception:
#   plate — a flat 0.30 m square lying on the floor: its pixel height is ~0,
#           so monocular depth is useless; instead the centroid ray is
#           intersected with the floor plane (z = 0).
#   door  — a vertical 0.50 m-tall panel on the wall: monocular depth from
#           its HEIGHT, like the cube. Height is angle-invariant horizontally
#           and the panel is fully in frame when seen from across the room.
_LANDMARKS: tuple[ColorTarget, ...] = (
    ColorTarget('plate', 200, (0.1, 1.0, 0.1), [_hsv_range(40, 80)],
                localize='ground', support_z=0.0, ref='centroid'),
    ColorTarget('door', 300, (0.1, 0.3, 1.0), [_hsv_range(100, 130)],
                localize='mono', real_size_m=0.50),
)

# Full set, used only for publisher/marker bookkeeping.
_TARGETS: tuple[ColorTarget, ...] = (_CUBE, *_LANDMARKS)

_CAMERA_FRAME = 'camera_optical_link'
_BASE_FRAME = 'base_link'


class ColorDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('color_detector_node')

        self.declare_parameter('image_topic',  '/camera/image_color')
        self.declare_parameter('robot_alias',  '/RoboMasterEP/BaseLinkFrame')
        self.declare_parameter('camera_alias', '/RoboMasterEP/Camera')
        self.declare_parameter('map_frame',    'map')
        image_topic = str(self.get_parameter('image_topic').value)
        robot_alias = str(self.get_parameter('robot_alias').value)
        camera_alias = str(self.get_parameter('camera_alias').value)
        self._map_frame = str(self.get_parameter('map_frame').value)

        # ---- TF ------------------------------------------------------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- sim handles --------------------------------------------
        # Keep the client as a member: anonymous clients can be
        # garbage-collected, dropping the ZMQ connection.
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')

        try:
            self._robot_handle: int | None = self.sim.getObject(robot_alias)
        except Exception:
            self.get_logger().warn(
                f"could not resolve robot '{robot_alias}'; "
                f"target localisation will fail")
            self._robot_handle = None

        # Diagnostic only: ground-truth cube pose, used solely to log the
        # perception error (never fed to navigation or the gripper).
        try:
            self._cube_truth_handle: int | None = self.sim.getObject(
                '/TargetCube')
        except Exception:
            self._cube_truth_handle = None

        # All three landmarks are localised from perception; no per-target
        # sim object handles are needed.
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
        # Live cube bearing for visual servoing (base_link, every frame).
        self._cube_live_pub = self.create_publisher(
            PoseStamped, '/targets/cube_live', 10)
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
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge failed: {e}',
                                   throttle_duration_sec=2.0)
            return
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        img_h, img_w = hsv.shape[:2]

        # Cube: perception-only, tracked on every frame for visual servoing.
        cube_det = self._detect(hsv, _CUBE)
        if cube_det is not None:
            self._publish_cube(cube_det, img_w, img_h)

        # Plate and door: perception-only, continuously refined while visible.
        for t in _LANDMARKS:
            det = self._detect(hsv, t)
            if det is not None:
                self._publish_landmark(t, det, img_w, img_h)

    # ===== detection / publishing =======================================

    def _detect(self, hsv: np.ndarray, target: ColorTarget
                ) -> tuple[float, float, int, int] | None:
        """Largest connected component matching the HSV windows.
        Returns (centroid_x_px, centroid_y_px, bbox_w_px, bbox_h_px), or
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
        w_px = int(stats[idx, cv2.CC_STAT_WIDTH])
        h_px = int(stats[idx, cv2.CC_STAT_HEIGHT])
        return cx, cy, w_px, h_px

    # ===== perception localisation (no sim ground truth) ================

    def _mono_point_camera(self, cx: float, cy: float, size_px: int,
                           real_size_m: float, img_w: int, img_h: int
                           ) -> tuple[float, float, float] | None:
        """Object centre in ``camera_optical_link`` via monocular depth.

        Pinhole back-projection of the centroid with depth from the known
        object HEIGHT: ``Z = f_pix * real_size / size_px`` (size_px is the
        blob's vertical pixel extent). Optical convention: +x right, +y down,
        +z forward.

        Intrinsics are derived from the *actual published image* size (not
        the sim sensor resolution, which may differ from what robomaster_ros
        publishes) and the camera FOV. The principal point is the image
        centre."""
        if self._cam_fov is None or size_px <= 0 or real_size_m <= 0.0:
            return None
        # CoppeliaSim's perspective_angle spans the longer image axis.
        f_pix = max(img_w, img_h) / (2.0 * math.tan(self._cam_fov / 2.0))
        z = (real_size_m * f_pix) / size_px
        x = (cx - img_w / 2.0) / f_pix * z
        y = (cy - img_h / 2.0) / f_pix * z
        return float(x), float(y), float(z)

    def _ground_point(self, px: float, py: float, z_plane: float,
                      img_w: int, img_h: int
                      ) -> tuple[float, float] | None:
        """Map (x, y) where the pixel ray meets the floor plane ``z=z_plane``.

        For objects resting on the floor this needs no size estimate: the
        camera ray through pixel ``(px, py)`` is transformed to the map frame
        via TF and intersected with the horizontal support plane."""
        if self._cam_fov is None:
            return None
        f_pix = max(img_w, img_h) / (2.0 * math.tan(self._cam_fov / 2.0))
        dx = (px - img_w / 2.0) / f_pix
        dy = (py - img_h / 2.0) / f_pix
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, _CAMERA_FRAME, rclpy.time.Time())
        except Exception:
            return None
        origin = do_transform_point(self._cam_ps(0.0, 0.0, 0.0), tf).point
        along = do_transform_point(self._cam_ps(dx, dy, 1.0), tf).point
        rz = along.z - origin.z
        if abs(rz) < 1e-6:
            return None
        t = (z_plane - origin.z) / rz
        if t <= 0.0:
            return None
        return (origin.x + t * (along.x - origin.x),
                origin.y + t * (along.y - origin.y))

    @staticmethod
    def _vertically_clipped(cy: float, h_px: int, img_h: int) -> bool:
        """True if the blob touches the top/bottom image edge, so its pixel
        height (hence monocular depth) is unreliable."""
        margin = 2.0
        top = cy - h_px / 2.0
        bottom = cy + h_px / 2.0
        return top <= margin or bottom >= img_h - margin

    @staticmethod
    def _cam_ps(x: float, y: float, z: float) -> PointStamped:
        ps = PointStamped()
        ps.header.frame_id = _CAMERA_FRAME
        ps.header.stamp = rclpy.time.Time().to_msg()  # latest available TF
        ps.point.x, ps.point.y, ps.point.z = x, y, z
        return ps

    def _publish_cube(self, detection: tuple[float, float, int, int],
                      img_w: int, img_h: int) -> None:
        """Localise the cube and publish the live base_link bearing (every
        frame, for visual servoing) plus a continuously-refined map pose.

        The map pose is re-published as the robot approaches: a far first
        sighting (a few pixels tall) gives a poor monocular range, which
        sharpens as the cube grows in frame. Latching the first estimate
        would freeze that early error, so it is refreshed instead."""
        cx, cy, _w_px, h_px = detection
        cam = self._mono_point_camera(cx, cy, h_px, _CUBE.real_size_m,
                                      img_w, img_h)
        if cam is None:
            return
        ps = self._cam_ps(*cam)

        # --- live bearing in base_link (for visual servoing) ------------
        bl = self._transform_point(ps, _BASE_FRAME)
        if bl is None:
            self.get_logger().warn(
                f"cube seen but TF '{_CAMERA_FRAME}'→'{_BASE_FRAME}' "
                f"unavailable; is robot_state_publisher running?",
                throttle_duration_sec=5.0)
            return
        self._cube_live_pub.publish(self._pose_from_point(bl, _BASE_FRAME))

        # --- refined map pose (for the GO_TO_KEY Nav2 standoff) ---------
        # Skip while the column clips the frame edge: its height (hence range)
        # is then unreliable. The live bearing above is still valid.
        if not self._vertically_clipped(cy, h_px, img_h):
            mp = self._transform_point(ps, self._map_frame)
            if mp is not None:
                self._publish_map('cube', mp.point.x, mp.point.y, mp.point.z)

        self._log_cube_debug(detection, cam, bl, img_w, img_h)

    def _publish_landmark(self, t: ColorTarget,
                          detection: tuple[float, float, int, int],
                          img_w: int, img_h: int) -> None:
        """Localise a plate/door landmark from perception and publish a
        continuously-refined map pose. ``mono`` targets use monocular height
        depth; ``ground`` targets intersect the pixel ray with the floor."""
        cx, cy, _w_px, h_px = detection
        if t.localize == 'ground':
            py = cy + h_px / 2.0 if t.ref == 'bottom' else cy
            gp = self._ground_point(cx, py, t.support_z, img_w, img_h)
            if gp is None:
                return
            self._publish_map(t.name, gp[0], gp[1], t.support_z)
            return
        # mono: skip clipped detections (unreliable height → range).
        if self._vertically_clipped(cy, h_px, img_h):
            return
        cam = self._mono_point_camera(cx, cy, h_px, t.real_size_m,
                                      img_w, img_h)
        if cam is None:
            return
        mp = self._transform_point(self._cam_ps(*cam), self._map_frame)
        if mp is not None:
            self._publish_map(t.name, mp.point.x, mp.point.y, mp.point.z)

    def _publish_map(self, name: str, x: float, y: float, z: float) -> None:
        """Publish a landmark's refined map pose and update its marker."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self._target_pubs[name].publish(msg)
        first = name not in self._published
        self._published.add(name)
        prev = self._marker_poses.get(name)
        if prev is None or math.hypot(x - prev[0], y - prev[1]) > 0.05:
            self._marker_poses[name] = (x, y, z)
            self._publish_markers()
        if first:
            self.get_logger().info(
                f'[{name}] perception map ({x:.2f}, {y:.2f})')

    def _log_cube_debug(self, detection, cam, bl,
                        img_w: int, img_h: int) -> None:
        """Throttled perception-vs-ground-truth check, in base_link so the
        map↔world frame offset doesn't pollute the error. Diagnostic only,
        never used for control."""
        if self._cube_truth_handle is None or self._robot_handle is None:
            return
        try:
            tw = self.sim.getObjectPosition(self._cube_truth_handle, -1)
            r_pos = self.sim.getObjectPosition(self._robot_handle, -1)
            r_yaw = _yaw_from_quat(
                self.sim.getObjectQuaternion(self._robot_handle, -1))
        except Exception:
            return
        # Ground-truth cube expressed in base_link (x fwd, y left).
        dx, dy = tw[0] - r_pos[0], tw[1] - r_pos[1]
        c, s = math.cos(-r_yaw), math.sin(-r_yaw)
        tbx, tby = c * dx - s * dy, s * dx + c * dy
        err = math.hypot(bl.point.x - tbx, bl.point.y - tby)
        _, _, _w_px, h_px = detection
        self.get_logger().info(
            f'[cube dbg] img={img_w}x{img_h} simres={self._cam_res} '
            f'fov={math.degrees(self._cam_fov or 0):.1f} h={h_px}px '
            f'Z={cam[2]:.2f} '
            f'base=({bl.point.x:.2f},{bl.point.y:.2f}) '
            f'truth_base=({tbx:.2f},{tby:.2f}) err={err:.2f}m',
            throttle_duration_sec=2.0)

    def _transform_point(self, ps: PointStamped, frame: str
                         ) -> PointStamped | None:
        """Transform a PointStamped into ``frame`` via TF; None if the
        transform is not (yet) available."""
        try:
            tf = self._tf_buffer.lookup_transform(
                frame, ps.header.frame_id, rclpy.time.Time())
        except Exception:
            return None
        return do_transform_point(ps, tf)

    @staticmethod
    def _pose_from_point(p: PointStamped, frame: str) -> PoseStamped:
        msg = PoseStamped()
        msg.header = p.header
        msg.header.frame_id = frame
        msg.pose.position.x = p.point.x
        msg.pose.position.y = p.point.y
        msg.pose.position.z = p.point.z
        msg.pose.orientation.w = 1.0
        return msg

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
        m.header.frame_id = self._map_frame
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
        m.header.frame_id = self._map_frame
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
