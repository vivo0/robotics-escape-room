#!/usr/bin/env python3
"""Color landmark detector for the discovery phase.

Looks for three coloured landmarks in the robot's camera image:

    cube  (magenta cuboid) → sim alias /TargetCube
    plate (green square)   → sim alias /PressurePlate
    door  (blue rectangle) → sim alias /Door_0

The largest HSV-matching connected component is the detection.

The **cube** is localised purely from perception (no sim ground truth):
the blob centroid is back-projected through the pinhole model, depth is
recovered monocularly from the known cube size, and the resulting 3D point
in ``camera_optical_link`` is transformed via the live TF tree. It is
published continuously in ``base_link`` on ``/targets/cube_live`` (for the
explorer's visual-servoing pickup) and once in ``map`` on ``/targets/cube``
(so Nav2 can drive to the standoff). The cube handle is never queried.

The **plate** and **door** still use sim truth (they only seed coarse Nav2
goals, not a precise grasp): world position from CoppeliaSim → base_link via
the robot's sim pose → map via TF → latched PoseStamped.

If the required TF is not yet available when a target is first detected,
the detection is stored in _pending and retried on every subsequent image
frame until the transform succeeds.

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
    sim_alias: str
    min_pixels: int
    rgb: tuple[float, float, float]   # marker colour in RViz
    real_size_m: float                # used for monocular depth recovery
    hsv_ranges: list = field(default_factory=list)


# Cube is magenta (not red, which causes artefacts; not yellow,
# which the default floor pattern uses). It is localised by perception
# only, so its sim_alias is unused. The "cube" is a thin upright column
# (0.03 x 0.03 x 0.20 m); monocular depth uses its 0.20 m HEIGHT, which
# projects to a far larger, more stable pixel extent than its 0.03 m width.
_CUBE: ColorTarget = ColorTarget(
    'cube', '/TargetCube', 80, (0.9, 0.1, 0.9), 0.20, [_hsv_range(140, 170)])

# Plate and door still use sim truth (coarse Nav2 seeds only).
_SIM_TARGETS: tuple[ColorTarget, ...] = (
    ColorTarget('plate', '/PressurePlate', 200, (0.1, 1.0, 0.1), 0.30,
                [_hsv_range(40, 80)]),
    ColorTarget('door',  '/Door_0',        300, (0.1, 0.3, 1.0), 0.80,
                [_hsv_range(100, 130)]),
)

# Full set, used only for publisher/marker bookkeeping.
_TARGETS: tuple[ColorTarget, ...] = (_CUBE, *_SIM_TARGETS)

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

        # Only plate/door need a sim handle; the cube is perception-only.
        self._handles: dict[str, int | None] = {}
        for t in _SIM_TARGETS:
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
        # Live cube bearing for visual servoing (base_link, every frame).
        self._cube_live_pub = self.create_publisher(
            PoseStamped, '/targets/cube_live', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/targets/markers', latched)
        self._published: set[str] = set()
        self._marker_poses: dict[str, tuple[float, float, float]] = {}
        self._pending: dict[str, tuple[ColorTarget, tuple]] = {}

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

        # Cube: perception-only, tracked on every frame (no sim truth).
        cube_det = self._detect(hsv, _CUBE)
        if cube_det is not None:
            self._publish_cube(cube_det, img_w, img_h)

        if len(self._published) == len(_TARGETS) and not self._pending:
            return

        # Retry pending sim-truth detections (TF may now be available).
        for name in list(self._pending):
            t, det = self._pending[name]
            if self._try_publish(t, det):
                del self._pending[name]

        # Detect and publish new sim-truth targets (plate, door).
        for t in _SIM_TARGETS:
            if t.name in self._published or t.name in self._pending:
                continue
            det = self._detect(hsv, t)
            if det is not None:
                if not self._try_publish(t, det):
                    self._pending[t.name] = (t, det)

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

    # ===== cube perception (no sim ground truth) ========================

    def _cube_point_camera(self, cx: float, cy: float, size_px: int,
                           img_w: int, img_h: int
                           ) -> tuple[float, float, float] | None:
        """Cube centre in the ``camera_optical_link`` frame from the blob.

        Pinhole back-projection of the centroid with monocular depth from
        the known cube height: ``Z = f_pix * real_size / size_px`` (size_px
        is the blob's vertical pixel extent). Optical convention: +x right,
        +y down, +z forward.

        Intrinsics are derived from the *actual published image* size (not
        the sim sensor resolution, which may differ from what robomaster_ros
        publishes) and the camera FOV. The principal point is the image
        centre."""
        if self._cam_fov is None or size_px <= 0:
            return None
        # CoppeliaSim's perspective_angle spans the longer image axis.
        f_pix = max(img_w, img_h) / (2.0 * math.tan(self._cam_fov / 2.0))
        z = (_CUBE.real_size_m * f_pix) / size_px
        x = (cx - img_w / 2.0) / f_pix * z
        y = (cy - img_h / 2.0) / f_pix * z
        return float(x), float(y), float(z)

    def _publish_cube(self, detection: tuple[float, float, int, int],
                      img_w: int, img_h: int) -> None:
        """Localise the cube from the camera and publish the live bearing
        (base_link, every frame) plus a continuously-refined map pose.

        The map pose is re-published as the robot approaches: a far first
        sighting (a few pixels tall) gives a poor monocular range, which
        sharpens as the cube grows in frame. Latching the first estimate
        would freeze that early error, so it is refreshed instead."""
        cx, cy, _w_px, h_px = detection
        cam = self._cube_point_camera(cx, cy, h_px, img_w, img_h)
        if cam is None:
            return
        ps = PointStamped()
        ps.header.frame_id = _CAMERA_FRAME
        ps.header.stamp = rclpy.time.Time().to_msg()  # latest available TF
        ps.point.x, ps.point.y, ps.point.z = cam

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
        mp = self._transform_point(ps, self._map_frame)
        if mp is None:
            return
        self._target_pubs['cube'].publish(
            self._pose_from_point(mp, self._map_frame))
        self._published.add('cube')
        prev = self._marker_poses.get('cube')
        cube_xyz = (mp.point.x, mp.point.y, mp.point.z)
        if prev is None or math.hypot(
                cube_xyz[0] - prev[0], cube_xyz[1] - prev[1]) > 0.05:
            self._marker_poses['cube'] = cube_xyz
            self._publish_markers()

        self._log_cube_debug(detection, cam, bl, img_w, img_h)

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

    def _try_publish(self, target: ColorTarget,
                     detection: tuple[float, float, int, int]) -> bool:
        """Localise target in map frame and publish. Returns False if TF
        is not yet available (caller should retry)."""
        if self._robot_handle is None:
            return False
        handle = self._handles.get(target.name)
        if handle is None:
            return True  # no sim object → skip permanently

        # --- sim truth → base_link frame --------------------------------
        pos_w = self.sim.getObjectPosition(handle, -1)
        r_pos = self.sim.getObjectPosition(self._robot_handle, -1)
        r_q = self.sim.getObjectQuaternion(self._robot_handle, -1)
        r_yaw = _yaw_from_quat(r_q)
        dx, dy = pos_w[0] - r_pos[0], pos_w[1] - r_pos[1]
        c, s = math.cos(-r_yaw), math.sin(-r_yaw)
        bl_x, bl_y = c * dx - s * dy, s * dx + c * dy

        # --- base_link → map via TF -------------------------------------
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, 'base_link', rclpy.time.Time())
            qz, qw_r = tf.transform.rotation.z, tf.transform.rotation.w
            yaw = 2.0 * math.atan2(qz, qw_r)
            c2, s2 = math.cos(yaw), math.sin(yaw)
            map_x = c2 * bl_x - s2 * bl_y + tf.transform.translation.x
            map_y = s2 * bl_x + c2 * bl_y + tf.transform.translation.y
        except Exception:
            return False  # TF not ready; caller stores in _pending

        # --- publish pose -----------------------------------------------
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        msg.pose.position.x = map_x
        msg.pose.position.y = map_y
        msg.pose.orientation.w = 1.0
        self._target_pubs[target.name].publish(msg)
        self._published.add(target.name)

        # --- markers + log ----------------------------------------------
        self._marker_poses[target.name] = (map_x, map_y, float(pos_w[2]))
        self._publish_markers()

        cam = self._estimate_world_xyz(target, *detection)
        if cam is not None:
            err = math.hypot(cam[0] - pos_w[0], cam[1] - pos_w[1])
            self.get_logger().info(
                f'[{target.name}] map ({map_x:.2f}, {map_y:.2f}) '
                f'sim ({pos_w[0]:.2f}, {pos_w[1]:.2f}) '
                f'cam-err={err:.2f} m')
        else:
            self.get_logger().info(
                f'[{target.name}] map ({map_x:.2f}, {map_y:.2f}) '
                f'sim ({pos_w[0]:.2f}, {pos_w[1]:.2f})')
        return True

    def _estimate_world_xyz(self, target: ColorTarget,
                            cx: float, cy: float, w_px: int, h_px: int
                            ) -> tuple[float, float, float] | None:
        """Pinhole back-projection of the blob centroid plus monocular
        depth from the known target size:
        ``depth = f_pix * real_size / pixel_size``."""
        size_px = max(w_px, h_px)
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
