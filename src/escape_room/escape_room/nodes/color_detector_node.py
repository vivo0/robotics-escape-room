#!/usr/bin/env python3
"""
Color landmark detector for the discovery phase.

Subscribes to the robot's RGB camera and looks for three coloured
landmarks placed in the scene by ``build_scene.py``:

    - magenta cuboid → the "key"           (sim alias TargetCube)
    - green plate    → the pressure plate  (sim alias PressurePlate)
    - blue door      → the exit door       (sim alias Door)

Why magenta for the key: red trips on the Velodyne's red scan-beam
overlay; yellow trips on the CoppeliaSim default floor pattern.
Magenta sits far from every other colour in the scene (red beams,
yellow tiles, green plate, blue door) and gives a clean HSV signal.

Detection is colour-based (HSV thresholding + connected-component
analysis on the camera image). The first time a colour clears the
size threshold:

* the node publishes the *true* sim pose of the object on
  ``/targets/<name>`` — that's the ground-truth used for navigation;
* the node also publishes an RViz marker at the **camera-estimated**
  position, computed by projecting the blob centroid through the
  camera (pinhole) and using the known target size to recover depth.
  This visualises *where the robot thinks it saw the target* — a
  false positive (e.g. on the Velodyne's red laser beams) shows up as
  a marker visibly off the real object.

Subscribes:
    /camera/image_color  (sensor_msgs/Image)

Publishes (latched, TRANSIENT_LOCAL):
    /targets/cube     (geometry_msgs/PoseStamped)        sim-truth pose
    /targets/plate    (geometry_msgs/PoseStamped)        sim-truth pose
    /targets/door     (geometry_msgs/PoseStamped)        sim-truth pose
    /targets/markers  (visualization_msgs/MarkerArray)   camera estimates
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from cv_bridge import CvBridge

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def _hsv_range(h_lo: int, h_hi: int,
               s_lo: int = 60, s_hi: int = 255,
               v_lo: int = 60, v_hi: int = 255
               ) -> tuple[np.ndarray, np.ndarray]:
    """Convenience builder for an OpenCV HSV (lo, hi) pair.
    OpenCV's H channel is in [0, 179], so values are halved compared
    to the conventional 0-359° hue space."""
    return (np.array([h_lo, s_lo, v_lo], dtype=np.uint8),
            np.array([h_hi, s_hi, v_hi], dtype=np.uint8))


@dataclass
class ColorTarget:
    name: str                                  # 'cube' / 'plate' / 'door'
    sim_alias: str                             # alias in the CoppeliaSim scene
    min_pixels: int                            # smallest blob accepted
    rgb: tuple[float, float, float]            # marker colour in RViz (0..1)
    real_size_m: float                         # real-world height/width used
                                               # for monocular depth recovery
    hsv_ranges: list = field(default_factory=list)


# Magenta lives around H≈150 in OpenCV's 0..179 hue space; the key gets
# a single window centred there. Green and blue fit comfortably in one
# window each. ``real_size_m`` is the physical dimension matched
# against the blob's largest axis in pixels for depth estimation.
_TARGETS: tuple[ColorTarget, ...] = (
    ColorTarget(
        name='cube',
        sim_alias='/TargetCube',
        min_pixels=80,
        rgb=(0.9, 0.1, 0.9),
        real_size_m=0.12,   # cube is 0.04 x 0.04 x 0.12 — height dominates
        hsv_ranges=[_hsv_range(140, 170)],
    ),
    ColorTarget(
        name='plate',
        sim_alias='/PressurePlate',
        min_pixels=200,
        rgb=(0.1, 1.0, 0.1),
        real_size_m=0.30,   # 30 cm square plate
        hsv_ranges=[_hsv_range(40, 80)],
    ),
    ColorTarget(
        name='door',
        sim_alias='/Door',
        min_pixels=300,
        rgb=(0.1, 0.3, 1.0),
        real_size_m=0.80,   # door opening width
        hsv_ranges=[_hsv_range(100, 130)],
    ),
)


class ColorDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('color_detector_node')

        # ---- parameters ---------------------------------------------
        self.declare_parameter('image_topic', '/camera/image_color')
        self.declare_parameter('target_frame', 'world')
        self.declare_parameter('camera_alias', '/RoboMasterEP/Camera')
        # When set, the BGR frame and HSV mask at the moment of trigger
        # are written here as ``<dump_dir>/det_<name>.png`` so we can
        # eyeball what the camera actually saw. Empty string disables.
        self.declare_parameter('dump_dir', '/tmp')
        image_topic = str(self.get_parameter('image_topic').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        camera_alias = str(self.get_parameter('camera_alias').value)
        self._dump_dir = str(self.get_parameter('dump_dir').value)

        # ---- sim connection -----------------------------------------
        self.get_logger().info('Connecting to CoppeliaSim ZMQ remote API...')
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')

        # Missing aliases are non-fatal: we just won't publish that target.
        self._handles: dict[str, int | None] = {}
        for t in _TARGETS:
            try:
                self._handles[t.name] = self.sim.getObject(t.sim_alias)
            except Exception as e:
                self.get_logger().warn(
                    f"Could not resolve '{t.sim_alias}': {e}. "
                    f"'{t.name}' detections will be ignored."
                )
                self._handles[t.name] = None

        # Camera handle + intrinsics. We first try the configured
        # alias; if it doesn't resolve we fall back to enumerating
        # every vision sensor in the scene and picking the first one.
        # That way the user doesn't need to guess the exact path
        # inside the robot model.
        self._camera_handle: int | None = None
        self._cam_res: tuple[int, int] | None = None
        self._cam_fov: float | None = None
        self._camera_handle, resolved_alias = self._resolve_camera(
            camera_alias)
        if self._camera_handle is not None:
            try:
                self._cam_res = (
                    int(self.sim.getObjectInt32Param(
                        self._camera_handle,
                        self.sim.visionintparam_resolution_x)),
                    int(self.sim.getObjectInt32Param(
                        self._camera_handle,
                        self.sim.visionintparam_resolution_y)),
                )
                self._cam_fov = float(self.sim.getObjectFloatParam(
                    self._camera_handle,
                    self.sim.visionfloatparam_perspective_angle))
                self.get_logger().info(
                    f"camera '{resolved_alias}' resolved: "
                    f"res={self._cam_res}, "
                    f"fov={math.degrees(self._cam_fov):.1f}°"
                )
            except Exception as e:
                self.get_logger().warn(
                    f"Camera '{resolved_alias}' resolved but couldn't "
                    f"read intrinsics: {e}. Markers will use sim-truth.")
                self._camera_handle = None
        else:
            self.get_logger().warn(
                'No vision sensor found in the scene. Markers will '
                'fall back to sim-truth target positions.')

        # ---- ROS pub/sub --------------------------------------------
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._target_pubs = {
            t.name: self.create_publisher(
                PoseStamped, f'/targets/{t.name}', latched_qos)
            for t in _TARGETS
        }
        self._marker_pub = self.create_publisher(
            MarkerArray, '/targets/markers', latched_qos)
        self._published: set[str] = set()
        self._marker_poses: dict[str, tuple[float, float, float]] = {}

        self._bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, image_topic, self._on_image, 10)

        self.get_logger().info(
            f'color_detector_node ready. listening on {image_topic}; '
            f'targets: {[t.name for t in _TARGETS]}'
        )

    def _resolve_camera(self, hint_alias: str
                        ) -> tuple[int | None, str | None]:
        """Try the configured alias first; on failure enumerate every
        vision sensor in the scene and return the first match."""
        try:
            handle = self.sim.getObject(hint_alias)
            return handle, hint_alias
        except Exception:
            pass

        try:
            candidates = self.sim.getObjectsInTree(
                self.sim.handle_scene, self.sim.object_visionsensor)
        except Exception as e:
            self.get_logger().warn(f'getObjectsInTree failed: {e}')
            return None, None

        if not candidates:
            return None, None

        for h in candidates:
            try:
                alias = self.sim.getObjectAlias(int(h), 1)  # full path
                self.get_logger().info(f'discovered vision sensor: {alias}')
            except Exception:
                continue
        # First one wins. Robomaster only carries one camera in this scene.
        first = int(candidates[0])
        try:
            alias = self.sim.getObjectAlias(first, 1)
        except Exception:
            alias = '<unknown>'
        return first, alias

    # ===== callbacks =====================================================

    def _on_image(self, msg: Image) -> None:
        if len(self._published) == len(_TARGETS):
            return  # all targets already discovered

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
                self._dump_detection(t, bgr, hsv, det)
                self._publish_target(t, det)

    # ===== detection / pose =============================================

    def _detect(self, hsv: np.ndarray, target: ColorTarget
                ) -> tuple[float, float, int] | None:
        """Find the largest connected component matching any of the HSV
        windows. Return ``(cx, cy, pixel_size)`` for it — pixel
        coordinates of the centroid and the larger of the bounding-box
        width/height — or ``None`` if no blob clears
        ``target.min_pixels``."""
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in target.hsv_ranges:
            mask |= cv2.inRange(hsv, lo, hi)
        n, _, stats, centroids = cv2.connectedComponentsWithStats(
            mask, connectivity=8)
        if n < 2:  # only the background label
            return None
        # stats[0] is the background; pick the largest real component.
        areas = stats[1:, cv2.CC_STAT_AREA]
        idx = int(areas.argmax()) + 1
        if int(stats[idx, cv2.CC_STAT_AREA]) < target.min_pixels:
            return None
        cx = float(centroids[idx, 0])
        cy = float(centroids[idx, 1])
        bbox_w = int(stats[idx, cv2.CC_STAT_WIDTH])
        bbox_h = int(stats[idx, cv2.CC_STAT_HEIGHT])
        return cx, cy, max(bbox_w, bbox_h)

    def _publish_target(self, target: ColorTarget,
                        detection: tuple[float, float, int]) -> None:
        """``detection`` is ``(cx, cy, pixel_size)`` from ``_detect``.
        We still publish the sim-truth pose on ``/targets/<name>`` (so
        the navigation stack stays exact); the marker, however, is
        drawn at the camera-estimated position."""
        handle = self._handles.get(target.name)
        if handle is None:
            return
        try:
            pos = self.sim.getObjectPosition(handle, -1)
        except Exception as e:
            self.get_logger().warn(
                f"sim.getObjectPosition({target.sim_alias}) failed: {e}")
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.w = 1.0
        self._target_pubs[target.name].publish(msg)
        self._published.add(target.name)

        # Marker = camera estimate when we can compute it; otherwise we
        # fall back to the sim-truth pose so the user still sees something.
        cx, cy, pixel_size = detection
        cam = self._estimate_world_xyz(target, cx, cy, pixel_size)
        marker_xyz = cam if cam is not None else (
            float(pos[0]), float(pos[1]), float(pos[2]))
        self._marker_poses[target.name] = marker_xyz
        self._publish_markers()

        if cam is not None:
            err = math.hypot(cam[0] - pos[0], cam[1] - pos[1])
            self.get_logger().info(
                f'[{target.name}] sim @ ({pos[0]:.2f}, {pos[1]:.2f}, '
                f'{pos[2]:.2f}); camera estimate @ ({cam[0]:.2f}, '
                f'{cam[1]:.2f}, {cam[2]:.2f}); xy_err={err:.2f} m'
            )
        else:
            self.get_logger().info(
                f'[{target.name}] sim @ ({pos[0]:.2f}, {pos[1]:.2f}, '
                f'{pos[2]:.2f}); camera estimate unavailable'
            )
        if len(self._published) == len(_TARGETS):
            self.get_logger().info('All landmarks discovered.')

    def _dump_detection(self, target: ColorTarget, bgr: np.ndarray,
                        hsv: np.ndarray,
                        detection: tuple[float, float, int]) -> None:
        """Write the BGR frame (with the detected blob outlined and the
        centroid marked) plus the binary HSV mask to ``dump_dir`` so we
        can eyeball what really triggered the detection."""
        if not self._dump_dir:
            return
        cx, cy, _ = detection
        # Build the same mask _detect built (cheap; the alternative
        # would be plumbing the mask all the way through).
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in target.hsv_ranges:
            mask |= cv2.inRange(hsv, lo, hi)

        annotated = bgr.copy()
        cv2.drawMarker(annotated, (int(cx), int(cy)), (0, 255, 255),
                       markerType=cv2.MARKER_CROSS, markerSize=20,
                       thickness=2)
        cv2.putText(annotated, target.name, (int(cx) + 8, int(cy) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,
                    cv2.LINE_AA)

        path_frame = f'{self._dump_dir}/det_{target.name}.png'
        path_mask = f'{self._dump_dir}/det_{target.name}_mask.png'
        try:
            cv2.imwrite(path_frame, annotated)
            cv2.imwrite(path_mask, mask)
            self.get_logger().info(
                f'[{target.name}] frame dumped → {path_frame} '
                f'(mask: {path_mask})')
        except Exception as e:
            self.get_logger().warn(f'frame dump failed: {e}')

    def _estimate_world_xyz(self, target: ColorTarget,
                            cx: float, cy: float, pixel_size: int
                            ) -> tuple[float, float, float] | None:
        """Project the blob centroid through the pinhole camera and
        recover depth from the known target size: a pixel_size-sized
        blob of a real_size_m-tall object sits at distance
        ``f * real_size / pixel_size`` along the camera ray."""
        if (self._camera_handle is None or self._cam_res is None
                or self._cam_fov is None or pixel_size <= 0):
            return None
        try:
            mat = self.sim.getObjectMatrix(self._camera_handle, -1)
        except Exception:
            return None

        W, H = self._cam_res
        # CoppeliaSim's perspective_angle is the FOV across the larger
        # image dimension, so the equivalent focal length in pixels is:
        f_pix = max(W, H) / (2.0 * math.tan(self._cam_fov / 2.0))
        if f_pix <= 0:
            return None

        # Standard pinhole: +x right, +y down, +z forward in camera
        # frame. CoppeliaSim's vision sensor convention matches this
        # for our purposes (image rows top-down, optical axis +z).
        nx = (cx - W / 2.0) / f_pix
        ny = (cy - H / 2.0) / f_pix
        ray_cam = np.array([nx, ny, 1.0], dtype=np.float64)
        ray_cam /= np.linalg.norm(ray_cam)

        # Camera→world from the 3x4 row-major matrix returned by sim.
        M = np.array(mat, dtype=np.float64).reshape(3, 4)
        ray_world = M[:, :3] @ ray_cam
        cam_origin = M[:, 3]
        depth = (target.real_size_m * f_pix) / float(pixel_size)
        point = cam_origin + ray_world * depth
        return float(point[0]), float(point[1]), float(point[2])

    def _publish_markers(self) -> None:
        """Publish a MarkerArray with one sphere + one text label per
        already-detected target. Re-published every detection so a
        late-joining RViz also sees the latched state."""
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        for t in _TARGETS:
            xyz = self._marker_poses.get(t.name)
            if xyz is None:
                continue
            arr.markers.append(self._build_sphere(t, xyz, stamp))
            arr.markers.append(self._build_label(t, xyz, stamp))
        self._marker_pub.publish(arr)

    def _build_sphere(self, target: ColorTarget,
                      xyz: tuple[float, float, float],
                      stamp) -> Marker:
        m = Marker()
        m.header.frame_id = self.target_frame
        m.header.stamp = stamp
        m.ns = 'targets'
        m.id = hash(target.name) & 0xFFFF
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.18
        m.color.r, m.color.g, m.color.b = target.rgb
        m.color.a = 0.85
        return m

    def _build_label(self, target: ColorTarget,
                     xyz: tuple[float, float, float],
                     stamp) -> Marker:
        m = Marker()
        m.header.frame_id = self.target_frame
        m.header.stamp = stamp
        m.ns = 'target_labels'
        m.id = hash(target.name) & 0xFFFF
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = xyz[0]
        m.pose.position.y = xyz[1]
        m.pose.position.z = xyz[2] + 0.30  # float above the sphere
        m.pose.orientation.w = 1.0
        m.scale.z = 0.18  # text height
        m.color.r = m.color.g = m.color.b = 1.0
        m.color.a = 1.0
        m.text = target.name
        return m


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ColorDetectorNode()
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
