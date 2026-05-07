#!/usr/bin/env python3
"""
Color landmark detector for the discovery phase.

Subscribes to the robot's RGB camera and looks for three coloured
landmarks placed in the scene by ``build_scene.py``:

    - red cylinder  → the "key"           (sim alias TargetCube)
    - green plate   → the pressure plate  (sim alias PressurePlate)
    - blue door     → the exit door       (sim alias Door)

Detection is colour-based (HSV thresholding + connected-component
analysis on the camera image). The first time a colour clears the
size threshold the node queries CoppeliaSim for the corresponding
object's exact world pose and publishes it on a latched topic.
Subsequent detections of the same colour are ignored.

This is the same hybrid pattern used by ``mapper_node`` and
``explorer_node``: the *event* (did we see it?) comes from a real
sensor; the resulting *pose* comes from sim, so we don't fight
camera intrinsics or TF lag.

Subscribes:
    /camera/image_color  (sensor_msgs/Image)

Publishes (latched, TRANSIENT_LOCAL):
    /targets/cube   (geometry_msgs/PoseStamped)  red object
    /targets/plate  (geometry_msgs/PoseStamped)  green object
    /targets/door   (geometry_msgs/PoseStamped)  blue object
"""
from __future__ import annotations

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
    hsv_ranges: list = field(default_factory=list)


# Saturated red wraps the H=0/180 boundary in OpenCV's HSV space, so
# red gets two windows. Green and blue fit comfortably in one each.
_TARGETS: tuple[ColorTarget, ...] = (
    ColorTarget(
        name='cube',
        sim_alias='/TargetCube',
        min_pixels=80,
        hsv_ranges=[_hsv_range(0, 10), _hsv_range(170, 180)],
    ),
    ColorTarget(
        name='plate',
        sim_alias='/PressurePlate',
        min_pixels=200,
        hsv_ranges=[_hsv_range(40, 80)],
    ),
    ColorTarget(
        name='door',
        sim_alias='/Door',
        min_pixels=300,
        hsv_ranges=[_hsv_range(100, 130)],
    ),
)


class ColorDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('color_detector_node')

        # ---- parameters ---------------------------------------------
        self.declare_parameter('image_topic', '/camera/image_color')
        self.declare_parameter('target_frame', 'world')
        image_topic = str(self.get_parameter('image_topic').value)
        self.target_frame = str(self.get_parameter('target_frame').value)

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
        self._published: set[str] = set()

        self._bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, image_topic, self._on_image, 10)

        self.get_logger().info(
            f'color_detector_node ready. listening on {image_topic}; '
            f'targets: {[t.name for t in _TARGETS]}'
        )

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
            if self._detect(hsv, t):
                self._publish_target(t)

    # ===== detection / pose =============================================

    def _detect(self, hsv: np.ndarray, target: ColorTarget) -> bool:
        """Return True iff there's a connected component of at least
        `target.min_pixels` matching any of the HSV windows."""
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lo, hi in target.hsv_ranges:
            mask |= cv2.inRange(hsv, lo, hi)
        n, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        if n < 2:  # only the background label is present
            return False
        # stats[0] is the background; the rest are real components.
        return int(stats[1:, cv2.CC_STAT_AREA].max()) >= target.min_pixels

    def _publish_target(self, target: ColorTarget) -> None:
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

        self.get_logger().info(
            f'[{target.name}] detected at '
            f'({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})'
        )
        if len(self._published) == len(_TARGETS):
            self.get_logger().info('All landmarks discovered.')


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
