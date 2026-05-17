"""TF pose lookup helper."""

import math

from rclpy.duration import Duration
from rclpy.time import Time


def lookup_pose(
    tf_buffer, map_frame: str, base_frame: str
) -> tuple[float, float, float] | None:
    """Return (x, y, yaw) of base_frame in map_frame, or None on TF failure."""
    try:
        tf = tf_buffer.lookup_transform(
            map_frame, base_frame, Time(), timeout=Duration(seconds=0.1)
        )
    except Exception:
        return None
    x = tf.transform.translation.x
    y = tf.transform.translation.y
    q = tf.transform.rotation
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2)
    )
    return float(x), float(y), float(yaw)
