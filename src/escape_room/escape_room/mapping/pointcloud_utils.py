"""
Lightweight PointCloud2 parsing utilities. No ROS imports here so the
module is testable standalone; the caller passes the message dict-like
object produced by rclpy.
"""
from __future__ import annotations

import numpy as np


_DTYPE_BY_PCL_TYPE = {
    1: np.int8,    2: np.uint8,
    3: np.int16,   4: np.uint16,
    5: np.int32,   6: np.uint32,
    7: np.float32, 8: np.float64,
}


def pointcloud2_to_xyz(msg) -> np.ndarray:
    """Decode a sensor_msgs/PointCloud2 into an (N, 3) float32 array of XYZ.

    Reads the field offsets from msg.fields rather than assuming a layout,
    so it works for the Velodyne (xyz + intensity, point_step=16) as well
    as other producers.
    """
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ('x', 'y', 'z')):
        raise ValueError("PointCloud2 missing x/y/z fields")

    point_step = msg.point_step
    n = msg.width * msg.height
    if n == 0:
        return np.zeros((0, 3), dtype=np.float32)

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    # one row per point
    raw = raw.reshape(n, point_step)

    out = np.empty((n, 3), dtype=np.float32)
    for axis_idx, axis in enumerate(('x', 'y', 'z')):
        f = fields[axis]
        dtype = _DTYPE_BY_PCL_TYPE.get(f.datatype)
        if dtype is None:
            raise ValueError(f"Unsupported datatype {f.datatype} on field {axis}")
        item_size = np.dtype(dtype).itemsize
        col = raw[:, f.offset:f.offset + item_size].reshape(-1).view(dtype)
        out[:, axis_idx] = col.astype(np.float32, copy=False)
    return out
