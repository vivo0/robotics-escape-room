"""Door-exit geometry."""

import math


def door_threshold_xy_yaw(
    door_xy: tuple[float, float],
    door_normal: tuple[float, float],
    inset_m: float,
) -> tuple[float, float, float]:
    """Approach pose just inside the door, facing perpendicular to its
    panel along the outward normal. The robot then drives straight forward
    (exit_drive) to cross the threshold."""
    dx, dy = door_xy
    nx, ny = door_normal
    return dx - inset_m * nx, dy - inset_m * ny, math.atan2(ny, nx)
