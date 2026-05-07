"""
Pure-pursuit path follower for a differential drive.

Given a path (list of world (x, y) waypoints) and the robot's current
pose (x, y, yaw in the same frame), pick a "lookahead" point on the
path ahead of the robot and return the (linear, angular) command that
steers toward it. Standard textbook implementation tuned for low
speeds in simulation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class PurePursuitConfig:
    """Tuning for the controller. Defaults intentionally conservative
    so per-tick rotation stays small enough that the deterministic
    mapper paints occlusion correctly."""
    lookahead_m: float = 0.35           # distance ahead on path to aim at
    linear_speed: float = 0.12          # m/s nominal forward speed
    max_angular: float = 0.4            # rad/s clamp on omega
    angular_gain: float = 1.0           # P-gain on heading error
    rotate_in_place_threshold: float = 0.4  # rad — spin in place above this
    goal_tolerance_m: float = 0.12      # path is "done" within this of last waypoint


class PurePursuit:
    def __init__(self, path: list[tuple[float, float]],
                 config: PurePursuitConfig | None = None) -> None:
        self.path = path
        self.cfg = config or PurePursuitConfig()
        self._idx = 0  # progress along path; never goes backwards

    def is_finished(self, robot_xy: tuple[float, float]) -> bool:
        if not self.path:
            return True
        gx, gy = self.path[-1]
        rx, ry = robot_xy
        return math.hypot(gx - rx, gy - ry) <= self.cfg.goal_tolerance_m

    def step(self, robot_x: float, robot_y: float, robot_yaw: float
             ) -> tuple[float, float]:
        """Return (linear_v, angular_w) for the current robot pose. Returns
        zeros if the path is empty or already finished."""
        if not self.path or self.is_finished((robot_x, robot_y)):
            return 0.0, 0.0

        tx, ty = self._lookahead_point(robot_x, robot_y)

        heading_to_target = math.atan2(ty - robot_y, tx - robot_x)
        err = _angle_wrap(heading_to_target - robot_yaw)

        cfg = self.cfg
        # Big heading error → spin in place. Pure pursuit isn't
        # geometrically valid when the lookahead point is behind us.
        if abs(err) > cfg.rotate_in_place_threshold:
            return 0.0, _clamp(cfg.angular_gain * err, cfg.max_angular)

        # Otherwise: forward motion + proportional steering. Slow down
        # when the heading error is moderate to avoid cutting corners.
        slowdown = max(0.3, 1.0 - abs(err) / cfg.rotate_in_place_threshold)
        v = cfg.linear_speed * slowdown
        w = _clamp(cfg.angular_gain * err, cfg.max_angular)
        return v, w

    def _lookahead_point(self, rx: float, ry: float) -> tuple[float, float]:
        """Find the first waypoint ahead of the robot at distance
        ≥ ``lookahead_m`` from its current position; fall back to the
        last waypoint if no such point exists."""
        L = self.cfg.lookahead_m
        # Skip waypoints we've already passed.
        while self._idx + 1 < len(self.path):
            x, y = self.path[self._idx]
            if math.hypot(x - rx, y - ry) > L:
                break
            self._idx += 1
        for i in range(self._idx, len(self.path)):
            x, y = self.path[i]
            if math.hypot(x - rx, y - ry) >= L:
                return x, y
        return self.path[-1]


def _angle_wrap(a: float) -> float:
    """Wrap angle to [-π, π]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _clamp(x: float, limit: float) -> float:
    return max(-limit, min(limit, x))
