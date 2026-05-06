"""
Pure-pursuit path follower.

Given a path (list of world (x, y) waypoints) and the robot's current pose
(x, y, yaw in the same frame), pick a "lookahead" point on the path ahead
of the robot, and return the (linear, angular) command that steers toward
it. Standard textbook implementation; tuned for a small differential
drive at low speeds.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class PurePursuitConfig:
    lookahead_m: float = 0.35       # distance ahead on path to aim at
    linear_speed: float = 0.08      # m/s nominal forward speed
    max_angular: float = 0.15       # rad/s clamp on omega — kept very low
                                    # so ZMQ query lag (cloud arrives → we
                                    # query sensor pose) doesn't translate
                                    # into a visible per-scan rotation
                                    # error. At 0.15 rad/s and ~50 ms lag,
                                    # the pose offset per scan is < 0.5°.
    angular_gain: float = 0.5       # P-gain on heading error
    rotate_in_place_threshold: float = 0.4  # rad; spin in place above this
    goal_tolerance_m: float = 0.12  # path is "done" when within this of last waypoint


class PurePursuit:
    def __init__(self, path: list[tuple[float, float]],
                 config: PurePursuitConfig | None = None):
        self.path = path
        self.cfg = config or PurePursuitConfig()
        self._last_idx = 0  # progress along path; never goes backwards

    def is_finished(self, robot_xy: tuple[float, float]) -> bool:
        if not self.path:
            return True
        gx, gy = self.path[-1]
        rx, ry = robot_xy
        return math.hypot(gx - rx, gy - ry) <= self.cfg.goal_tolerance_m

    def step(self, robot_x: float, robot_y: float, robot_yaw: float
             ) -> tuple[float, float]:
        """Return (linear_v, angular_w) for the current robot pose.

        Returns (0, 0) if the path is empty or already finished.
        """
        if self.is_finished((robot_x, robot_y)) or not self.path:
            return 0.0, 0.0

        target = self._lookahead_point(robot_x, robot_y)
        tx, ty = target

        # Heading error in robot frame
        heading_to_target = math.atan2(ty - robot_y, tx - robot_x)
        err = _angle_wrap(heading_to_target - robot_yaw)

        cfg = self.cfg
        # If we are pointing way off, spin in place — pure pursuit isn't
        # geometrically valid when the lookahead point is behind us.
        if abs(err) > cfg.rotate_in_place_threshold:
            w = max(-cfg.max_angular, min(cfg.max_angular, cfg.angular_gain * err))
            return 0.0, w

        # Otherwise: forward + proportional steering. Slow down when the
        # heading error is moderate to avoid cutting corners.
        slow = max(0.3, 1.0 - abs(err) / cfg.rotate_in_place_threshold)
        v = cfg.linear_speed * slow
        w = max(-cfg.max_angular, min(cfg.max_angular, cfg.angular_gain * err))
        return v, w

    def _lookahead_point(self, rx: float, ry: float) -> tuple[float, float]:
        """Walk forward from `_last_idx` until we find a waypoint at least
        `lookahead_m` away, or fall back to the last waypoint.
        """
        L = self.cfg.lookahead_m
        # Advance progress: skip waypoints we've already passed.
        while self._last_idx + 1 < len(self.path):
            x, y = self.path[self._last_idx]
            if math.hypot(x - rx, y - ry) > L:
                break
            self._last_idx += 1
        # Now find first point at distance >= L from current pose
        for i in range(self._last_idx, len(self.path)):
            x, y = self.path[i]
            if math.hypot(x - rx, y - ry) >= L:
                return (x, y)
        return self.path[-1]


def _angle_wrap(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a
