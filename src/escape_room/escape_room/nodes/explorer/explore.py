"""Frontier-driven exploration: pick the nearest frontier and send it to Nav2."""

import math

from .frontier import compute_frontiers


def send_frontier_goal(node) -> None:
    frontiers = compute_frontiers(node.current_map)
    if not frontiers:
        node.get_logger().info(
            "no frontiers; map fully explored", throttle_duration_sec=5.0
        )
        return
    rx, ry, _ = node.get_robot_pose()
    fx, fy = min(frontiers, key=lambda f: math.hypot(f[0] - rx, f[1] - ry))
    node.publish_nav_goal_path(fx, fy)
    node.nav.send(fx, fy)
