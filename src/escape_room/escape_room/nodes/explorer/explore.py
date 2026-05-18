"""Frontier-driven exploration: pick the nearest frontier and send it to Nav2."""

import math

from .frontier import compute_frontiers


def send_frontier_goal(node) -> bool:
    """Send next frontier goal. Returns False when no frontiers remain."""
    frontiers = compute_frontiers(node.current_map)
    if not frontiers:
        node.get_logger().info(
            "no frontiers; map fully explored", throttle_duration_sec=5.0
        )
        return False
    pose = node.get_robot_pose()
    if pose is None:
        return True
    rx, ry, _ = pose
    fx, fy = min(frontiers, key=lambda f: math.hypot(f[0] - rx, f[1] - ry))
    node.publish_nav_goal_path(fx, fy)
    node.nav.send(fx, fy)
    return True
