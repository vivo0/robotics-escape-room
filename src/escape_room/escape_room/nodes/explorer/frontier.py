"""Frontier extraction from OccupancyGrid and nearest-frontier goal dispatch."""

from __future__ import annotations

import math

from nav_msgs.msg import OccupancyGrid

MIN_FRONTIER_CELLS = 20


def compute_frontiers(grid: OccupancyGrid) -> list[tuple[float, float]]:
    """Return world-frame centroids of frontier clusters.

    A frontier cell is a free cell (0) with at least one unknown (-1)
    4-neighbour. Adjacent frontier cells are merged into clusters; only
    clusters with >= MIN_FRONTIER_CELLS cells are returned.
    """
    w = grid.info.width
    h = grid.info.height
    data = grid.data
    res = grid.info.resolution
    ox = grid.info.origin.position.x
    oy = grid.info.origin.position.y

    frontier_set: set[tuple[int, int]] = set()
    for r in range(1, h - 1):
        for c in range(1, w - 1):
            if data[r * w + c] != 0:
                continue
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                if data[(r + dr) * w + (c + dc)] == -1:
                    frontier_set.add((r, c))
                    break

    visited: set[tuple[int, int]] = set()
    centroids: list[tuple[float, float]] = []
    for seed in frontier_set:
        if seed in visited:
            continue
        cluster: list[tuple[int, int]] = []
        stack = [seed]
        visited.add(seed)
        while stack:
            r, c = stack.pop()
            cluster.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    nb = (r + dr, c + dc)
                    if nb in frontier_set and nb not in visited:
                        visited.add(nb)
                        stack.append(nb)
        if len(cluster) >= MIN_FRONTIER_CELLS:
            cr = sum(r for r, _ in cluster) / len(cluster)
            cc = sum(c for _, c in cluster) / len(cluster)
            centroids.append((ox + (cc + 0.5) * res, oy + (cr + 0.5) * res))
    return centroids


def send_frontier_goal(node) -> None:
    frontiers = compute_frontiers(node.current_map)
    if not frontiers:
        node.get_logger().info(
            "no frontiers; map fully explored", throttle_duration_sec=5.0
        )
        return
    pose = node.get_robot_pose()
    if pose is None:
        return
    rx, ry, _ = pose
    fx, fy = min(frontiers, key=lambda f: math.hypot(f[0] - rx, f[1] - ry))
    node.publish_nav_goal_path(fx, fy)
    node.nav.send(fx, fy)
