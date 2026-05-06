"""
A* path planning on an OccupancyGrid.

Works on the inflated grid: only FREE cells are traversable (UNKNOWN is
treated as blocked, so the planner never crosses unexplored space).
8-connected with octile heuristic; returns world-frame waypoints.
"""
from __future__ import annotations

import heapq
import math

from escape_room.mapping.occupancy_grid import OccupancyGrid


# 8-connected neighbour offsets and their step costs
_NEIGHBOURS = [
    (-1,  0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)),  (1, 1, math.sqrt(2)),
]


def _octile(c0: int, r0: int, c1: int, r1: int) -> float:
    """Admissible heuristic for 8-connected grids."""
    dc = abs(c1 - c0)
    dr = abs(r1 - r0)
    return (dc + dr) + (math.sqrt(2) - 2.0) * min(dc, dr)


def plan_path(grid: OccupancyGrid,
              start_xy: tuple[float, float],
              goal_xy: tuple[float, float]) -> list[tuple[float, float]] | None:
    """A* on `grid`. Returns a list of world (x, y) waypoints from start to
    goal (inclusive of both endpoints), or None if unreachable.

    `grid` is expected to already be inflated by the robot radius.
    """
    sc, sr = grid.world_to_grid(*start_xy)
    gc, gr = grid.world_to_grid(*goal_xy)

    # If start sits on a non-traversable cell (e.g. inflation hugs the wall
    # right where the robot is), allow it as a starting point anyway —
    # otherwise we'd never plan when the robot is wedged near a wall.
    if not grid.in_bounds(sc, sr) or not grid.in_bounds(gc, gr):
        return None
    if not grid.is_traversable(gc, gr):
        return None

    start = (sc, sr)
    goal = (gc, gr)
    if start == goal:
        return [start_xy, goal_xy]

    open_heap: list = []
    heapq.heappush(open_heap, (0.0, 0, start))
    g_score: dict[tuple[int, int], float] = {start: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    counter = 1  # tiebreaker for heap order

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current == goal:
            return _reconstruct(grid, came_from, current, start_xy, goal_xy)

        cc, cr = current
        for dc, dr, step in _NEIGHBOURS:
            nc, nr = cc + dc, cr + dr
            if not grid.in_bounds(nc, nr):
                continue
            if not grid.is_traversable(nc, nr):
                # Only block if it's NOT the start cell — see comment above
                if (nc, nr) != start:
                    continue
            tentative = g_score[current] + step
            nb = (nc, nr)
            if tentative < g_score.get(nb, math.inf):
                g_score[nb] = tentative
                came_from[nb] = current
                f = tentative + _octile(nc, nr, gc, gr)
                heapq.heappush(open_heap, (f, counter, nb))
                counter += 1

    return None


def _reconstruct(grid: OccupancyGrid,
                 came_from: dict,
                 end: tuple[int, int],
                 start_xy: tuple[float, float],
                 goal_xy: tuple[float, float]) -> list[tuple[float, float]]:
    """Walk came_from back to start; convert to world (x, y) cell centers."""
    path_cells = [end]
    while end in came_from:
        end = came_from[end]
        path_cells.append(end)
    path_cells.reverse()

    res = grid.spec.resolution
    ox, oy = grid.spec.origin_x, grid.spec.origin_y
    waypoints = [
        (ox + (c + 0.5) * res, oy + (r + 0.5) * res)
        for c, r in path_cells
    ]
    # Replace endpoints with the exact metric values asked for
    waypoints[0] = start_xy
    waypoints[-1] = goal_xy
    return waypoints
