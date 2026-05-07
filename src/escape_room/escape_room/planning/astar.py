"""
A* path planning on an OccupancyGrid.

Operates on the inflated grid: only FREE cells are traversable
(UNKNOWN is treated as blocked, so the planner never crosses
unexplored space). 8-connected with the octile heuristic — admissible
on grids that allow diagonal moves at cost √2.
"""
from __future__ import annotations

import heapq
import math

from escape_room.mapping.occupancy_grid import OccupancyGrid


# 8-connected neighbour offsets and their step costs (col_delta, row_delta, cost).
_NEIGHBOURS = (
    (-1,  0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)),  (1, 1, math.sqrt(2)),
)


def _octile(c0: int, r0: int, c1: int, r1: int) -> float:
    """Admissible heuristic for 8-connected grids."""
    dc = abs(c1 - c0)
    dr = abs(r1 - r0)
    return (dc + dr) + (math.sqrt(2) - 2.0) * min(dc, dr)


def plan_path(grid: OccupancyGrid,
              start_xy: tuple[float, float],
              goal_xy: tuple[float, float],
              ) -> list[tuple[float, float]] | None:
    """A* on ``grid``. Returns world (x, y) waypoints from start to
    goal (inclusive of both endpoints), or None if unreachable.

    ``grid`` is expected to be already inflated by the robot radius.
    """
    sc, sr = grid.world_to_grid(*start_xy)
    gc, gr = grid.world_to_grid(*goal_xy)

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
    counter = 1  # tiebreaker so the heap never compares tuples by node

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current == goal:
            return _reconstruct(grid, came_from, current, start_xy, goal_xy)

        cc, cr = current
        for dc, dr, step in _NEIGHBOURS:
            nc, nr = cc + dc, cr + dr
            if not grid.in_bounds(nc, nr):
                continue
            # Allow leaving the start cell even if the inflation hugs
            # right next to the robot (otherwise we could never plan
            # when the robot is wedged near a wall).
            if not grid.is_traversable(nc, nr) and (nc, nr) != start:
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
                 goal_xy: tuple[float, float],
                 ) -> list[tuple[float, float]]:
    """Walk ``came_from`` back to start, return world (x, y) cell centers.
    The first/last entries are replaced with the exact metric endpoints
    requested so pure pursuit doesn't need to snap to cell centers."""
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
    waypoints[0] = start_xy
    waypoints[-1] = goal_xy
    return waypoints
