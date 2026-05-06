"""
Occupancy grid + ray-casting, pure Python (no ROS).

The grid stores int8 cells with the ROS OccupancyGrid convention:
    -1  = unknown
     0  = free
   100  = occupied

The class is reusable as-is by the A* planner (Phase 2): inflation gives
a planning-safe grid, world↔grid helpers convert between metric and indices.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


UNKNOWN = -1
FREE = 0
OCCUPIED = 100


@dataclass
class GridSpec:
    width_m: float
    height_m: float
    resolution: float
    origin_x: float = 0.0  # world coords of cell (0, 0) lower-left corner
    origin_y: float = 0.0


class OccupancyGrid:
    def __init__(self, spec: GridSpec):
        self.spec = spec
        self.cols = int(round(spec.width_m / spec.resolution))
        self.rows = int(round(spec.height_m / spec.resolution))
        self.data = np.full((self.rows, self.cols), UNKNOWN, dtype=np.int8)

    # ---- coordinate conversion ----------------------------------------

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        col = int((x - self.spec.origin_x) / self.spec.resolution)
        row = int((y - self.spec.origin_y) / self.spec.resolution)
        return col, row

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.cols and 0 <= row < self.rows

    # ---- updates ------------------------------------------------------

    def mark(self, col: int, row: int, value: int) -> None:
        if self.in_bounds(col, row):
            self.data[row, col] = value

    def cast_ray(self, x0: float, y0: float, x1: float, y1: float,
                 hit: bool = True) -> None:
        """Bresenham from (x0,y0) to (x1,y1) in world coords.
        Cells along the ray are marked FREE; the endpoint is marked OCCUPIED
        if hit is True (i.e., the endpoint is the obstacle), else FREE
        (used for max-range / no-return rays).
        """
        c0, r0 = self.world_to_grid(x0, y0)
        c1, r1 = self.world_to_grid(x1, y1)

        dc = abs(c1 - c0)
        dr = abs(r1 - r0)
        sc = 1 if c0 < c1 else -1
        sr = 1 if r0 < r1 else -1
        err = dc - dr

        c, r = c0, r0
        while True:
            if (c, r) != (c1, r1):
                # don't overwrite occupied with free (sticky obstacles)
                if self.in_bounds(c, r) and self.data[r, c] != OCCUPIED:
                    self.data[r, c] = FREE
            else:
                if self.in_bounds(c, r):
                    self.data[r, c] = OCCUPIED if hit else FREE
                break
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c += sc
            if e2 < dc:
                err += dc
                r += sr

    def update_from_scan(self, robot_xy: tuple[float, float],
                         points_xy: np.ndarray,
                         max_range: float) -> None:
        """Vectorised scan integration.

        points_xy: (N, 2) hit points in world frame.
        Each ray from robot to hit is rasterised; rays > max_range are
        truncated and marked free (no-return).
        """
        rx, ry = robot_xy
        if points_xy.size == 0:
            return
        dx = points_xy[:, 0] - rx
        dy = points_xy[:, 1] - ry
        dist = np.hypot(dx, dy)
        valid = dist > 1e-6
        for px, py, d, ok in zip(points_xy[:, 0], points_xy[:, 1], dist, valid):
            if not ok:
                continue
            if d <= max_range:
                self.cast_ray(rx, ry, float(px), float(py), hit=True)
            else:
                # trim the ray to max_range and mark as no-return
                ux, uy = (px - rx) / d, (py - ry) / d
                self.cast_ray(rx, ry,
                              rx + ux * max_range,
                              ry + uy * max_range,
                              hit=False)

    # ---- planning helpers (used by A* in Phase 2) ---------------------

    def inflate(self, radius_cells: int) -> 'OccupancyGrid':
        """Return a copy with occupied cells dilated by radius_cells."""
        if radius_cells <= 0:
            out = OccupancyGrid(self.spec)
            out.data[:] = self.data
            return out
        # simple Manhattan dilation; cheap and good enough for grids ~50x40
        occ = (self.data == OCCUPIED)
        dilated = occ.copy()
        for d in range(1, radius_cells + 1):
            dilated[d:, :] |= occ[:-d, :]
            dilated[:-d, :] |= occ[d:, :]
            dilated[:, d:] |= occ[:, :-d]
            dilated[:, :-d] |= occ[:, d:]
        out = OccupancyGrid(self.spec)
        out.data[:] = self.data
        out.data[dilated] = OCCUPIED
        return out

    def is_traversable(self, col: int, row: int) -> bool:
        """Free cells are traversable. Unknown cells are NOT (be conservative)."""
        if not self.in_bounds(col, row):
            return False
        return self.data[row, col] == FREE
