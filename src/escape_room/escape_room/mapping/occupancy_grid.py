"""
2D occupancy grid with hybrid hit-counter / log-odds updates.

Tailored for the static-world assumption of this project: walls and
obstacles never move once placed in CoppeliaSim, so a cell that has been
confirmed as an obstacle by enough independent hits is "frozen" OCC and
no future free pass can erase it. This kills the wall-smearing problem
that pure log-odds suffers from when the robot rotates and the per-scan
TF jitter shifts the apparent wall position by a cell or two.

Per-cell state:
    hits     : uint16 counter, incremented on each ray endpoint that
               terminates here. Once `hits >= HIT_THRESHOLD` the cell
               is permanently OCC.
    log_odds : float belief used only to decide FREE vs UNKNOWN. Free
               passes accumulate (negative); doesn't affect OCC cells.

Public `.data` view converts to the ROS OccupancyGrid convention:
    -1  = unknown
     0  = free
   100  = occupied
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
    # Log-odds is used only for FREE detection here.
    L_FREE = -0.4
    L_MIN = -2.0
    L_THRESH_FREE = -0.4
    # Hit counter: a cell becomes permanently OCC once >= HIT_THRESHOLD
    # rays have terminated on it. 2 is enough in simulation because hits
    # are noise-free; in real life you'd raise this.
    HIT_THRESHOLD = 2

    def __init__(self, spec: GridSpec):
        self.spec = spec
        self.cols = int(round(spec.width_m / spec.resolution))
        self.rows = int(round(spec.height_m / spec.resolution))
        self.log_odds = np.zeros((self.rows, self.cols), dtype=np.float32)
        self.hits = np.zeros((self.rows, self.cols), dtype=np.uint16)

    # ---- coordinate conversion ----------------------------------------

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        col = int((x - self.spec.origin_x) / self.spec.resolution)
        row = int((y - self.spec.origin_y) / self.spec.resolution)
        return col, row

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.cols and 0 <= row < self.rows

    @property
    def data(self) -> np.ndarray:
        """int8 ROS-style view. Recomputed on access.

        Priority: a cell with hits >= HIT_THRESHOLD is OCC regardless of
        log-odds (frozen wall). Otherwise FREE if log-odds is negative
        enough, else UNKNOWN.
        """
        out = np.full((self.rows, self.cols), UNKNOWN, dtype=np.int8)
        out[self.log_odds <= self.L_THRESH_FREE] = FREE
        out[self.hits >= self.HIT_THRESHOLD] = OCCUPIED
        return out

    # ---- updates ------------------------------------------------------

    def mark(self, col: int, row: int, value: int) -> None:
        """Force a single cell. Used by tests; production uses cast_ray."""
        if not self.in_bounds(col, row):
            return
        if value == OCCUPIED:
            self.hits[row, col] = self.HIT_THRESHOLD
        elif value == FREE:
            self.log_odds[row, col] = self.L_MIN
            self.hits[row, col] = 0
        else:
            self.log_odds[row, col] = 0.0
            self.hits[row, col] = 0

    def cast_ray(self, x0: float, y0: float, x1: float, y1: float,
                 hit: bool = True) -> None:
        """Bresenham from (x0,y0) to (x1,y1) in world coords. Cells along
        the ray accumulate free evidence (log_odds += L_FREE); the
        endpoint increments the hit counter if hit else accumulates free.

        Free evidence does NOT clear hits — once a cell crosses
        HIT_THRESHOLD it is permanently OCC.
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
                if self.in_bounds(c, r):
                    v = self.log_odds[r, c] + self.L_FREE
                    self.log_odds[r, c] = max(self.L_MIN, v)
            else:
                if self.in_bounds(c, r):
                    if hit:
                        # Cap the counter to stay in uint16 range.
                        if self.hits[r, c] < 65535:
                            self.hits[r, c] += 1
                    else:
                        v = self.log_odds[r, c] + self.L_FREE
                        self.log_odds[r, c] = max(self.L_MIN, v)
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
        truncated and treated as no-return (cells along the ray free, but
        endpoint also free since we have no evidence of an obstacle).
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
                ux, uy = (px - rx) / d, (py - ry) / d
                self.cast_ray(rx, ry,
                              rx + ux * max_range,
                              ry + uy * max_range,
                              hit=False)

    # ---- planning helpers (used by A*) --------------------------------

    def inflate(self, radius_cells: int) -> 'OccupancyGrid':
        """Return a copy with occupied cells dilated by radius_cells."""
        out = OccupancyGrid(self.spec)
        out.log_odds[:] = self.log_odds
        out.hits[:] = self.hits
        if radius_cells <= 0:
            return out
        occ = (self.data == OCCUPIED)
        dilated = occ.copy()
        for d in range(1, radius_cells + 1):
            dilated[d:, :] |= occ[:-d, :]
            dilated[:-d, :] |= occ[d:, :]
            dilated[:, d:] |= occ[:, :-d]
            dilated[:, :-d] |= occ[:, d:]
        out.hits[dilated] = self.HIT_THRESHOLD
        return out

    def is_traversable(self, col: int, row: int) -> bool:
        """Free cells only — UNKNOWN and OCC count as blocked."""
        if not self.in_bounds(col, row):
            return False
        if self.hits[row, col] >= self.HIT_THRESHOLD:
            return False
        return self.log_odds[row, col] <= self.L_THRESH_FREE
