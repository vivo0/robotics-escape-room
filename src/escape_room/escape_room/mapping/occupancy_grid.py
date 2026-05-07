"""
2D occupancy grid backed by two boolean masks: `free` and `occ`.

A cell is OCCUPIED when ``occ`` is True (priority over everything),
FREE when ``free`` is True and not occupied, and UNKNOWN otherwise.

The grid is permissive about repeated observations:
    - An OCC mark, once set, is permanent (suits the static simulated world).
    - FREE marks accumulate; OCC always overrides FREE in the public view.
    - Cells never written to remain UNKNOWN.

This module is consumed by the planner (A*) and the frontier detector;
the obstacle detection itself happens deterministically in
``mapper_node`` by querying CoppeliaSim, so the grid only records what
the robot has been able to *see* from the positions it has visited.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


# ROS OccupancyGrid convention used by the public ``data`` view.
UNKNOWN = -1
FREE = 0
OCCUPIED = 100


@dataclass
class GridSpec:
    """Geometry of the grid in metres. ``origin_*`` is the world-frame
    position of the lower-left corner of cell (0, 0)."""
    width_m: float
    height_m: float
    resolution: float
    origin_x: float = 0.0
    origin_y: float = 0.0


class OccupancyGrid:
    def __init__(self, spec: GridSpec):
        self.spec = spec
        self.cols = int(round(spec.width_m / spec.resolution))
        self.rows = int(round(spec.height_m / spec.resolution))
        self.free = np.zeros((self.rows, self.cols), dtype=bool)
        self.occ = np.zeros((self.rows, self.cols), dtype=bool)

    # ---- coordinate conversion ----------------------------------------

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """World (x, y) in metres → (col, row) of the containing cell."""
        col = int((x - self.spec.origin_x) / self.spec.resolution)
        row = int((y - self.spec.origin_y) / self.spec.resolution)
        return col, row

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.cols and 0 <= row < self.rows

    # ---- public view --------------------------------------------------

    @property
    def data(self) -> np.ndarray:
        """ROS OccupancyGrid-style int8 view of the grid. Recomputed on
        every access (the underlying state is the boolean masks)."""
        out = np.full((self.rows, self.cols), UNKNOWN, dtype=np.int8)
        out[self.free] = FREE
        out[self.occ] = OCCUPIED  # OCC takes priority
        return out

    def is_traversable(self, col: int, row: int) -> bool:
        """True iff the cell is observed-FREE and not occupied. UNKNOWN
        cells count as blocked — be conservative for planning."""
        if not self.in_bounds(col, row):
            return False
        return bool(self.free[row, col]) and not bool(self.occ[row, col])

    # ---- planning helper ---------------------------------------------

    def inflate(self, radius_cells: int) -> 'OccupancyGrid':
        """Return a copy with the OCC mask Manhattan-dilated by
        ``radius_cells``. Treating obstacles as bigger by the robot's
        radius lets A* plan with a point robot — the standard
        configuration-space trick."""
        out = OccupancyGrid(self.spec)
        out.free[:] = self.free
        out.occ[:] = self.occ
        if radius_cells <= 0:
            return out
        occ = self.occ.copy()
        dilated = occ.copy()
        for d in range(1, radius_cells + 1):
            dilated[d:, :] |= occ[:-d, :]
            dilated[:-d, :] |= occ[d:, :]
            dilated[:, d:] |= occ[:, :-d]
            dilated[:, :-d] |= occ[:, d:]
        out.occ |= dilated
        return out
