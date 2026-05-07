"""
Frontier detection on an OccupancyGrid.

A *frontier cell* is a FREE cell with at least one UNKNOWN 4-neighbor
— i.e. a cell on the boundary between explored-free and unexplored
space. Frontier cells are clustered into 8-connected components; each
component is returned as a ``Frontier`` with a world-frame centroid
and a size.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from escape_room.mapping.occupancy_grid import FREE, UNKNOWN, OccupancyGrid


@dataclass
class Frontier:
    centroid_xy: tuple[float, float]   # world-frame centroid (m)
    size: int                          # number of cells in the cluster
    cells: list[tuple[int, int]]       # member cells as (col, row)


def _frontier_mask(grid: OccupancyGrid) -> np.ndarray:
    """Boolean mask of FREE cells with at least one UNKNOWN 4-neighbor.

    We shift the UNKNOWN mask in each cardinal direction and OR the
    shifted views — much faster than per-cell neighbor checks.
    """
    data = grid.data
    free = data == FREE
    unk = data == UNKNOWN

    up = np.zeros_like(unk); up[:-1, :] = unk[1:, :]
    dn = np.zeros_like(unk); dn[1:, :] = unk[:-1, :]
    lt = np.zeros_like(unk); lt[:, :-1] = unk[:, 1:]
    rt = np.zeros_like(unk); rt[:, 1:] = unk[:, :-1]
    return free & (up | dn | lt | rt)


# 8-connected neighborhood for cluster flood fill.
_NEIGHBOR_DELTAS = (
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1),           (0, 1),
    (1, -1),  (1, 0),  (1, 1),
)


def find_frontiers(grid: OccupancyGrid,
                   min_size: int = 3) -> list[Frontier]:
    """Return frontier clusters in ``grid`` sorted by size (largest first).

    ``min_size`` filters out tiny clusters caused by the boundary
    detection picking up isolated speckle.
    """
    mask = _frontier_mask(grid)
    visited = np.zeros_like(mask, dtype=bool)
    rows, cols = mask.shape
    res = grid.spec.resolution
    ox, oy = grid.spec.origin_x, grid.spec.origin_y

    out: list[Frontier] = []

    for r0 in range(rows):
        for c0 in range(cols):
            if not mask[r0, c0] or visited[r0, c0]:
                continue
            cluster = _flood_fill(mask, visited, r0, c0)
            if len(cluster) < min_size:
                continue
            cs = np.asarray(cluster, dtype=np.float32)  # (N, 2): col, row
            cx = ox + (cs[:, 0].mean() + 0.5) * res
            cy = oy + (cs[:, 1].mean() + 0.5) * res
            out.append(Frontier(
                centroid_xy=(float(cx), float(cy)),
                size=len(cluster),
                cells=cluster,
            ))

    out.sort(key=lambda f: f.size, reverse=True)
    return out


def _flood_fill(mask: np.ndarray, visited: np.ndarray,
                r0: int, c0: int) -> list[tuple[int, int]]:
    """BFS over the 8-neighborhood; collect (col, row) cells of the cluster."""
    rows, cols = mask.shape
    cluster: list[tuple[int, int]] = []
    q: deque = deque([(r0, c0)])
    visited[r0, c0] = True
    while q:
        r, c = q.popleft()
        cluster.append((c, r))
        for dr, dc in _NEIGHBOR_DELTAS:
            nr, nc = r + dr, c + dc
            if (0 <= nr < rows and 0 <= nc < cols
                    and mask[nr, nc] and not visited[nr, nc]):
                visited[nr, nc] = True
                q.append((nr, nc))
    return cluster
