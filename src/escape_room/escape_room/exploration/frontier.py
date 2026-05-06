"""
Frontier detection on an OccupancyGrid.

A frontier cell is a FREE cell that has at least one UNKNOWN 4-neighbor —
i.e. a cell on the boundary between explored-free and unexplored space.
Frontier cells are clustered into connected components; each component is
returned as a `Frontier` with a world-frame centroid and a size.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from escape_room.mapping.occupancy_grid import FREE, UNKNOWN, OccupancyGrid


@dataclass
class Frontier:
    centroid_xy: tuple[float, float]   # world frame
    size: int                          # number of frontier cells in cluster
    cells: list[tuple[int, int]]       # (col, row) for the cluster


def _frontier_mask(grid: OccupancyGrid) -> np.ndarray:
    """Boolean mask of FREE cells with at least one UNKNOWN 4-neighbor."""
    data = grid.data
    free = data == FREE
    unk = data == UNKNOWN

    # neighbour-is-unknown via shifted views; out-of-bounds shifts treated as False
    up = np.zeros_like(unk); up[:-1, :] = unk[1:, :]
    dn = np.zeros_like(unk); dn[1:, :]  = unk[:-1, :]
    lt = np.zeros_like(unk); lt[:, :-1] = unk[:, 1:]
    rt = np.zeros_like(unk); rt[:, 1:]  = unk[:, :-1]
    has_unknown_neighbor = up | dn | lt | rt
    return free & has_unknown_neighbor


def find_frontiers(grid: OccupancyGrid,
                   min_size: int = 3) -> list[Frontier]:
    """Return frontier clusters in the grid, sorted by size (largest first).

    `min_size` filters out tiny noise clusters (e.g. single-cell speckle from
    transient ray-cast aliasing).
    """
    mask = _frontier_mask(grid)
    visited = np.zeros_like(mask, dtype=bool)
    rows, cols = mask.shape

    # 8-connected flood fill
    deltas = [(-1, -1), (-1, 0), (-1, 1),
              (0, -1),           (0, 1),
              (1, -1),  (1, 0),  (1, 1)]

    out: list[Frontier] = []
    res = grid.spec.resolution
    ox, oy = grid.spec.origin_x, grid.spec.origin_y

    for r0 in range(rows):
        for c0 in range(cols):
            if not mask[r0, c0] or visited[r0, c0]:
                continue
            # BFS the cluster
            cluster: list[tuple[int, int]] = []
            q = deque([(r0, c0)])
            visited[r0, c0] = True
            while q:
                r, c = q.popleft()
                cluster.append((c, r))  # store as (col, row)
                for dr, dc in deltas:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols \
                            and mask[nr, nc] and not visited[nr, nc]:
                        visited[nr, nc] = True
                        q.append((nr, nc))
            if len(cluster) < min_size:
                continue
            cs = np.array(cluster, dtype=np.float32)  # (N, 2) col,row
            cx = ox + (cs[:, 0].mean() + 0.5) * res
            cy = oy + (cs[:, 1].mean() + 0.5) * res
            out.append(Frontier(
                centroid_xy=(float(cx), float(cy)),
                size=len(cluster),
                cells=cluster,
            ))

    out.sort(key=lambda f: f.size, reverse=True)
    return out
