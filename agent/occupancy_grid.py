"""
Occupancy grid — Phase 2.

2D probabilistic occupancy grid built from LiDAR scans + odometry.
Uses log-odds updates to avoid flip-flopping.

Grid coordinate conventions:
  - Origin (cell [cy_origin, cx_origin]) = robot start position in world
  - World +x → increasing col, World +y → increasing row
  - Cell (row, col) stores log-odds value

Log-odds:
  - 0.0  = unknown
  - >0   = likely occupied
  - <0   = likely free
  Clamped to [-5, 5] to prevent over-certainty.
"""

import math
import numpy as np
from pathlib import Path


# Log-odds increments
L_OCC = 0.85   # per occupied hit
L_FREE = -0.40  # per free cell traversed (weaker than hit to avoid over-clearing)
L_CLIP = 5.0   # clamp magnitude

# Grid parameters
RESOLUTION = 0.05          # m per cell
GRID_SIDE_M = 40.0         # total grid size (m) — 40×40 = 800×800 cells
GRID_CELLS = int(GRID_SIDE_M / RESOLUTION)  # cells per side

MAX_RANGE = 8.0            # cap ray-cast range (LiDAR max is 12 m but distant walls matter less)


class OccupancyGrid:
    def __init__(self):
        # log-odds array, float32
        self._grid = np.zeros((GRID_CELLS, GRID_CELLS), dtype=np.float32)
        # origin cell = centre of grid (robot starts here)
        self._cx0 = GRID_CELLS // 2
        self._cy0 = GRID_CELLS // 2

    # ------------------------------------------------------------------ #
    # Public API                                                           #
    # ------------------------------------------------------------------ #

    def update(self, robot_x: float, robot_y: float, robot_yaw: float,
               ranges, angle_min: float, angle_increment: float) -> None:
        """Integrate one LiDAR scan at the given robot pose."""
        rx, ry = self._world_to_cell(robot_x, robot_y)
        if not self._in_bounds(rx, ry):
            return

        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r <= 0.15:
                continue
            hit = r < MAX_RANGE
            ray_range = min(r, MAX_RANGE)

            theta = robot_yaw + angle_min + i * angle_increment
            ex = robot_x + ray_range * math.cos(theta)
            ey = robot_y + ray_range * math.sin(theta)
            ecx, ecy = self._world_to_cell(ex, ey)

            # free cells along ray
            free_cells = _bresenham(rx, ry, ecx, ecy)
            for (fc, fr) in free_cells[:-1]:  # exclude endpoint
                if self._in_bounds(fr, fc):
                    self._grid[fr, fc] = float(
                        np.clip(self._grid[fr, fc] + L_FREE, -L_CLIP, L_CLIP)
                    )

            # occupied endpoint (only if real hit, not max-range clip)
            if hit and self._in_bounds(ecy, ecx):
                self._grid[ecy, ecx] = float(
                    np.clip(self._grid[ecy, ecx] + L_OCC, -L_CLIP, L_CLIP)
                )

    def is_occupied(self, world_x: float, world_y: float, threshold: float = 0.5) -> bool:
        cx, cy = self._world_to_cell(world_x, world_y)
        if not self._in_bounds(cy, cx):
            return False
        return self._grid[cy, cx] > threshold

    def is_free(self, world_x: float, world_y: float, threshold: float = -0.5) -> bool:
        cx, cy = self._world_to_cell(world_x, world_y)
        if not self._in_bounds(cy, cx):
            return False
        return self._grid[cy, cx] < threshold

    def is_unknown(self, world_x: float, world_y: float) -> bool:
        cx, cy = self._world_to_cell(world_x, world_y)
        if not self._in_bounds(cy, cx):
            return True
        return -0.5 <= self._grid[cy, cx] <= 0.5

    @property
    def grid(self) -> np.ndarray:
        """Raw log-odds array (read-only view)."""
        return self._grid

    @property
    def resolution(self) -> float:
        return RESOLUTION

    @property
    def origin_cell(self) -> tuple[int, int]:
        """(col, row) of world origin in grid."""
        return self._cx0, self._cy0

    def cell_to_world(self, col: int, row: int) -> tuple[float, float]:
        x = (col - self._cx0) * RESOLUTION
        y = (row - self._cy0) * RESOLUTION
        return x, y

    def world_to_cell(self, world_x: float, world_y: float) -> tuple[int, int]:
        return self._world_to_cell(world_x, world_y)

    def save_png(self, path: str | Path) -> None:
        """Save occupancy grid as greyscale PNG. Requires Pillow."""
        try:
            from PIL import Image
        except ImportError:
            return

        g = self._grid
        # map log-odds → pixel: free=255 (white), unknown=128 (grey), occupied=0 (black)
        img_arr = np.full_like(g, 128, dtype=np.uint8)
        img_arr[g < -0.5] = 255   # free → white
        img_arr[g > 0.5] = 0      # occupied → black

        # flip vertically so +y is up in the image
        img_arr = np.flipud(img_arr)

        # crop to explored bounding box + margin to keep file small
        occ_mask = (g != 0)
        if occ_mask.any():
            rows = np.where(occ_mask.any(axis=1))[0]
            cols = np.where(occ_mask.any(axis=0))[0]
            margin = 20  # cells
            r0 = max(0, rows[0] - margin)
            r1 = min(GRID_CELLS, rows[-1] + margin + 1)
            c0 = max(0, cols[0] - margin)
            c1 = min(GRID_CELLS, cols[-1] + margin + 1)
            img_arr = img_arr[GRID_CELLS - r1: GRID_CELLS - r0, c0:c1]

        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(img_arr, mode="L").save(path)

    # ------------------------------------------------------------------ #
    # Internal helpers                                                      #
    # ------------------------------------------------------------------ #

    def _world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        col = int(round(x / RESOLUTION)) + self._cx0
        row = int(round(y / RESOLUTION)) + self._cy0
        return col, row

    def _in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < GRID_CELLS and 0 <= col < GRID_CELLS


def _bresenham(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
    """Bresenham line — returns list of (col, row) cells from (x0,y0) to (x1,y1)."""
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return cells
