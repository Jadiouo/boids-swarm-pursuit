"""Pure-Python OccupancyGrid rasterizer (M7 Nav2 spike, SDD v3 §6.4).

Circles are marked conservatively: a cell is occupied if the circle
intersects the cell rectangle (never under-estimates an obstacle).
Row-major, index = iy * width + ix, ROS OccupancyGrid convention
(origin = world coordinate of the lower-left corner of cell (0, 0)).
"""

import math
from dataclasses import dataclass

OCCUPIED = 100
FREE = 0


@dataclass
class Grid:
    width: int
    height: int
    resolution: float
    origin: tuple
    data: list


def cell_of(x, y, resolution, origin=(0.0, 0.0)):
    return (int(math.floor((x - origin[0]) / resolution)),
            int(math.floor((y - origin[1]) / resolution)))


def rasterize_circles(width_m, height_m, resolution, circles,
                      border_cells=2, origin=(0.0, 0.0)):
    w = int(round(width_m / resolution))
    h = int(round(height_m / resolution))
    data = [FREE] * (w * h)
    for (cx, cy, r) in circles:
        ix0, iy0 = cell_of(cx - r, cy - r, resolution, origin)
        ix1, iy1 = cell_of(cx + r, cy + r, resolution, origin)
        for iy in range(max(iy0, 0), min(iy1, h - 1) + 1):
            y0 = origin[1] + iy * resolution
            ny = min(max(cy, y0), y0 + resolution)
            for ix in range(max(ix0, 0), min(ix1, w - 1) + 1):
                x0 = origin[0] + ix * resolution
                nx = min(max(cx, x0), x0 + resolution)
                if (nx - cx) ** 2 + (ny - cy) ** 2 <= r * r:
                    data[iy * w + ix] = OCCUPIED
    for b in range(min(border_cells, w // 2, h // 2)):
        for ix in range(w):
            data[b * w + ix] = OCCUPIED
            data[(h - 1 - b) * w + ix] = OCCUPIED
        for iy in range(h):
            data[iy * w + b] = OCCUPIED
            data[iy * w + (w - 1 - b)] = OCCUPIED
    return Grid(w, h, resolution, origin, data)


# ----------------------------------------------------------------- arena map
def rasterize_arena(arena_w, arena_h, resolution, circles, border_cells=1,
                    arena_origin=(0.0, 0.0)):
    """Occupancy grid whose wall is drawn OUTSIDE the arena.

    The grid is padded by `border_cells` on every side and only the padding
    is marked occupied, so the lethal band starts exactly at the real wall
    surface (a border drawn inside the arena would claim free floor the sim
    lets the body stand on). The grid origin is therefore
    `arena_origin - border_cells * resolution` and the arena interior keeps
    its world coordinates. Obstacles use the conservative rule of
    `rasterize_circles` (never under-estimated).
    """
    b = int(border_cells)
    w_in = int(round(arena_w / resolution))
    h_in = int(round(arena_h / resolution))
    origin = (arena_origin[0] - b * resolution, arena_origin[1] - b * resolution)
    inner = rasterize_circles((w_in + 2 * b) * resolution,
                              (h_in + 2 * b) * resolution, resolution, circles, border_cells=0,
                              origin=origin)
    w, h = inner.width, inner.height
    data = inner.data
    for k in range(b):
        for ix in range(w):
            data[k * w + ix] = OCCUPIED
            data[(h - 1 - k) * w + ix] = OCCUPIED
        for iy in range(h):
            data[iy * w + k] = OCCUPIED
            data[iy * w + (w - 1 - k)] = OCCUPIED
    return Grid(w, h, resolution, origin, data)


# Nav2 InflationLayer model (nav2_costmap_2d): LETHAL 254 on occupied cells,
# INSCRIBED 253 within the footprint's inscribed radius, then an exponential
# decay out to inflation_radius. robot_radius becomes a 16-gon footprint
# whose inscribed radius is r * cos(pi/16).
LETHAL, INSCRIBED = 254, 253


def inscribed_radius(robot_radius, n_vertices=16):
    return robot_radius * math.cos(math.pi / n_vertices)


def inflation_cost(occupied, resolution, robot_radius,
                   cost_scaling_factor=4.0, inflation_radius=0.8):
    """Per-cell cost (int array) Nav2 would assign to `occupied` (bool array,
    row = y). Distances are cell centre to nearest occupied cell centre,
    like the InflationLayer's distance cache."""
    import numpy as np
    from scipy import ndimage as ndi
    d = ndi.distance_transform_edt(~occupied) * resolution
    r_ins = inscribed_radius(robot_radius)
    cost = np.zeros(occupied.shape, dtype=int)
    near = d <= inflation_radius + 1e-9
    decay = ((INSCRIBED - 1) * np.exp(-cost_scaling_factor * (d - r_ins)))
    cost[near] = decay[near].astype(int)
    cost[d <= r_ins + 1e-9] = INSCRIBED
    cost[occupied] = LETHAL
    return cost


def grid_array(grid):
    """Grid -> bool ndarray [iy, ix] (True = occupied)."""
    import numpy as np
    return np.asarray(grid.data, dtype=np.int16).reshape(
        grid.height, grid.width) >= 50
