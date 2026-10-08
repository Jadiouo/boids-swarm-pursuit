"""Grid model of the arena for the Nav2 evader's goal choice (M7, C).

Builds the same occupancy the Nav2 bridge publishes (`rasterize_arena`) plus
the cost the planner's inflation layer assigns, and answers three questions
the Euclidean scorer cannot:

* is a candidate reachable at all (geodesic distance over traversable cells)?
* does the target get there before the nearest pursuer (arrival lead)?
* is it an open area or a pocket / dead end (free-space fraction + clearance)?

Pure numpy/scipy, no ROS. A 200x200 map takes a few ms to build the graph
once and roughly 10-20 ms per geodesic field (see tests / docs).
"""

import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage as ndi
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from ..occupancy import (INSCRIBED, grid_array, inflation_cost,
                         rasterize_arena)

SQRT2 = math.sqrt(2.0)


@dataclass
class ReachConfig:
    pursuer_speed_ratio: float = 0.6   # pursuer speed / target speed
    lead_cap: float = 6.0              # metres of pursuer travel
    w_lead: float = 1.5
    w_pocket: float = 1.0
    onward_radius: float = 3.0         # m of travel looked ahead of a goal
    onward_ref_area: float = 8.0       # m^2 of onward room that counts as open
    clearance_cap: float = 1.0         # m, clearance counted as fully open
    top_k: int = 8                     # candidates given the (costlier) pocket check


# ------------------------------------------------------- pure scoring parts
def lead_margin(d_self, d_pursuer, speed_ratio):
    """Metres of pursuer travel the target is ahead at a candidate:
    d_pursuer - speed_ratio * d_self. Positive = the target arrives first
    (pursuers cover `speed_ratio` metres per target metre). -inf if the
    target cannot get there; +inf if only pursuers cannot."""
    if not math.isfinite(d_self):
        return -math.inf
    if not math.isfinite(d_pursuer):
        return math.inf
    return d_pursuer - speed_ratio * d_self


def pocket_penalty(onward_area, clearance, cfg: ReachConfig):
    """0 (open) .. 1 (pocket / dead end).

    onward_area: m^2 of traversable floor within `onward_radius` of the goal
    (by travel distance) that lies no closer to the pursuers than the goal
    itself, i.e. room left to keep fleeing once the goal is reached. A dead
    end has almost none; open ground has about half a disc. A narrow spot
    (small clearance) adds a little."""
    dead = 1.0 - min(onward_area / cfg.onward_ref_area, 1.0)
    narrow = 1.0 - min(clearance / cfg.clearance_cap, 1.0)
    return min(1.0, dead + 0.3 * narrow)


def reach_adjustment(d_self, d_pursuer, onward_area, clearance,
                     cfg: ReachConfig):
    """Additive score term; -inf when the target cannot reach the spot.
    onward_area=None skips the pocket term (first-pass ranking)."""
    lead = lead_margin(d_self, d_pursuer, cfg.pursuer_speed_ratio)
    if lead == -math.inf:
        return -math.inf
    lead_n = max(-1.0, min(1.0, lead / cfg.lead_cap))
    pen = 0.0 if onward_area is None else \
        cfg.w_pocket * pocket_penalty(onward_area, clearance, cfg)
    return cfg.w_lead * lead_n - pen


# --------------------------------------------------------------- the map
class EscapeMap:
    def __init__(self, bmin, bmax, obstacles, resolution=0.1, border_cells=1,
                 robot_radius=0.15, cost_scaling_factor=4.0,
                 inflation_radius=0.8, cfg: ReachConfig = None):
        self.cfg = cfg or ReachConfig()
        self.res = resolution
        self.robot_radius = robot_radius
        self.bmin, self.bmax = tuple(bmin), tuple(bmax)
        g = rasterize_arena(bmax[0] - bmin[0], bmax[1] - bmin[1], resolution,
                            list(obstacles), border_cells, arena_origin=bmin)
        self.origin = g.origin
        self.occ = grid_array(g)
        self.h, self.w = self.occ.shape
        self.cost = inflation_cost(self.occ, resolution, robot_radius,
                                   cost_scaling_factor, inflation_radius)
        self.free = self.cost < INSCRIBED           # body centre may be here
        # metres from each cell centre to the nearest occupied cell centre
        self.clearance = ndi.distance_transform_edt(~self.occ) * resolution
        self._graph = self._build_graph()

    # -- geometry
    def cell(self, xy):
        ix = int(math.floor((xy[0] - self.origin[0]) / self.res))
        iy = int(math.floor((xy[1] - self.origin[1]) / self.res))
        return (min(max(iy, 0), self.h - 1), min(max(ix, 0), self.w - 1))

    def world(self, iy, ix):
        return (self.origin[0] + (ix + 0.5) * self.res,
                self.origin[1] + (iy + 0.5) * self.res)

    def nearest_free(self, xy, reach=0.7):
        """Nearest traversable cell centre within `reach` metres, as world
        xy; None if there is none. Used to snap starts / sources."""
        iy, ix = self.cell(xy)
        if self.free[iy, ix]:
            return self.world(iy, ix)
        n = int(math.ceil(reach / self.res))
        y0, y1 = max(iy - n, 0), min(iy + n + 1, self.h)
        x0, x1 = max(ix - n, 0), min(ix + n + 1, self.w)
        win = self.free[y0:y1, x0:x1]
        ys, xs = np.nonzero(win)
        if len(ys) == 0:
            return None
        d2 = (ys + y0 - iy) ** 2 + (xs + x0 - ix) ** 2
        k = int(np.argmin(d2))
        if math.sqrt(d2[k]) * self.res > reach:
            return None
        return self.world(ys[k] + y0, xs[k] + x0)

    # -- graph / geodesics
    def _build_graph(self):
        h, w = self.h, self.w
        idx = np.arange(h * w).reshape(h, w)
        rows, cols, wts = [], [], []
        for dy, dx, wt in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, SQRT2),
                           (1, -1, SQRT2)):
            ys0 = slice(0, h - dy)
            if dx >= 0:
                xs0, xs1 = slice(0, w - dx), slice(dx, w)
            else:
                xs0, xs1 = slice(-dx, w), slice(0, w + dx)
            a = idx[ys0, xs0]
            b = idx[dy:, xs1]
            ok = self.free[ys0, xs0] & self.free[dy:, xs1]
            if dx != 0 and dy != 0:      # no corner cutting through walls
                ok &= self.free[ys0, xs1] & self.free[dy:, xs0]
            rows.append(a[ok])
            cols.append(b[ok])
            wts.append(np.full(int(ok.sum()), wt * self.res))
        r = np.concatenate(rows)
        c = np.concatenate(cols)
        v = np.concatenate(wts)
        return coo_matrix((np.concatenate([v, v]),
                           (np.concatenate([r, c]), np.concatenate([c, r]))),
                          shape=(h * w, h * w)).tocsr()

    def geodesic(self, sources_xy):
        """Shortest traversable distance (m) from the nearest of the source
        points to every cell, as an (h, w) array; inf where unreachable.
        Sources are snapped to the nearest traversable cell (pursuers hug
        walls too); sources that cannot snap are ignored."""
        idx = []
        for s in sources_xy:
            p = self.nearest_free(s)
            if p is not None:
                iy, ix = self.cell(p)
                idx.append(iy * self.w + ix)
        if not idx:
            return np.full((self.h, self.w), np.inf)
        d = dijkstra(self._graph, directed=False, indices=sorted(set(idx)),
                     min_only=True)
        return d.reshape(self.h, self.w)

    def onward_area(self, xy, d_purs, radius=None):
        """m^2 of cells within `radius` of travel from xy whose pursuer
        distance is not smaller than at xy (room left to keep fleeing)."""
        radius = self.cfg.onward_radius if radius is None else radius
        p = self.nearest_free(xy)
        if p is None:
            return 0.0
        iy, ix = self.cell(p)
        local = dijkstra(self._graph, directed=False, indices=iy * self.w + ix,
                         limit=radius).reshape(self.h, self.w)
        here = d_purs[iy, ix]
        ahead = np.isfinite(local) & (d_purs >= here - 1e-9)
        return float(ahead.sum()) * self.res * self.res

    # -- evader interface
    def context(self, self_xy, pursuers_xy):
        return ReachContext(self, self_xy, pursuers_xy)

    def plan_start(self, xy, reach=0.7):
        """Where to start the planner from: the true position if it is a
        traversable cell, else the nearest traversable cell centre within
        `reach`; else the position itself."""
        if self.free[self.cell(xy)]:
            return xy
        p = self.nearest_free(xy, reach)
        return xy if p is None else p


class ReachContext:
    """Geodesic fields for one goal decision."""

    def __init__(self, emap: EscapeMap, self_xy, pursuers_xy):
        self.emap = emap
        self.d_self = emap.geodesic([self_xy])
        self.d_purs = emap.geodesic(list(pursuers_xy))

    def terms(self, cand, pocket=False):
        e = self.emap
        iy, ix = e.cell(cand)
        area = e.onward_area(cand, self.d_purs) if pocket else None
        return (float(self.d_self[iy, ix]), float(self.d_purs[iy, ix]),
                area, float(e.clearance[iy, ix]))

    def adjustment(self, cand, pocket=False):
        return reach_adjustment(*self.terms(cand, pocket), self.emap.cfg)
