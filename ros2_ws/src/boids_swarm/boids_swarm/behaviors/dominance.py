"""Dominance-region escape planning (shared by SmartEvader and Nav2Evader).

Idea (pursuit-evasion "dominance region" / Apollonius region): a point x is
safe to run to if the evader gets there clearly before any pursuer does,

    D = { x :  T_e(x) + margin  <  T_p(x) }

* T_p(x): earliest time any pursuer can be at x. One multi-source geodesic
  (Dijkstra over the 8-neighbour occupancy graph of `EscapeMap`) divided by
  the pursuer speed. Obstacles are respected, so a pillar that lengthens the
  pursuers' detour widens D behind it (the discrete Apollonius curve bends
  around obstacles).
* T_e(x): earliest time the evader is at x = geodesic / evader speed plus a
  turning delay |bearing - heading| / omega (the evader's turn rate is
  capped, so a goal behind it is far away in time even when near in space).
  Heading delay of the pursuers is not modelled (they are flockers, not
  planners; the margin absorbs it).
* D is the connected piece of that set containing the evader; a "safe" patch
  the evader could only reach through unsafe ground does not count.

Why 8-neighbour Dijkstra rather than fast marching: it reuses the graph
EscapeMap already holds (obstacle-aware, no corner cutting) and runs in
~2 ms per field on 200x200 via scipy; FMM would remove the up-to-8 % octile
over-estimate but needs a hand-written heap loop that is much slower in pure
Python. The octile bias only blurs the boundary of D, which the margin
dwarfs.

Goal choice inside D maximises safety margin (capped, so past "safe enough"
openness decides), openness, distance from the pursuer centroid and stays
out of the pursuers' convex hull (capture = target inside the hull of >= 3
pursuers). If D is empty the least-bad point (largest T_p - T_e) is chosen,
steered through the widest angular gap between pursuers (break out).
The incumbent goal is kept until it leaves D, is reached, or a rival beats
it by `hysteresis`.

Pure numpy/scipy, no ROS.
"""

import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage as ndi

from ..game import convex_hull


@dataclass
class DominanceConfig:
    v_self: float = 3.6            # m/s the evader can run
    v_purs: float = 2.0            # m/s of a pursuer
    omega: float = 1.2             # rad/s evader turn-rate cap
    margin_s: float = 0.6          # seconds the evader must be ahead
    margin_cap_s: float = 4.0      # margin beyond this adds no more score
    # goal score weights (all terms ~[0, 1])
    w_margin: float = 1.0
    w_margin_fb: float = 2.0       # empty D: weight on the signed margin
    w_room: float = 0.8
    w_clear: float = 0.3
    w_hull: float = 1.2
    w_away: float = 0.4
    w_travel: float = 0.25
    w_gap: float = 0.8
    room_radius: float = 3.0       # m window for the free-floor share
    clear_cap: float = 3.0
    hull_radius: float = 8.0       # pursuers this near the evader form the hull
    hull_soft: float = 2.0         # m over which the hull penalty fades
    gap_radius: float = 6.0
    min_goal_dist: float = 2.0     # geodesic metres
    max_goal_dist: float = 16.0
    reached_dist: float = 1.0
    goal_clearance: float = 0.5    # m a goal keeps from walls / obstacles
    hysteresis: float = 0.12       # rival must beat the incumbent by this


@dataclass
class Fields:
    t_e: np.ndarray
    t_p: np.ndarray
    d_self: np.ndarray
    d_purs: np.ndarray
    region: np.ndarray
    self_cell: tuple
    margin_s: float


@dataclass
class Choice:
    goal: tuple            # (x, y) world, None if nothing is reachable
    score: float
    empty: bool            # D was empty: the goal is a least-bad break-out
    switched: bool         # differs from the incumbent
    margin: float          # T_p - T_e at the goal (s)


# ------------------------------------------------------------- grid caches
def grid_xy(emap):
    """World x / y of every cell centre, cached on the map."""
    g = getattr(emap, '_dom_xy', None)
    if g is None:
        iy, ix = np.mgrid[0:emap.h, 0:emap.w]
        g = (emap.origin[0] + (ix + 0.5) * emap.res,
             emap.origin[1] + (iy + 0.5) * emap.res)
        emap._dom_xy = g
    return g


def room_map(emap, radius=3.0):
    """Share of free floor within `radius`: ~1 open, ~0.5 along a wall,
    ~0.25 in a corner, small in a pocket. Static per map (cached)."""
    r = getattr(emap, 'room', None)
    if r is None:
        win = max(3, int(round(radius / emap.res)))
        occ = (~emap.free).astype(float)
        r = 1.0 - ndi.uniform_filter(occ, size=win, mode='constant', cval=1.0)
        emap.room = r
    return r


# ----------------------------------------------------------------- fields
def pursuer_fields(emap, pursuers_xy, cfg):
    """(d_purs, t_p): geodesic metres and seconds of the earliest pursuer."""
    if not len(pursuers_xy):
        inf = np.full((emap.h, emap.w), np.inf)
        return inf, inf
    d = emap.geodesic([(p[0], p[1]) for p in pursuers_xy])
    return d, d / cfg.v_purs


def turn_time(emap, d, self_xy, theta, cfg):
    """T_e from a geodesic field `d`: travel time plus the delay for turning
    from `theta` toward each cell's bearing (omega-capped)."""
    X, Y = grid_xy(emap)
    ang = np.arctan2(Y - self_xy[1], X - self_xy[0])
    dth = np.abs(np.arctan2(np.sin(ang - theta), np.cos(ang - theta)))
    ramp = np.clip(d / 1.0, 0.0, 1.0)      # no bearing for cells at our feet
    return d / cfg.v_self + ramp * dth / cfg.omega


def self_fields(emap, self_xy, theta, cfg):
    """(d_self, t_e): geodesic metres and seconds incl. the turning delay."""
    d = emap.geodesic([self_xy])
    return d, turn_time(emap, d, self_xy, theta, cfg)


def dominance_region(emap, t_e, t_p, margin_s, self_xy):
    """Boolean mask of D: cells the evader beats every pursuer to by
    `margin_s`, connected to the evader through such cells."""
    raw = np.isfinite(t_e) & (t_e + margin_s < t_p)
    s = emap.nearest_free(self_xy)
    if s is None:
        return np.zeros_like(raw)
    iy, ix = emap.cell(s)
    if not raw[iy, ix]:
        return np.zeros_like(raw)
    lab, _ = ndi.label(raw, structure=np.ones((3, 3), dtype=int))
    return lab == lab[iy, ix]


def assemble_fields(emap, self_xy, d_self, t_e, d_purs, t_p, cfg):
    s = emap.nearest_free(self_xy) or self_xy
    return Fields(t_e, t_p, d_self, d_purs,
                  dominance_region(emap, t_e, t_p, cfg.margin_s, self_xy),
                  emap.cell(s), cfg.margin_s)


def compute_fields(emap, self_xy, theta, pursuers_xy, cfg):
    d_purs, t_p = pursuer_fields(emap, pursuers_xy, cfg)
    d_self, t_e = self_fields(emap, self_xy, theta, cfg)
    return assemble_fields(emap, self_xy, d_self, t_e, d_purs, t_p, cfg)


# ----------------------------------------------------------- scoring parts
def hull_penalty(X, Y, pursuers_xy, soft=2.0):
    """1 inside the convex hull of the pursuers, fading to 0 over `soft`
    metres outside it; 0 when they do not span a hull."""
    pts = [(float(p[0]), float(p[1])) for p in pursuers_xy]
    hull = convex_hull(pts)
    if len(hull) < 3:
        return np.zeros_like(X, dtype=float)
    out = np.full(X.shape, -np.inf)       # max over edges of outward distance
    n = len(hull)
    for i in range(n):
        ax, ay = hull[i]
        bx, by = hull[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        ln = math.hypot(ex, ey)
        if ln < 1e-9:
            continue
        # CCW hull: inside is on the left; outward distance = -cross/len
        out = np.maximum(out, -(ex * (Y - ay) - ey * (X - ax)) / ln)
    return np.where(out <= 0.0, 1.0, np.clip(1.0 - out / soft, 0.0, 1.0))


def widest_gap(self_xy, pursuers_xy, radius=6.0):
    """(centre bearing, width) in radians of the widest pursuer-free arc
    around the evader, counting pursuers within `radius`; None with fewer
    than two (no gap to speak of)."""
    angs = sorted(math.atan2(p[1] - self_xy[1], p[0] - self_xy[0])
                  for p in pursuers_xy
                  if math.hypot(p[0] - self_xy[0], p[1] - self_xy[1]) < radius)
    if len(angs) < 2:
        return None
    best, centre = -1.0, 0.0
    for i, a in enumerate(angs):
        b = angs[(i + 1) % len(angs)]
        w = (b - a) % (2.0 * math.pi)
        if w > best:
            best, centre = w, a + w / 2.0
    return centre, best


def _centroid(pursuers_xy):
    n = len(pursuers_xy)
    return (sum(p[0] for p in pursuers_xy) / n,
            sum(p[1] for p in pursuers_xy) / n)


def _score_all(emap, f, self_xy, pursuers_xy, cfg, empty):
    """Score of every cell (no masking) and the T_p - T_e margin field."""
    X, Y = grid_xy(emap)
    cap = cfg.margin_cap_s
    with np.errstate(invalid='ignore'):
        mrg = np.where(np.isfinite(f.t_p), f.t_p - f.t_e, cap + cfg.margin_s)
    mrg = np.where(np.isfinite(mrg), mrg, -cap)
    if empty:
        m_n = cfg.w_margin_fb * np.clip(mrg / cap, -1.0, 1.0)
    else:
        m_n = cfg.w_margin * np.clip((mrg - cfg.margin_s) / cap, 0.0, 1.0)
    room = room_map(emap, cfg.room_radius)
    sc = (m_n + cfg.w_room * np.clip(room / 0.9, 0.0, 1.0)
          + cfg.w_clear * np.minimum(emap.clearance, cfg.clear_cap)
          / cfg.clear_cap
          - cfg.w_travel * np.clip(np.where(np.isfinite(f.d_self), f.d_self,
                                            cfg.max_goal_dist)
                                   / cfg.max_goal_dist, 0.0, 1.0))
    if len(pursuers_xy):
        cx, cy = _centroid(pursuers_xy)
        sc = sc + cfg.w_away * np.minimum(np.hypot(X - cx, Y - cy), 15.0) / 15.0
        near = [p for p in pursuers_xy
                if math.hypot(p[0] - self_xy[0], p[1] - self_xy[1])
                < cfg.hull_radius]
        sc = sc - cfg.w_hull * hull_penalty(X, Y, near, cfg.hull_soft)
        if empty:
            g = widest_gap(self_xy, pursuers_xy, cfg.gap_radius)
            if g is not None:
                centre, width = g
                brg = np.arctan2(Y - self_xy[1], X - self_xy[0])
                sc = sc + cfg.w_gap * min(width / math.pi, 1.0) \
                    * np.cos(brg - centre)
    return sc, mrg


def _excluded(emap, exclude):
    X, Y = grid_xy(emap)
    bad = np.zeros(X.shape, dtype=bool)
    for (x, y, r) in exclude:
        bad |= np.hypot(X - x, Y - y) <= r
    return bad


# ------------------------------------------------------------ goal choice
def select_goal(emap, f, self_xy, pursuers_xy, incumbent, cfg, exclude=()):
    """Pick the escape goal. Returns a `Choice`; goal is None only when no
    cell is reachable at all."""
    empty = not f.region.any()
    sc, mrg = _score_all(emap, f, self_xy, pursuers_xy, cfg, empty)
    bad = _excluded(emap, exclude) if exclude else None
    base = f.region if not empty else (np.isfinite(f.t_e) & emap.free)
    if bad is not None:
        base = base & ~bad
    ok = base & (f.d_self >= cfg.min_goal_dist) & (f.d_self <= cfg.max_goal_dist)
    roomy = ok & (emap.clearance >= cfg.goal_clearance)
    if roomy.any():
        ok = roomy
    if not ok.any():
        ok = base & (f.d_self > 0.5)
    if not ok.any():
        return Choice(None, -math.inf, empty, False, -math.inf)
    masked = np.where(ok, sc, -np.inf)
    k = int(np.argmax(masked))
    iy, ix = divmod(k, emap.w)
    best_s = float(masked[iy, ix])
    best = emap.world(iy, ix)
    if incumbent is not None:
        jy, jx = emap.cell(incumbent)
        inc_ok = (base[jy, jx] and f.d_self[jy, jx] > cfg.reached_dist
                  and math.isfinite(f.d_self[jy, jx]))
        if inc_ok and best_s < float(sc[jy, jx]) + cfg.hysteresis:
            return Choice(tuple(incumbent), float(sc[jy, jx]), empty, False,
                          float(mrg[jy, jx]))
    switched = incumbent is None or math.hypot(
        best[0] - incumbent[0], best[1] - incumbent[1]) > 1e-6
    return Choice(best, best_s, empty, switched, float(mrg[iy, ix]))
