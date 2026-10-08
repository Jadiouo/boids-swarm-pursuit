"""SmartEvader (`evader:=smart`): a pure-Python evader that finds ways out.

ReactiveEvader scores 24 straight rays and treats walls as a soft penalty, so
under pressure it slides along walls and pins itself on obstacles. This brain
plans on the same grid model the Nav2 evader's goal scorer uses
(`EscapeMap`), but needs no Nav2 and no other process.

Two layers
----------
Strategy (low rate, `decision_hz`, amortised over several calls so no single
`compute` pays for more than one geodesic field):
    sample candidate escape points in free space; score each by
      * lead   - how many seconds sooner the target gets there than the
                 nearest pursuer (geodesic distance / speed),
      * open   - clearance from walls/obstacles,
      * away   - distance from the pursuer centroid,
      * dead   - penalty for corners / pockets (low free-area fraction),
    keep the incumbent goal unless a rival beats it by `hysteresis`.
Control (every call, cheap):
    walk down the geodesic distance-to-goal field (a path, not a straight
    line), add repulsion from close pursuers, then filter against the nearest
    wall/obstacle: the into-surface component is dropped and, when close, the
    output slides along the tangent toward the open side. If the heading
    change is large the vehicle (omega-limited) is pointed at the more open
    side instead of an arbitrary one.
Unstick: if the evader barely moves, or stays on a boundary, for `stuck_steps`
    it commits for `escape_hold` steps to the most open direction.

All tunables live in `SmartConfig`; `SmartConfig.from_params` reads
`smart_<field>` keys (the target_controller feeds ROS parameters through it).
"""

import math
import random
from collections import deque
from dataclasses import dataclass, fields

import numpy as np
from scipy import ndimage as ndi

from ..geometry import unit, wrap_angle
from . import dominance as dom
from .escape_map import EscapeMap
from .evasion import ReactiveEvader


@dataclass
class SmartConfig:
    # strategy layer
    decision_hz: float = 5.0        # goal re-evaluation rate
    control_rate_hz: float = 30.0   # calls per second (sets the step clock)
    candidates: int = 80            # fixed pool of escape points scored per decision
    selector: str = 'dominance'     # dominance | sampled (A/B: the old scorer)
    dom_margin: float = 0.6         # s the target must beat every pursuer by
    w_dom: float = 0.8              # rollout cost: arriving where a pursuer is first
    w_lead: float = 1.0
    w_open: float = 0.0             # goal openness (rollouts use w_clear)
    w_away: float = 0.4
    w_dead: float = 0.3
    hysteresis: float = 0.15        # rival must beat the incumbent by this
    target_speed: float = 3.6       # m/s the target flees at (sprint)
    pursuer_speed: float = 2.0      # m/s of the nearest pursuer
    lead_cap_s: float = 4.0         # seconds of lead that count as "safe"
    open_cap: float = 3.0           # m of clearance counted as fully open
    min_goal_dist: float = 2.0      # candidates closer than this are skipped
    max_goal_dist: float = 14.0
    urgent_shift: float = 2.0       # pursuer-centroid move forcing a re-plan
    omega_max: float = 1.2          # rad/s, = target_omega_max (sim cap)
    cruise_speed: float = 2.0       # m/s when no pursuer inside panic_distance
    panic_distance: float = 6.0
    # stamina model, mirrors the sim (sprinting = a pursuer inside
    # panic_distance; an empty tank drops the cap to cruise_speed)
    use_stamina: int = 1
    stamina_drain: float = 0.25
    stamina_regen: float = 0.35
    # grid
    resolution: float = 0.2
    robot_radius: float = 0.35      # m kept from surfaces in the planner
    rebuild_shift: float = 0.3      # bounds move that rebuilds the grid
    # control layer
    lookahead: float = 1.4          # m along the geodesic path
    rollouts: int = 15              # turn-rate commands tried per step
    horizon: float = 2.0            # s each rollout is simulated
    chord_t: float = 1.0            # s ahead the output direction points at
    danger_radius: float = 2.8      # m: pursuer closeness that costs
    w_progress: float = 1.0
    w_danger: float = 1.5
    w_clear: float = 0.3
    w_hit: float = 4.0
    # enclosure: capture needs >=3 pursuers within d_capture whose hull
    # contains the target, i.e. no pursuer-free half-plane. The rollout cost
    # rises as the predicted pursuer-free arc around the target shrinks.
    w_enclose: float = 3.0
    enclose_radius: float = 4.0
    enclose_gap_safe: float = 3.7   # rad of empty arc counted as safe
    w_switch: float = 0.08
    rep_radius: float = 3.5
    rep_gain: float = 1.2
    wall_margin: float = 0.9        # surface filter starts here (m)
    tangent_zone: float = 0.45      # forced slide inside this distance (m)
    turn_flip: float = 2.3          # rad: pick the open side beyond this
    # unstick
    stuck_steps: int = 36
    stuck_disp: float = 0.45        # m moved over the window that counts
    escape_hold: int = 30
    seed: int = 11

    @classmethod
    def from_params(cls, params):
        """Build from a mapping with `smart_<field>` keys (extra keys and
        missing ones are fine; values are coerced to the field's type)."""
        kw = {}
        for f in fields(cls):
            key = 'smart_' + f.name
            if params and key in params and params[key] is not None:
                kw[f.name] = type(f.default)(params[key])
        return cls(**kw)


def _surfaces(x, y, bmin, bmax, obstacles, reach):
    """Nearby boundaries as (distance, nx, ny): walls and obstacle discs
    whose surface lies within `reach` of the point; n points away from it."""
    out = []
    for s, nx, ny in ((x - bmin[0], 1.0, 0.0), (bmax[0] - x, -1.0, 0.0),
                      (y - bmin[1], 0.0, 1.0), (bmax[1] - y, 0.0, -1.0)):
        if s < reach:
            out.append((max(s, 0.0), nx, ny))
    for cx, cy, r in obstacles:
        dx, dy = x - cx, y - cy
        d = math.hypot(dx, dy)
        s = d - r
        if s < reach:
            if d < 1e-9:
                dx, dy, d = 1.0, 0.0, 1.0
            out.append((max(s, 0.0), dx / d, dy / d))
    return out


class SmartEvader:

    def __init__(self, bounds_min, bounds_max, obstacles=(), config=None):
        self.cfg = config or SmartConfig()
        self.bmin, self.bmax = bounds_min, bounds_max
        self.obstacles = [tuple(o) for o in obstacles]
        self._geo = ReactiveEvader(bounds_min, bounds_max, self.obstacles)
        self._rng = random.Random(self.cfg.seed)
        self._built_for = None
        self.emap = None
        self._build_map()

        self._period = max(1, round(self.cfg.control_rate_hz
                                    / self.cfg.decision_hz))
        self._since = self._period      # decide at once on the first call
        self._goal_dirty = False
        self._phase = None              # None idle, else 0..3
        self._snap = None               # inputs frozen at phase 0
        self._d_purs = None
        self._d_self = None
        self._t_p = None                # pursuer arrival-time field (s)
        self.region = None              # dominance region mask of the last decision
        self.dom_empty = False          # ... was empty (break-out goal)
        self.goal = None                # (x, y) world
        self._d_goal = None
        self._goal_score = -math.inf
        self._centroid = None
        self._hist = deque(maxlen=self.cfg.stuck_steps)
        self._near_wall = deque(maxlen=self.cfg.stuck_steps)
        self.escaping = False
        self._esc_left = 0
        self._esc_dir = None
        self.mode = 'flee'
        self._w_prev = 0.0
        self.stamina = 1.0

    # ------------------------------------------------------------ the map
    def _build_map(self):
        c = self.cfg
        self.emap = EscapeMap(self.bmin, self.bmax, self.obstacles,
                              resolution=c.resolution,
                              robot_radius=c.robot_radius)
        e = self.emap
        # share of floor within ~3 m: ~1 open, ~0.5 along a wall, ~0.25 in a
        # corner, small in a pocket. Static, so computed once per map.
        win = max(3, int(round(3.0 / c.resolution)))
        occ = (~e.free).astype(float)
        e.room = 1.0 - ndi.uniform_filter(occ, size=win, mode='constant',
                                          cval=1.0)
        self._built_for = (tuple(self.bmin), tuple(self.bmax))
        self._flat_free = np.flatnonzero(e.free.ravel())
        # Fixed candidate pool (same inputs -> same decision, no sampling
        # noise to fight the hysteresis). Shuffled once with the seeded rng.
        pool = self._flat_free.copy()
        np.random.RandomState(c.seed).shuffle(pool)
        self._pool = pool[:max(c.candidates * 6, c.candidates)]

    def _maybe_rebuild(self):
        (a, b), (c, d) = self._built_for
        if (abs(self.bmin[0] - a) > self.cfg.rebuild_shift
                or abs(self.bmin[1] - b) > self.cfg.rebuild_shift
                or abs(self.bmax[0] - c) > self.cfg.rebuild_shift
                or abs(self.bmax[1] - d) > self.cfg.rebuild_shift):
            self._build_map()
            self.goal = self._d_goal = None
            self._t_p = self.region = None
            self._phase = None
            self._since = self._period
        self._geo.bmin, self._geo.bmax = self.bmin, self.bmax

    # --------------------------------------------------- strategy (amortised)
    def _centroid_of(self, pursuers):
        n = len(pursuers)
        return (sum(p[0] for p in pursuers) / n,
                sum(p[1] for p in pursuers) / n)

    def _strategy_step(self, xy, theta, pursuers):
        """Advance the decision pipeline by (at most) one phase."""
        self._since += 1
        if self._phase is None:
            urgent = (self._centroid is not None and pursuers
                      and math.dist(self._centroid,
                                    self._centroid_of(pursuers))
                      > self.cfg.urgent_shift)
            if self._since < self._period and not urgent \
                    and self.goal is not None:
                return
            self._phase = 0
        e = self.emap
        if self._phase == 0:                       # pursuer geodesic field
            self._snap = (xy, list(pursuers), theta)
            self._d_purs = (e.geodesic([(p[0], p[1]) for p in pursuers])
                            if pursuers else None)
            self._phase = 1
        elif self._phase == 1:                     # own geodesic field
            self._d_self = e.geodesic([xy])
            self._phase = 2
        elif self._phase == 2:                     # score candidates
            self._select_goal(xy)
            self._phase = 3 if self._goal_dirty else None
            if self._phase is None:
                self._finish_decision(pursuers)
        else:                                      # goal field, only on change
            self._d_goal = e.geodesic([self.goal])
            self._goal_dirty = False
            self._phase = None
            self._finish_decision(pursuers)

    def _finish_decision(self, pursuers):
        self._since = 0
        self._centroid = self._centroid_of(pursuers) if pursuers else None

    def _score(self, cells, cent, theta=None):
        """Vectorised candidate score over flat cell indices."""
        c, e = self.cfg, self.emap
        t_self = self._d_self.ravel()[cells] / c.target_speed
        if theta is not None:        # turning toward a candidate takes time
            iy0, ix0 = np.divmod(cells, e.w)
            ang = np.arctan2(e.origin[1] + (iy0 + 0.5) * e.res - self._snap[0][1],
                             e.origin[0] + (ix0 + 0.5) * e.res - self._snap[0][0])
            dth = np.abs(np.arctan2(np.sin(ang - theta), np.cos(ang - theta)))
            t_self = t_self + dth / c.omega_max
        if self._d_purs is None:
            lead = np.full(len(cells), c.lead_cap_s)
        else:
            dp = self._d_purs.ravel()[cells]
            lead = np.where(np.isfinite(dp), dp / c.pursuer_speed - t_self,
                            c.lead_cap_s)
        lead_n = np.clip(lead, -c.lead_cap_s, c.lead_cap_s) / c.lead_cap_s
        open_n = np.minimum(e.clearance.ravel()[cells], c.open_cap) / c.open_cap
        room = e.room.ravel()[cells]
        dead = np.clip((0.7 - room) / 0.45, 0.0, 1.0)
        iy, ix = np.divmod(cells, e.w)
        wx = e.origin[0] + (ix + 0.5) * e.res
        wy = e.origin[1] + (iy + 0.5) * e.res
        if cent is None:
            away = np.full(len(cells), 0.5)
        else:
            away = np.minimum(np.hypot(wx - cent[0], wy - cent[1]), 15.0) / 15.0
        return (c.w_lead * lead_n + c.w_open * open_n + c.w_away * away
                - c.w_dead * dead), (wx, wy), t_self

    def _dom_cfg(self):
        c = self.cfg
        v = c.target_speed
        if c.use_stamina:     # an empty tank means cruise speed
            v = c.cruise_speed + (c.target_speed - c.cruise_speed) \
                * min(1.0, self.stamina / 0.3)
        return dom.DominanceConfig(
            v_self=max(v, 0.1), v_purs=c.pursuer_speed, omega=c.omega_max,
            margin_s=c.dom_margin, hysteresis=c.hysteresis,
            min_goal_dist=c.min_goal_dist, max_goal_dist=c.max_goal_dist + 2.0)

    def _select_goal_dominance(self, xy):
        """Goal from the dominance region (see behaviors/dominance.py); the
        two geodesic fields were computed in earlier phases."""
        e = self.emap
        dc = self._dom_cfg()
        purs = [(p[0], p[1]) for p in self._snap[1]]
        if self._d_purs is None:
            d_p = np.full((e.h, e.w), np.inf)
        else:
            d_p = self._d_purs
        t_p = d_p / dc.v_purs
        t_e = dom.turn_time(e, self._d_self, xy, self._snap[2], dc)
        f = dom.assemble_fields(e, xy, self._d_self, t_e, d_p, t_p, dc)
        ch = dom.select_goal(e, f, xy, purs, self.goal, dc)
        self._t_p = t_p
        self.region, self.dom_empty = f.region, ch.empty
        if ch.goal is None:
            self._goal_dirty = False
            return
        self._goal_score = ch.score
        if ch.switched:
            self.goal = ch.goal
            self._goal_dirty = True
        else:
            self._goal_dirty = False

    def _select_goal(self, xy):
        if self.cfg.selector == 'dominance':
            return self._select_goal_dominance(xy)
        c, e = self.cfg, self.emap
        pursuers = self._snap[1]
        cent = self._centroid_of(pursuers) if pursuers else None
        d_self = self._d_self.ravel()
        pool = self._pool
        ok = pool[(d_self[pool] >= c.min_goal_dist)
                  & (d_self[pool] <= c.max_goal_dist)][:c.candidates]
        if len(ok) == 0:
            flat = self._flat_free
            ok = flat[np.isfinite(d_self[flat])][:c.candidates]
        cells = ok
        keep_cur = False
        if self.goal is not None and self._d_goal is not None:
            iy, ix = e.cell(self.goal)
            ci = iy * e.w + ix
            if np.isfinite(d_self[ci]) and d_self[ci] > 0.8:
                cells = np.append(cells, ci)
                keep_cur = True
        if len(cells) == 0:
            self._goal_dirty = False
            return
        sc, (wx, wy), _ = self._score(cells, cent, self._snap[2])
        if keep_cur:
            sc[-1] += c.hysteresis
        b = int(np.argmax(sc))
        if keep_cur and b == len(cells) - 1:
            self._goal_score = float(sc[b])
            self._goal_dirty = False
            return
        self.goal = (float(wx[b]), float(wy[b]))
        self._goal_score = float(sc[b])
        self._goal_dirty = True

    # ------------------------------------------------------------- control
    def _path_dir(self, xy):
        """Direction of the next point on the descent of the goal field."""
        e = self.emap
        if self._d_goal is None:
            return None
        start = e.nearest_free(xy, reach=1.0)
        if start is None:
            return None
        iy, ix = e.cell(start)
        d = self._d_goal
        if not math.isfinite(d[iy, ix]):
            return None
        for _ in range(max(1, int(self.cfg.lookahead / e.res))):
            best, bv = None, d[iy, ix]
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    ny, nx = iy + dy, ix + dx
                    if (dy or dx) and 0 <= ny < e.h and 0 <= nx < e.w \
                            and d[ny, nx] < bv:
                        best, bv = (ny, nx), d[ny, nx]
            if best is None:
                break
            iy, ix = best
        tx, ty = e.world(iy, ix)
        v = unit(tx - xy[0], ty - xy[1], None)
        return v

    def _repulsion(self, xy, pursuers):
        c = self.cfg
        rx = ry = 0.0
        for (px, py, *_r) in pursuers:
            dx, dy = xy[0] - px, xy[1] - py
            d = math.hypot(dx, dy)
            if 1e-6 < d < c.rep_radius:
                w = (c.rep_radius - d) / c.rep_radius / d
                rx, ry = rx + dx * w, ry + dy * w
        return rx, ry

    def _open_score(self, x, y, pursuers):
        """How good a spot is to flee through (clearance + pursuer gap)."""
        sf = _surfaces(x, y, self.bmin, self.bmax, self.obstacles, 5.0)
        if any(s < 0.3 for s, _, _ in sf):
            return -1.0
        near = min((math.hypot(x - p[0], y - p[1]) for p in pursuers),
                   default=10.0)
        clear = min((s for s, _, _ in sf), default=5.0)
        return min(clear, 3.0) / 3.0 + 0.5 * min(near, 8.0) / 8.0

    def _filter_surfaces(self, xy, u, pursuers, forced):
        """Drop the into-surface component; slide toward the open side."""
        c = self.cfg
        x, y = xy
        near = _surfaces(x, y, self.bmin, self.bmax, self.obstacles,
                         c.wall_margin)
        if not near:
            return u
        ux, uy = u
        closest = min(s for s, _, _ in near)
        for s, nx, ny in near:
            un = ux * nx + uy * ny
            if un < 0.0:
                ux, uy = ux - un * nx, uy - un * ny
        # push out a little in proportion to how deep we are
        for s, nx, ny in near:
            k = 0.5 * (c.wall_margin - s) / c.wall_margin
            ux, uy = ux + k * nx, uy + k * ny
        if closest < c.tangent_zone and not forced:
            s0, nx, ny = min(near)
            tx, ty = -ny, nx
            ut = ux * tx + uy * ty
            if abs(ut) < 0.6:
                a = self._open_score(x + tx * 1.5, y + ty * 1.5, pursuers)
                b = self._open_score(x - tx * 1.5, y - ty * 1.5, pursuers)
                sign = (1.0 if a >= b else -1.0) if abs(ut) < 0.15 \
                    else (1.0 if ut > 0 else -1.0)
                ux, uy = ux - ut * tx, uy - ut * ty
                ux, uy = ux + sign * 0.8 * tx, uy + sign * 0.8 * ty
        out = unit(ux, uy, None)
        return out if out is not None else u

    def _turn_aware(self, xy, theta, u):
        """If u is far from the heading, prefer turning toward the open side
        (omega is capped, so a half-turn takes long either way)."""
        if abs(wrap_angle(math.atan2(u[1], u[0]) - theta)) <= self.cfg.turn_flip:
            return u
        best, bs = None, -1.0
        for sgn in (1.0, -1.0):
            ang = theta + sgn * math.pi / 2
            d = (math.cos(ang), math.sin(ang))
            s = min(self._geo._clearance(xy[0], xy[1], d[0], d[1]), 5.0)
            if s > bs:
                best, bs = d, s
        v = unit(0.4 * u[0] + 0.6 * best[0], 0.4 * u[1] + 0.6 * best[1], u)
        return v

    # ----------------------------------------------- rollout (heading aware)
    def _speed_now(self, near):
        """Speed the controller will run at this step (and tracks the
        sim's stamina: it drains while sprinting, an empty tank means
        cruise speed)."""
        c = self.cfg
        sprint = near < c.panic_distance
        if c.use_stamina:
            dt = 1.0 / c.control_rate_hz
            if sprint and self.stamina > 0.0:
                self.stamina = max(0.0, self.stamina - c.stamina_drain * dt)
            else:
                self.stamina = min(1.0, self.stamina + c.stamina_regen * dt)
            if self.stamina <= 1e-6:
                return c.cruise_speed
        return c.target_speed if sprint else c.cruise_speed

    def _enclosure(self, px, py, pos, n_t):
        """Per-rollout enclosure risk in [0, 1]: at a few instants, how
        small the widest pursuer-free arc around the predicted position is
        (12 sectors; fewer than 3 pursuers nearby = no risk)."""
        c = self.cfg
        ts = np.arange(3, n_t, 4)                      # sampled instants
        dx = pos[ts][:, None, :, 0] - px[:, ts].T[:, :, None]   # (S,M,P)
        dy = pos[ts][:, None, :, 1] - py[:, ts].T[:, :, None]
        near = np.hypot(dx, dy) < c.enclose_radius
        sec = (np.floor((np.arctan2(dy, dx) % (2 * np.pi))
                        / (np.pi / 6)).astype(int)) % 12
        occ = np.zeros(dx.shape[:2] + (12,), dtype=bool)
        for k in range(12):
            occ[..., k] = ((sec == k) & near).any(axis=2)
        e2 = np.concatenate([~occ, ~occ], axis=2)
        run = np.zeros(dx.shape[:2])
        best = np.zeros(dx.shape[:2])
        for k in range(24):
            run = (run + 1.0) * e2[..., k]
            best = np.maximum(best, run)
        gap = np.minimum(best, 12.0) * (np.pi / 6)
        risk = np.clip((c.enclose_gap_safe - gap) / (c.enclose_gap_safe - np.pi / 2),
                       0.0, 1.0)
        risk = np.where(near.sum(axis=2) >= 3, risk, 0.0)
        return risk.max(axis=0) + 0.5 * risk.mean(axis=0)

    def _rollout_dir(self, xy, theta, pursuers):
        """Try `rollouts` constant-turn-rate arcs at the speed the
        controller will use, score each (progress along the geodesic field,
        pursuer closeness, clearance, collision, turn changes) and return
        the direction toward where the best arc is `chord_t` s ahead, plus
        the chosen turn rate. Respects the omega cap by construction."""
        c, e = self.cfg, self.emap
        near = min((math.hypot(p[0] - xy[0], p[1] - xy[1]) for p in pursuers),
                   default=math.inf)
        v = self._speed_now(near)
        n_t = max(4, int(c.horizon / 0.1))
        t = np.arange(1, n_t + 1) * (c.horizon / n_t)
        w = np.linspace(-c.omega_max, c.omega_max, c.rollouts)
        wz = np.where(np.abs(w) < 1e-4, 1e-4, w)[:, None]
        th = theta + wz * t[None, :]
        px = xy[0] + v / wz * (np.sin(th) - math.sin(theta))
        py = xy[1] - v / wz * (np.cos(th) - math.cos(theta))
        ix = np.clip(np.floor((px - e.origin[0]) / e.res).astype(int), 0, e.w - 1)
        iy = np.clip(np.floor((py - e.origin[1]) / e.res).astype(int), 0, e.h - 1)
        inside = ((px > self.bmin[0]) & (px < self.bmax[0])
                  & (py > self.bmin[1]) & (py < self.bmax[1]))
        free = e.free[iy, ix] & inside
        # collision: first non-free sample
        hit = ~free
        any_hit = hit.any(axis=1)
        first = np.where(any_hit, hit.argmax(axis=1), n_t)
        cost = c.w_hit * (1.0 - first / n_t) * any_hit
        cost = cost + 1.0 * any_hit
        # progress toward the goal (or away from the pursuers if no plan)
        if self._d_goal is not None:
            dg = self._d_goal[iy, ix]
            dg = np.where(np.isfinite(dg), dg, 50.0)
            iy0, ix0 = e.cell(xy)
            d0 = self._d_goal[iy0, ix0]
            d0 = d0 if math.isfinite(d0) else float(dg.max())
            prog = (dg[:, -1] - d0) / (v * c.horizon)
            prog = 0.5 * prog + 0.5 * (dg[:, n_t // 2] - d0) / (0.5 * v * c.horizon)
        elif pursuers:
            cx, cy = self._centroid_of(pursuers)
            ax, ay = unit(xy[0] - cx, xy[1] - cy, (1.0, 0.0))
            prog = -((px[:, -1] - xy[0]) * ax + (py[:, -1] - xy[1]) * ay) \
                / (v * c.horizon)
        else:
            prog = np.zeros(len(w))
        cost = cost + c.w_progress * prog
        # pursuers close in on where we are now at their own speed
        if pursuers:
            P = np.array([[p[0], p[1]] for p in pursuers])
            dirs = np.array([xy[0], xy[1]]) - P
            nd = np.maximum(np.linalg.norm(dirs, axis=1, keepdims=True), 1e-6)
            tp = np.minimum(t[:, None, None] * c.pursuer_speed,
                            nd[None, :, :] * 0.9)
            pos = P[None, :, :] + dirs[None, :, :] / nd[None, :, :] * tp  # (T,P,2)
            dx = px.T[:, :, None] - pos[:, None, :, 0]                    # (T,M,P)
            dy = py.T[:, :, None] - pos[:, None, :, 1]
            d = np.hypot(dx, dy)
            r = c.danger_radius
            dang = np.clip(1.0 - d / r, 0.0, 1.0)
            danger = dang.sum(axis=2)                                     # (T,M)
            cost = cost + c.w_danger * (danger.max(axis=0)
                                        + 0.5 * danger.mean(axis=0))
        if pursuers and c.w_enclose > 0.0:
            cost = cost + c.w_enclose * self._enclosure(px, py, pos, n_t)
        if c.selector == 'dominance' and c.w_dom > 0.0 \
                and self._t_p is not None and pursuers:
            # arriving at a spot a pursuer can reach first (+ margin) is
            # outside the dominance region
            viol = np.clip((t[None, :] + c.dom_margin - self._t_p[iy, ix])
                           / 1.0, 0.0, 1.0)
            cost = cost + c.w_dom * (viol.max(axis=1) + 0.5 * viol.mean(axis=1))
        # stay off surfaces
        clr = e.clearance[iy, ix]
        cost = cost + c.w_clear * np.clip(1.0 - clr / 0.9, 0.0, 1.0).mean(axis=1)
        cost = cost + c.w_switch * np.abs(w - self._w_prev) / c.omega_max
        b = int(np.argmin(cost))
        self._w_prev = float(w[b])
        k = min(n_t - 1, max(0, int(c.chord_t / (c.horizon / n_t)) - 1))
        d = unit(px[b, k] - xy[0], py[b, k] - xy[1], None)
        return d, float(cost[b])

    # -------------------------------------------------------------- unstick
    def _update_stuck(self, xy):
        c = self.cfg
        self._hist.append(xy)
        s = min((s for s, _, _ in _surfaces(xy[0], xy[1], self.bmin,
                                            self.bmax, self.obstacles, 1.0)),
                default=1.0)
        self._near_wall.append(s < 0.3)
        if len(self._hist) < self._hist.maxlen or self.escaping:
            return False
        disp = math.dist(self._hist[0], self._hist[-1])
        hugging = sum(self._near_wall) >= 0.9 * len(self._near_wall)
        return disp < c.stuck_disp or (hugging and disp < 3.0 * c.stuck_disp)

    def _escape_dir(self, xy, pursuers):
        best, bs = None, -math.inf
        for k in range(16):
            a = 2 * math.pi * k / 16
            d = (math.cos(a), math.sin(a))
            for dist in (1.5, 3.0):
                px, py = xy[0] + d[0] * dist, xy[1] + d[1] * dist
                if not (self.bmin[0] < px < self.bmax[0]
                        and self.bmin[1] < py < self.bmax[1]):
                    break
            else:
                s = self._open_score(xy[0] + d[0] * 2.0, xy[1] + d[1] * 2.0,
                                     pursuers)
                s += 0.15 * min(self._geo._clearance(xy[0], xy[1], *d), 4.0)
                if s > bs:
                    best, bs = d, s
        return best

    # ---------------------------------------------------------------- API
    def compute(self, self_xy, self_theta, pursuers):
        """pursuers: list of (x, y, theta). Returns a unit vector."""
        self._maybe_rebuild()
        xy = (float(self_xy[0]), float(self_xy[1]))
        self._strategy_step(xy, self_theta, pursuers)

        if self._update_stuck(xy):
            self.escaping = True
            self._esc_left = self.cfg.escape_hold
            self._esc_dir = None
            self.goal = self._d_goal = None      # decide afresh afterwards
            self._since = self._period
            self._phase = None
        forced = False
        u = None
        if self.escaping:
            if self._esc_dir is None or self._esc_left % 10 == 0:
                self._esc_dir = self._escape_dir(xy, pursuers)
            u = self._esc_dir
            forced = True
            self.mode = 'escape'
            self._esc_left -= 1
            if self._esc_left <= 0:
                self.escaping = False
                self._hist.clear()
                self._near_wall.clear()
        else:
            self.mode = 'flee'
            u, _cost = self._rollout_dir(xy, self_theta, pursuers)
        if u is None:
            u = (math.cos(self_theta), math.sin(self_theta))
        u = self._filter_surfaces(xy, u, pursuers, forced)
        return unit(u[0], u[1], (math.cos(self_theta), math.sin(self_theta)))
