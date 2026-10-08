"""Nav2Evader (SDD v3 §6.4, M7): obstacle-aware fleeing via Nav2.

Architecture (one process, `target_controller`):

    escape-goal sampler --ComputePathToPose--> planner_server (global costmap)
            |                                        |
            +------------FollowPath----------> controller_server (MPPI,
                                                 local costmap + boid cloud)
                                                        |  /target/nav2_cmd_vel
    ReactiveEvader (30 Hz) ----------------+            v
                                        blend by nearest-boid distance
                                                        |
                                                 /target/cmd_vel

Everything decision-related is a pure function in this module (testable
without ROS); the `Nav2Evader` class only wires them to the action
clients. Mode is observable on `/target/evader_status` (JSON).
"""

import json
import math
import os
import random
from dataclasses import dataclass

from ..geometry import clamp, to_twist, unit, wrap_angle
from . import dominance as dom
from .evasion import ReactiveEvader


@dataclass
class EscapeConfig:
    n_candidates: int = 64
    wall_margin: float = 1.2        # candidates stay this far inside walls
    obstacle_margin: float = 0.8    # ... and this far outside obstacle edges
    w_boid: float = 1.0             # distance from nearest boid
    w_open: float = 0.5             # openness (wall / obstacle clearance)
    w_cross: float = 1.2            # penalty for passing boids on the way
    w_travel: float = 0.15          # mild preference for nearer goals
    boid_cap: float = 12.0
    open_cap: float = 4.0
    cross_radius: float = 2.5
    keep_margin: float = 0.15       # hysteresis: switch only if clearly better
    reached_dist: float = 1.0       # incumbent counts as reached inside this


# ------------------------------------------------------------ pure helpers
def _clearance(pt, obstacles, bmin, bmax):
    """Distance from pt to the nearest wall or obstacle surface."""
    x, y = pt
    d = min(x - bmin[0], bmax[0] - x, y - bmin[1], bmax[1] - y)
    for (cx, cy, r) in obstacles:
        d = min(d, math.hypot(x - cx, y - cy) - r)
    return d


def _seg_dist(p, a, b):
    """(distance from p to segment a-b, raw projection parameter)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    l2 = dx * dx + dy * dy
    if l2 < 1e-12:
        return math.hypot(p[0] - a[0], p[1] - a[1]), 0.0
    t_raw = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2
    t = clamp(t_raw, 0.0, 1.0)
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy)), t_raw


def score_candidate(cand, self_xy, pursuers, obstacles, bmin, bmax,
                    cfg: EscapeConfig):
    """Higher is better; -inf for points that are not free space.

    score = w_boid * (dist to nearest boid) + w_open * (clearance)
            - w_cross * (boids the straight route would pass)
            - w_travel * (route length)
    each term normalised to roughly [0, 1].
    """
    x, y = cand
    if (x < bmin[0] + cfg.wall_margin or x > bmax[0] - cfg.wall_margin
            or y < bmin[1] + cfg.wall_margin or y > bmax[1] - cfg.wall_margin):
        return -math.inf
    for (cx, cy, r) in obstacles:
        if math.hypot(x - cx, y - cy) < r + cfg.obstacle_margin - 1e-9:
            return -math.inf
    d_boid = min((math.hypot(x - p[0], y - p[1]) for p in pursuers),
                 default=cfg.boid_cap)
    openness = clamp(_clearance(cand, obstacles, bmin, bmax), 0.0,
                     cfg.open_cap)
    cross = 0.0
    for p in pursuers:
        d, t_raw = _seg_dist((p[0], p[1]), self_xy, cand)
        if t_raw > 0.0 and d < cfg.cross_radius:    # boids behind us don't count
            cross += (cfg.cross_radius - d) / cfg.cross_radius
    span = math.hypot(bmax[0] - bmin[0], bmax[1] - bmin[1])
    travel = math.hypot(x - self_xy[0], y - self_xy[1]) / span
    return (cfg.w_boid * min(d_boid, cfg.boid_cap) / cfg.boid_cap
            + cfg.w_open * openness / cfg.open_cap
            - cfg.w_cross * min(cross, 2.0)
            - cfg.w_travel * travel)


def sample_candidates(rng, n, bmin, bmax, obstacles, cfg: EscapeConfig):
    """n uniformly sampled free-space points (rejection sampling)."""
    out = []
    lo_x, hi_x = bmin[0] + cfg.wall_margin, bmax[0] - cfg.wall_margin
    lo_y, hi_y = bmin[1] + cfg.wall_margin, bmax[1] - cfg.wall_margin
    guard = 0
    while len(out) < n and guard < n * 200:
        guard += 1
        p = (rng.uniform(lo_x, hi_x), rng.uniform(lo_y, hi_y))
        if all(math.hypot(p[0] - cx, p[1] - cy) >= r + cfg.obstacle_margin
               for (cx, cy, r) in obstacles):
            out.append(p)
    return out


def select_escape_goal(rng, self_xy, pursuers, obstacles, bmin, bmax,
                       incumbent, cfg: EscapeConfig, emap=None, blocked=None):
    """Best sampled goal, keeping the incumbent unless clearly beaten.

    Returns (goal, score). An incumbent that is reached (within
    cfg.reached_dist), no longer free, or blacklisted is discarded.

    emap: optional `EscapeMap`; adds the geodesic terms (unreachable ->
    rejected, arrival lead over the nearest pursuer, pocket penalty).
    blocked: optional predicate(goal) -> True for blacklisted goals.
    """
    ctx = emap.context(self_xy, [(p[0], p[1]) for p in pursuers]) \
        if emap is not None else None

    def full_score(c, pocket=False):
        s = score_candidate(c, self_xy, pursuers, obstacles, bmin, bmax, cfg)
        if ctx is not None and s > -math.inf:
            s += ctx.adjustment(c, pocket)
        return s

    cands = [c for c in sample_candidates(rng, cfg.n_candidates, bmin, bmax,
                                          obstacles, cfg)
             if not (blocked is not None and blocked(c))]
    scored = sorted(((full_score(c), c) for c in cands), reverse=True)
    if ctx is not None:           # costlier pocket check on the leaders only
        k = emap.cfg.top_k
        scored = sorted(((full_score(c, True), c) for _, c in scored[:k]
                         if _ > -math.inf), reverse=True)
    best, best_s = (scored[0][1], scored[0][0]) if scored else (None, -math.inf)
    if incumbent is not None and math.hypot(
            incumbent[0] - self_xy[0],
            incumbent[1] - self_xy[1]) > cfg.reached_dist \
            and not (blocked is not None and blocked(incumbent)):
        s_inc = full_score(incumbent, ctx is not None)
        if s_inc > -math.inf and (best is None
                                  or best_s < s_inc + cfg.keep_margin):
            return tuple(incumbent), s_inc
    if best is None:                       # fully blocked arena: stay put
        return tuple(self_xy), -math.inf
    return best, best_s


class GoalBlacklist:
    """Short-lived memory of goals that failed (plan failure / FollowPath
    abort). A goal within `radius` of a live entry is not tried again until
    the entry expires; a repeat failure at the same spot doubles its ttl."""

    def __init__(self, ttl=6.0, radius=1.0, max_entries=16):
        self.ttl, self.radius, self.max_entries = ttl, radius, max_entries
        self._e = []                       # [x, y, expires_at, strikes]

    def add(self, goal, now):
        for e in self._e:
            if math.hypot(e[0] - goal[0], e[1] - goal[1]) <= self.radius \
                    and e[2] > now:
                e[3] += 1
                e[2] = now + self.ttl * (2 ** (e[3] - 1))
                return
        self._e.append([goal[0], goal[1], now + self.ttl, 1])
        self._e.sort(key=lambda e: e[2])
        del self._e[:-self.max_entries]

    def active(self, now):
        self._e = [e for e in self._e if e[2] > now]
        return [(e[0], e[1]) for e in self._e]

    def blocked(self, goal, now):
        return any(math.hypot(x - goal[0], y - goal[1]) <= self.radius
                   for (x, y) in self.active(now))


class PlanTracker:
    """Sequence bookkeeping for the single in-flight ComputePathToPose.

    A timed-out request is closed (and must be cancelled by the caller), so
    its late result cannot be processed a second time; only the newest open
    request is ever accepted."""

    def __init__(self, timeout):
        self.timeout = timeout
        self.seq = 0
        self.pending = False
        self.t_sent = None

    def begin(self, now):
        self.seq += 1
        self.pending, self.t_sent = True, now
        return self.seq

    def accept(self, seq):
        return self.pending and seq == self.seq

    def close(self, seq):
        if seq == self.seq:
            self.pending = False

    def expire(self, now):
        """Seq of a request that just timed out (else None)."""
        if self.pending and now - self.t_sent > self.timeout:
            self.pending = False
            return self.seq
        return None


def should_request_plan(now, fallback_until, changed, stale, nav2_ok):
    """Send a ComputePathToPose only outside the fallback hold; inside it the
    evader is reactive and re-asking the same planner every tick just
    repeats the failure."""
    if now < fallback_until:
        return False
    return changed or stale or not nav2_ok


def plan_start(xy, bmin, bmax, obstacles, margin, reach=0.7):
    """Start pose to hand the planner: the robot's position moved to the
    nearest nearby point that is at least `margin` from every wall and
    obstacle surface (or, if none exists within `reach`, the best-clearance
    point found).

    The sim lets the target touch a wall or obstacle (centre 0.15 m away),
    but the planner's costmap (robot_radius > body) marks that as inscribed
    and navfn then refuses to plan. The evader often stands against a wall,
    so planning from a slightly inset start keeps Nav2 usable there; the
    controller still follows the path from the true pose. Narrow gaps
    (< 2 * margin between wall and obstacle) have no such point.
    """
    def clear(x, y):
        return _clearance((x, y), obstacles, bmin, bmax)

    if clear(xy[0], xy[1]) >= margin:
        return (xy[0], xy[1])
    best, best_c = (xy[0], xy[1]), clear(xy[0], xy[1])
    steps = 7
    for k in range(1, steps + 1):
        r = reach * k / steps
        for a in range(16):
            ang = 2.0 * math.pi * a / 16
            x, y = xy[0] + r * math.cos(ang), xy[1] + r * math.sin(ang)
            c = clear(x, y)
            if c >= margin:
                return (x, y)          # nearest acceptable point
            if c > best_c + 1e-9:
                best, best_c = (x, y), c
    return best


def smooth_start(path, pose_xy, theta, min_angle=0.5, step=0.2, radius=2.0):
    """Make a freshly planned path leave the robot along its heading.

    Pure pursuit (RPP) rotates in place when the carrot is more than
    `rotate_to_heading_min_angle` (1.2 rad) off the heading, i.e. every goal
    change that points away stops the target (v = 0) for a second or more.
    If the path's first ~1.5 m is more than `min_angle` off `theta`, its head
    is replaced by a turning circle of `radius` (about v/omega) that starts
    on `theta` and a straight tangent into the path further on (a
    Dubins-style CS curve), so the follower turns while moving. A path that
    is too short to be joined is extended along its last direction. Aligned
    paths are returned unchanged. path: [(x, y)]."""
    if len(path) < 3:
        return path
    px, py = pose_xy
    s_acc, look = 0.0, path[-1]
    for a, b in zip(path, path[1:]):
        s_acc += math.hypot(b[0] - a[0], b[1] - a[1])
        if s_acc >= 1.5:
            look = b
            break
    err = wrap_angle(math.atan2(look[1] - py, look[0] - px) - theta)
    if abs(err) < min_angle:
        return path
    sgn = 1.0 if err > 0 else -1.0                 # +1 = turn left (CCW)
    cx = px - sgn * radius * math.sin(theta)
    cy = py + sgn * radius * math.cos(theta)
    # join point: far enough along the path to be outside the circle
    L = clamp(2.0 * radius + abs(err), 2.5, 8.0)
    s_acc, j = 0.0, len(path) - 1
    for k in range(1, len(path)):
        s_acc += math.hypot(path[k][0] - path[k - 1][0],
                            path[k][1] - path[k - 1][1])
        if s_acc >= L:
            j = k
            break
    q = path[j]
    a, b = path[max(j - 1, 0)], path[min(j + 1, len(path) - 1)]
    tq = unit(b[0] - a[0], b[1] - a[1], (math.cos(theta), math.sin(theta)))
    ext = []
    while math.hypot(q[0] - cx, q[1] - cy) < 1.15 * radius and len(ext) < 40:
        q = (q[0] + step * tq[0], q[1] + step * tq[1])
        ext.append(q)
    d = math.hypot(q[0] - cx, q[1] - cy)
    phi = math.atan2(q[1] - cy, q[0] - cx)
    alpha = math.acos(min(1.0, radius / d))
    tpt = None
    for beta in (phi - sgn * alpha, phi + sgn * alpha):
        tx, ty = cx + radius * math.cos(beta), cy + radius * math.sin(beta)
        tan = (-sgn * math.sin(beta), sgn * math.cos(beta))
        if (q[0] - tx) * tan[0] + (q[1] - ty) * tan[1] > 0.0:
            tpt = (beta, tx, ty)
            break
    if tpt is None:
        return path
    beta, tx, ty = tpt
    a0 = math.atan2(py - cy, px - cx)
    sweep = (beta - a0) * sgn % (2.0 * math.pi)     # travelled turning way
    n = max(4, int(math.ceil(sweep * radius / step)))
    arc = [(cx + radius * math.cos(a0 + sgn * sweep * i / n),
            cy + radius * math.sin(a0 + sgn * sweep * i / n))
           for i in range(n)]
    seg = math.hypot(q[0] - tx, q[1] - ty)
    m = max(1, int(math.ceil(seg / step)))
    line = [(tx + (q[0] - tx) * i / m, ty + (q[1] - ty) * i / m)
            for i in range(m)]
    tail = list(path[j:]) if not ext else ext
    return arc + line + tail


def blend_weight(near, radius, r_min):
    """Reactive weight in [0, 1], linear in nearest-boid distance:
    0 at/after `radius` (pure Nav2) up to 1 at/inside `r_min` (pure reactive).
    """
    if radius <= r_min:
        return 1.0 if near < radius else 0.0
    return clamp((radius - near) / (radius - r_min), 0.0, 1.0)


def blend_cmd(theta, nav2_cmd, reactive_cmd, reactive_heading, w_reactive,
              speed_cap, w_max, kw=3.0, horizon=1.0, last_w=0.0):
    """Mix Nav2's (v, w) and the reactive evader in HEADING space.

    Averaging (v, w) directly lets opposite turns cancel (nav2 left +
    reactive right ~ straight ahead, i.e. into the boid). Instead each
    source becomes a velocity vector: Nav2's heading is the chord of its arc
    over `horizon` (theta + w*horizon/2), the reactive one is its desired
    heading. The vectors are mixed with weight `w_reactive` and converted
    back with the same non-holonomic law the node uses. If they nearly
    cancel (head-on conflict) the reactive heading wins: it is the safety
    behaviour. Endpoints return the pure sources unchanged.
    """
    if w_reactive <= 0.0:
        return tuple(nav2_cmd)
    if w_reactive >= 1.0:
        return tuple(reactive_cmd)
    a = 1.0 - w_reactive
    v_n, w_n = nav2_cmd
    h_n = theta + clamp(w_n, -w_max, w_max) * horizon / 2.0
    v_r = reactive_cmd[0]
    h_r = reactive_heading
    vx = a * v_n * math.cos(h_n) + w_reactive * v_r * math.cos(h_r)
    vy = a * v_n * math.sin(h_n) + w_reactive * v_r * math.sin(h_r)
    speed = clamp(a * v_n + w_reactive * v_r, 0.0, speed_cap)
    if math.hypot(vx, vy) < 0.3 * max(a * v_n + w_reactive * v_r, 1e-9):
        heading = h_r                       # cancelling vectors
    else:
        heading = math.atan2(vy, vx)
    return to_twist((math.cos(heading) * speed, math.sin(heading) * speed),
                    theta, kv=1.0, kw=kw, v_min=0.4 * speed, v_max=speed,
                    w_max=w_max, last_w=last_w)


def mode_for(w_reactive, nav2_ok):
    if not nav2_ok or w_reactive >= 1.0:
        return 'reactive'
    return 'nav2' if w_reactive <= 0.0 else 'blend'


# ----------------------------------------------------------- ROS adapter
class Nav2Evader:
    """Brain with the same `compute(self_xy, theta, pursuers)` interface as
    the other evaders. `compute` returns the reactive direction (and runs
    the goal loop); the node then calls `blend_twist` on the resulting
    twist to mix in the Nav2 controller's command.

    `node=None` yields a Nav2-less instance (always reactive) for tests.
    """

    NAV2_CMD_TOPIC = '/target/nav2_cmd_vel'
    STATUS_TOPIC = '/target/evader_status'

    def __init__(self, bounds_min, bounds_max, obstacles=(), node=None,
                 cfg: EscapeConfig = None, body_radius=0.15):
        self.reactive = ReactiveEvader(bounds_min, bounds_max, obstacles)
        self.obstacles = list(obstacles)
        self.cfg = cfg or EscapeConfig()
        self.node = node
        self.body_radius = body_radius
        self.emap = None
        self._emap_key = None
        self.mode = 'reactive'
        self.stats = {'plan_requests': 0, 'plan_ok': 0, 'plan_fail': 0,
                      'plan_timeout': 0, 'late_dropped': 0, 'blacklisted': 0,
                      'follow_abort': 0, 'follow_done': 0, 'goals': 0,
                      'ticks_nav2': 0, 'ticks_blend': 0, 'ticks_reactive': 0,
                      'ticks_unavailable': 0}
        self._self = None
        self._theta = 0.0
        self._pursuers = []
        self._mode_time = {'nav2': 0.0, 'blend': 0.0, 'reactive': 0.0}
        self._last_tick_t = None
        self.goal = None
        self.dom_empty = False
        self._rng = random.Random(self._param('seed', 7))
        self._nav2_cmd, self._nav2_cmd_t = None, -1e9
        self._fallback_until = -1e9
        self.blacklist = GoalBlacklist(
            ttl=self._param('blacklist_ttl'),
            radius=self._param('blacklist_radius'))
        self.tracker = PlanTracker(self._param('plan_timeout'))
        self._plan_gh = None
        self._follow_seq = 0
        self._last_plan_t = -1e9
        self._last_status_t = -1e9
        self._plan_goal = None
        self._follow_goal = None
        if node is not None:
            self._setup_ros()

    # bounds are pushed in live by the node (shrinking arena)
    @property
    def bmin(self):
        return self.reactive.bmin

    @bmin.setter
    def bmin(self, v):
        self.reactive.bmin = v

    @property
    def bmax(self):
        return self.reactive.bmax

    @bmax.setter
    def bmax(self, v):
        self.reactive.bmax = v

    DEFAULTS = {'goal_period': 0.75, 'replan_period': 2.0,
                'reactive_radius': 4.0, 'reactive_min': 1.5,
                'cmd_timeout': 0.3, 'fallback_hold': 1.0,
                'plan_timeout': 2.0, 'blacklist_ttl': 6.0,
                'blacklist_radius': 1.0, 'map_resolution': 0.1, 'seed': 7,
                'selector': 'dominance', 'dom_margin': 0.6, 'smooth_start': 1,
                'goal_clearance': 0.9, 'planner_radius': 0.3}

    @classmethod
    def declare_params(cls, node):
        """Declare every nav2_<k> in DEFAULTS the node has not declared yet
        (rclpy rejects/ignores undeclared launch parameters). `seed` is the
        node-wide parameter, not nav2_seed."""
        for k, v in cls.DEFAULTS.items():
            if k == 'seed':
                continue
            key = f'nav2_{k}'
            if not node.has_parameter(key):
                node.declare_parameter(key, v)

    def _param(self, name, default=None):
        # tuning / A-B hook (tools/evader_compare.py): a JSON object of
        # nav2_<name> overrides in the environment wins over node params
        extra = os.environ.get('NAV2_PARAMS_JSON')
        if extra:
            ov = json.loads(extra)
            if f'nav2_{name}' in ov:
                return ov[f'nav2_{name}']
        if self.node is not None:
            key = name if name == 'seed' else f'nav2_{name}'
            try:
                return self.node.get_parameter(key).value
            except Exception:           # undeclared -> default
                pass
        return self.DEFAULTS.get(name, default)

    # -- wiring ------------------------------------------------------------
    def _setup_ros(self):
        from geometry_msgs.msg import Twist
        from nav2_msgs.action import ComputePathToPose, FollowPath
        from rclpy.action import ActionClient
        from std_msgs.msg import String
        n = self.node
        self._plan_ac = ActionClient(n, ComputePathToPose,
                                     'compute_path_to_pose')
        self._follow_ac = ActionClient(n, FollowPath, 'follow_path')
        self._cmd_sub = n.create_subscription(
            Twist, self.NAV2_CMD_TOPIC, self._on_cmd, 10)
        self._status_pub = n.create_publisher(String, self.STATUS_TOPIC, 10)
        self._String = String
        self._timer = n.create_timer(self._param('goal_period'),
                                     self._goal_tick)

    def close(self):
        """Detach from the node (stop the goal loop, drop the action
        clients); used when the brain is swapped or a test is over."""
        if self.node is None or getattr(self, '_timer', None) is None:
            return
        self.node.destroy_timer(self._timer)
        self.node.destroy_subscription(self._cmd_sub)
        self._plan_ac.destroy()
        self._follow_ac.destroy()
        self._timer = None

    def _now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg):
        self._nav2_cmd = (msg.linear.x, msg.angular.z)
        self._nav2_cmd_t = self._now()

    # -- brain interface -----------------------------------------------------
    def compute(self, self_xy, self_theta, pursuers):
        self._self = (self_xy[0], self_xy[1])
        self._theta = self_theta
        self._pursuers = list(pursuers)
        return self.reactive.compute(self_xy, self_theta, pursuers)

    def _near(self):
        if self._self is None:
            return 1e9
        return min((math.hypot(p[0] - self._self[0], p[1] - self._self[1])
                    for p in self._pursuers), default=1e9)

    def nav2_available(self, now):
        if self.node is None or now < self._fallback_until:
            return False
        if now - self._nav2_cmd_t > self._param('cmd_timeout'):
            return False
        return self._nav2_cmd is not None

    def blend_twist(self, reactive_cmd, speed_cap, w_max, theta=0.0,
                    reactive_heading=None, last_w=0.0):
        """Final (v, w): reactive_cmd is what the ReactiveEvader would send
        (reactive_heading = the direction it was derived from)."""
        now = self._now() if self.node is not None else 0.0
        near = self._near()
        w_r = blend_weight(near, self._param('reactive_radius'),
                           self._param('reactive_min'))
        ok = self.nav2_available(now)
        self.mode = mode_for(w_r, ok)
        self.stats['ticks_' + self.mode] += 1
        self._mode_time[self.mode] += self._tick_dt(now)
        if not ok and self.node is not None:
            self.stats['ticks_unavailable'] += 1
        out = reactive_cmd
        if ok and self.mode != 'reactive':
            v = clamp(self._nav2_cmd[0], 0.0, speed_cap)
            w = clamp(self._nav2_cmd[1], -w_max, w_max)
            h_r = reactive_heading if reactive_heading is not None \
                else theta + reactive_cmd[1] / 3.0        # w = kw * error
            out = blend_cmd(theta, (v, w), reactive_cmd, h_r, w_r,
                            speed_cap, w_max, last_w=last_w)
        self._publish_status(now, near, w_r)
        return out

    def _tick_dt(self, now):
        dt = 0.0 if self._last_tick_t is None else min(now - self._last_tick_t, 0.2)
        self._last_tick_t = now
        return max(dt, 0.0)

    # -- goal loop -----------------------------------------------------------
    def _ensure_map(self):
        """EscapeMap of the current arena (rebuilt if the bounds shrink)."""
        key = (tuple(self.bmin), tuple(self.bmax))
        if key != self._emap_key:
            from .escape_map import EscapeMap
            self.emap = EscapeMap(
                self.bmin, self.bmax, self.obstacles,
                resolution=self._param('map_resolution'),
                robot_radius=float(self._param('planner_radius')))
            self._emap_key = key
        return self.emap

    def _start_pose(self):
        emap = self._ensure_map()
        if emap is not None:
            return emap.plan_start(self._self)
        return plan_start(self._self, self.bmin, self.bmax, self.obstacles,
                          self.body_radius + self._param('map_resolution') / 2)

    def _dom_cfg(self):
        n = self.node
        def get(name, default):
            try:
                return float(n.get_parameter(name).value)
            except Exception:
                return default
        try:
            v_self = float(n._v_max())
        except Exception:
            v_self = 3.6
        return dom.DominanceConfig(
            v_self=v_self, v_purs=get('agent_max_speed', 2.0),
            omega=get('target_omega_max', 1.2),
            margin_s=float(self._param('dom_margin')),
            goal_clearance=float(self._param('goal_clearance')))

    def _pick_goal(self, now):
        """(goal, score) from the dominance region (default) or the old
        sampled scorer (`nav2_selector:=sampled`)."""
        emap = self._ensure_map()
        if self._param('selector') == 'sampled':
            return select_escape_goal(
                self._rng, self._self, self._pursuers, self.obstacles,
                self.bmin, self.bmax, self.goal, self.cfg, emap=emap,
                blocked=lambda g: self.blacklist.blocked(g, now))
        dc = self._dom_cfg()
        purs = [(p[0], p[1]) for p in self._pursuers]
        f = dom.compute_fields(emap, self._self, self._theta, purs, dc)
        r = self._param('blacklist_radius')
        ch = dom.select_goal(
            emap, f, self._self, purs, self.goal, dc,
            exclude=[(x, y, r) for (x, y) in self.blacklist.active(now)])
        self.dom_empty = ch.empty
        if ch.goal is None:
            return tuple(self._self), -math.inf
        return ch.goal, ch.score

    def _goal_tick(self):
        now = self._now()
        if self._self is None:
            return
        late = self.tracker.expire(now)
        if late is not None:                   # timed out: cancel, drop late result
            self.stats['plan_timeout'] += 1
            if self._plan_gh is not None:
                self._plan_gh.cancel_goal_async()
                self._plan_gh = None
            self._fail('plan_fail', now)
        if self.tracker.pending or now < self._fallback_until:
            return                           # one request at a time; hold = reactive
        if not (self._plan_ac.server_is_ready()
                and self._follow_ac.server_is_ready()):
            return
        goal, score = self._pick_goal(now)
        if score == -math.inf:             # nothing reachable / allowed
            self.goal = None
            return
        changed = self.goal is None or math.hypot(
            goal[0] - self.goal[0], goal[1] - self.goal[1]) > 1e-6
        if changed:
            self.stats['goals'] += 1
        self.goal = goal
        stale = now - self._last_plan_t >= self._param('replan_period')
        if should_request_plan(now, self._fallback_until, changed, stale,
                               self.nav2_available(now)):
            self._request_plan(goal, now)

    def _request_plan(self, goal, now):
        from nav2_msgs.action import ComputePathToPose
        g = ComputePathToPose.Goal()
        g.goal.header.frame_id = 'map'
        g.goal.header.stamp = self.node.get_clock().now().to_msg()
        g.goal.pose.position.x, g.goal.pose.position.y = goal
        g.goal.pose.orientation.w = 1.0
        sx, sy = self._start_pose()
        g.use_start = True
        g.start.header = g.goal.header
        g.start.pose.position.x, g.start.pose.position.y = sx, sy
        g.start.pose.orientation.w = 1.0
        g.planner_id = 'GridBased'
        seq = self.tracker.begin(now)
        self._plan_goal = goal
        self._last_plan_t = now
        self.stats['plan_requests'] += 1
        fut = self._plan_ac.send_goal_async(g)
        fut.add_done_callback(lambda f, s=seq: self._on_plan_accepted(f, s))

    def _on_plan_accepted(self, fut, seq):
        gh = fut.result()
        if not self.tracker.accept(seq):       # timed out / superseded
            self.stats['late_dropped'] += 1
            if gh.accepted:
                gh.cancel_goal_async()
            return
        if not gh.accepted:
            self.tracker.close(seq)
            self._fail('plan_fail', self._now())
            return
        self._plan_gh = gh
        gh.get_result_async().add_done_callback(
            lambda f, s=seq: self._on_plan_result(f, s))

    def _on_plan_result(self, fut, seq):
        if not self.tracker.accept(seq):
            self.stats['late_dropped'] += 1
            return
        self.tracker.close(seq)
        self._plan_gh = None
        res = fut.result()
        path = res.result.path
        if res.status != 4 or len(path.poses) < 2:
            self._fail('plan_fail', self._now())
            return
        self.stats['plan_ok'] += 1
        from nav2_msgs.action import FollowPath
        f = FollowPath.Goal()
        if self._param('smooth_start') and self._self is not None:
            xy = smooth_start([(q.pose.position.x, q.pose.position.y)
                               for q in path.poses],
                              self._self, self._theta)
            if len(xy) != len(path.poses):
                from geometry_msgs.msg import PoseStamped
                np_ = type(path)()
                np_.header = path.header
                for (x, y) in xy:
                    q = PoseStamped()
                    q.header = path.header
                    q.pose.position.x, q.pose.position.y = x, y
                    q.pose.orientation.w = 1.0
                    np_.poses.append(q)
                path = np_
                self.stats['smoothed'] = self.stats.get('smoothed', 0) + 1
        f.path = path
        f.controller_id = 'FollowPath'
        f.goal_checker_id = 'general_goal_checker'
        self._follow_seq += 1
        fseq = self._follow_seq
        self._follow_goal = self._plan_goal
        # a new goal preempts the running FollowPath in controller_server
        self._follow_ac.send_goal_async(f).add_done_callback(
            lambda ff, s=fseq: self._on_follow_accepted(ff, s))

    def _on_follow_accepted(self, fut, fseq):
        gh = fut.result()
        if not gh.accepted:
            self._fail('follow_abort', self._now(), self._follow_goal)
            return
        gh.get_result_async().add_done_callback(
            lambda f, s=fseq: self._on_follow_result(f, s))

    def _on_follow_result(self, fut, fseq):
        if fseq != self._follow_seq:        # superseded (preempted) goal
            return
        status = fut.result().status
        if status == 4:
            self.stats['follow_done'] += 1
        elif status == 6:                   # ABORTED
            self._fail('follow_abort', self._now(), self._follow_goal)

    def _fail(self, key, now, goal=None):
        """Count the failure, go reactive for `fallback_hold`, remember the
        goal as bad for a while and drop it so the next pick is a new one."""
        goal = goal if goal is not None else self._plan_goal
        self.stats[key] += 1
        self._fallback_until = now + self._param('fallback_hold')
        if goal is not None:
            self.blacklist.add(goal, now)
            self.stats['blacklisted'] += 1
        self.goal = None
        self.node.get_logger().warn(
            f'nav2 evader: {key} (total {self.stats[key]}) -> reactive '
            f'for {self._param("fallback_hold"):.1f}s, goal blacklisted')

    def _publish_status(self, now, near, w_r):
        if self.node is None or now - self._last_status_t < 0.1:
            return
        self._last_status_t = now
        msg = self._String()
        msg.data = json.dumps({
            'mode': self.mode, 'near': round(min(near, 99.0), 2),
            'w_reactive': round(w_r, 3),
            'goal': None if self.goal is None
            else [round(self.goal[0], 2), round(self.goal[1], 2)],
            **{f't_{k}': round(v, 2) for k, v in self._mode_time.items()},
            **self.stats}, separators=(',', ':'))
        self._status_pub.publish(msg)
