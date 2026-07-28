"""Target evasion brains (SDD v3 §4.3, §6.3) — strategy pattern.

ReactiveEvader: reverse-Boids + gap-seeking + wall/obstacle avoidance.
AdaptiveEvader: utility selector over a behavior repertoire (v4 D / M14).
Nav2Evader: M7 adapter stub (see §6) — the sim already publishes the
plumbing contract; wiring Nav2 is a config exercise on top of this.
"""

import math

from ..geometry import unit, wrap_angle


class ReactiveEvader:
    """Sampled-direction scoring: pick the least-threatened heading.

    For K candidate directions, penalize (a) pursuers lying that way —
    closer pursuers weigh more (this *is* gap-seeking: the biggest gap
    between pursuers scores best), (b) walls/obstacles ahead, and
    (c) large turns (a non-holonomic target can't pivot instantly).
    """

    K = 24                    # candidate directions
    WALL_LOOKAHEAD = 3.5      # units of clear space we'd like ahead
    TURN_PENALTY = 0.55
    WALL_WEIGHT = 3.0

    def __init__(self, bounds_min, bounds_max, obstacles=()):
        self.bmin, self.bmax = bounds_min, bounds_max
        self.obstacles = list(obstacles)

    def _clearance(self, x, y, ux, uy):
        """Free distance from (x,y) along (ux,uy) before wall/obstacle."""
        ts = []
        if ux > 1e-9:
            ts.append((self.bmax[0] - x) / ux)
        elif ux < -1e-9:
            ts.append((self.bmin[0] - x) / ux)
        if uy > 1e-9:
            ts.append((self.bmax[1] - y) / uy)
        elif uy < -1e-9:
            ts.append((self.bmin[1] - y) / uy)
        t_free = min(ts) if ts else float('inf')
        for (cx, cy, r) in self.obstacles:
            # ray-circle intersection
            fx, fy = cx - x, cy - y
            proj = fx * ux + fy * uy
            if proj <= 0:
                continue
            perp2 = fx * fx + fy * fy - proj * proj
            if perp2 < r * r:
                t_free = min(t_free, proj - math.sqrt(r * r - perp2))
        return max(t_free, 0.0)

    def compute(self, self_xy, self_theta, pursuers):
        """pursuers: list of (x, y, theta). Returns desired unit vector."""
        x, y = self_xy
        best_u, best_score = (math.cos(self_theta), math.sin(self_theta)), None
        for k in range(self.K):
            ang = 2.0 * math.pi * k / self.K
            ux, uy = math.cos(ang), math.sin(ang)

            threat = 0.0
            for (px, py, _th) in pursuers:
                dx, dy = px - x, py - y
                d = max(math.hypot(dx, dy), 0.3)
                # only pursuers roughly in this direction threaten it
                cos_sim = (dx * ux + dy * uy) / d
                threat += max(0.0, cos_sim) / d

            clear = self._clearance(x, y, ux, uy)
            wall_pen = self.WALL_WEIGHT * max(
                0.0, 1.0 - clear / self.WALL_LOOKAHEAD)
            turn_pen = self.TURN_PENALTY * abs(
                wrap_angle(ang - self_theta)) / math.pi

            score = threat + wall_pen + turn_pen
            if best_score is None or score < best_score:
                best_score, best_u = score, (ux, uy)
        return best_u


class AdaptiveEvader:
    """Utility-based behavior selector (SDD v4 Part D, M14).

    An arms-race target: instead of one hard-wired flee rule, it scores a
    repertoire of behaviors each cycle by expected survival utility given
    the threat geometry (nearest pursuer, encirclement completeness, wall
    distance, obstacle cover) and picks the argmax — with hysteresis so it
    doesn't dither. Interpretable weighted heuristic, not a learned policy
    (D.2), so it stays tunable and debuggable.

    Note: stamina is NOT an input here. The sim owns the stamina state and
    the target_controller already gates sprinting on `panic_distance`
    (§4.2), so the selector only picks a *direction*; feeding stamina in
    would need a new sim→controller topic (not wired).

    Composes ReactiveEvader for the shared clearance/threat geometry.
    `.mode` exposes the last choice for logging (D acceptance).
    """

    SWITCH_MARGIN = 0.15      # new behavior must beat current by this
    PANIC = 3.0               # a pursuer this close ⇒ juke is attractive

    def __init__(self, bounds_min, bounds_max, obstacles=()):
        self.geo = ReactiveEvader(bounds_min, bounds_max, obstacles)
        self.bmin, self.bmax = bounds_min, bounds_max
        self.obstacles = list(obstacles)
        self.mode = 'retreat'

    # --- situational features ------------------------------------------
    def _nearest(self, xy, pursuers):
        if not pursuers:
            return None, float('inf')
        best, bd = None, float('inf')
        for p in pursuers:
            d = math.hypot(p[0] - xy[0], p[1] - xy[1])
            if d < bd:
                best, bd = p, d
        return best, bd

    def _encirclement(self, xy, pursuers):
        """Fraction of 8 sectors around self occupied by a near pursuer:
        1.0 = fully surrounded."""
        if not pursuers:
            return 0.0
        sect = [False] * 8
        for (px, py, *_r) in pursuers:
            dx, dy = px - xy[0], py - xy[1]
            if math.hypot(dx, dy) < 6.0:
                sect[int((math.atan2(dy, dx) % (2 * math.pi))
                         / (2 * math.pi / 8)) % 8] = True
        return sum(sect) / 8.0

    def _biggest_gap_dir(self, xy, pursuers):
        """Direction bisecting the widest angular gap between pursuers."""
        if not pursuers:
            return None
        angs = sorted(math.atan2(p[1] - xy[1], p[0] - xy[0])
                      for p in pursuers)
        best_mid, best_w = None, -1.0
        for i in range(len(angs)):
            a0 = angs[i]
            a1 = angs[(i + 1) % len(angs)] + (2 * math.pi if i + 1 == len(angs)
                                              else 0.0)
            w = a1 - a0
            if w > best_w:
                best_w, best_mid = w, (a0 + a1) / 2.0
        return (math.cos(best_mid), math.sin(best_mid))

    def _shield_dir(self, xy, nearest):
        """Direction that puts an obstacle between self and the nearest
        pursuer.

        Only obstacles on the side *away* from the pursuer qualify — running
        at the obstacle the pursuer is standing next to hands it the kill.
        Among those, take the closest. Returns None when no obstacle can
        actually shield, so the selector drops the option instead of
        scoring a direction that does the opposite of its name.
        """
        if not self.obstacles or nearest is None:
            return None
        px, py = unit(nearest[0] - xy[0], nearest[1] - xy[1])
        if (px, py) == (0.0, 0.0):
            return None                    # pursuer on top of us: no bearing
        best, best_d = None, float('inf')
        for (ox, oy, _r) in self.obstacles:
            dx, dy = ox - xy[0], oy - xy[1]
            d = math.hypot(dx, dy)
            if d < 1e-9:
                continue
            if (dx / d) * px + (dy / d) * py >= 0.0:
                continue                   # obstacle lies toward the pursuer
            if d < best_d:
                best, best_d = (dx / d, dy / d), d
        return best

    # --- the selector ---------------------------------------------------
    def compute(self, self_xy, self_theta, pursuers):
        nearest, nd = self._nearest(self_xy, pursuers)
        enc = self._encirclement(self_xy, pursuers)

        # candidate behaviors: (mode, direction, situational bonus)
        cands = []
        retreat = self.geo.compute(self_xy, self_theta, pursuers)
        cands.append(('retreat', retreat, 0.0))

        # perimeter-run: tangent along the nearest wall
        cands.append(('perimeter', self._perimeter_dir(self_xy, self_theta),
                      0.2))

        if nearest is not None and nd < self.PANIC:
            # juke: perpendicular to the nearest pursuer's approach
            ax, ay = unit(self_xy[0] - nearest[0], self_xy[1] - nearest[1])
            juke = (-ay, ax) if (self_theta % (2 * math.pi)) < math.pi \
                else (ay, -ax)
            cands.append(('juke', juke, 0.5 + 0.4 * (1 - nd / self.PANIC)))

        gap = self._biggest_gap_dir(self_xy, pursuers)
        if gap is not None:
            cands.append(('gap_dash', gap, 0.6 * enc))   # value rises when boxed

        shield = self._shield_dir(self_xy, nearest)
        if shield is not None:
            cands.append(('shield', shield, 0.3))

        # score each by clearance/threat (lower geo-score = safer) + bonus
        best = None
        for (mode, d, bonus) in cands:
            if d is None:
                continue
            safety = self._safety(self_xy, self_theta, d, pursuers)
            util = safety + bonus
            if mode == self.mode:
                util += self.SWITCH_MARGIN            # hysteresis: incumbency
            if best is None or util > best[0]:
                best = (util, mode, d)
        self.mode = best[1]
        return best[2]

    def _safety(self, xy, theta, d, pursuers):
        """Higher = safer. Clearance ahead minus pursuer threat that way."""
        clear = self.geo._clearance(xy[0], xy[1], d[0], d[1])
        threat = 0.0
        for (px, py, *_r) in pursuers:
            fx, fy = px - xy[0], py - xy[1]
            dist = max(math.hypot(fx, fy), 0.3)
            cos_sim = (fx * d[0] + fy * d[1]) / dist
            threat += max(0.0, cos_sim) / dist
        turn_pen = 0.4 * abs(wrap_angle(math.atan2(d[1], d[0]) - theta)) \
            / math.pi
        return min(clear, 6.0) / 6.0 - threat - turn_pen

    def _perimeter_dir(self, xy, theta):
        """Tangent along the nearest wall, continuing current heading."""
        dl, dr = xy[0] - self.bmin[0], self.bmax[0] - xy[0]
        db, dt = xy[1] - self.bmin[1], self.bmax[1] - xy[1]
        m = min(dl, dr, db, dt)
        if m in (dl, dr):
            tang = (0.0, 1.0)                          # wall is vertical
        else:
            tang = (1.0, 0.0)
        # pick the tangent sign that agrees with current heading
        if tang[0] * math.cos(theta) + tang[1] * math.sin(theta) < 0:
            tang = (-tang[0], -tang[1])
        return tang


class Nav2Evader:
    """M7 stretch (SDD §6): obstacle-aware fleeing via Nav2.

    Design contract (§6.4): consume /target/odom + TF + /map published by
    pygame_sim_node, compute an escape goal each cycle, send it to Nav2's
    local controller, and blend a reactive term between goal updates.
    Not wired in this milestone — instantiate ReactiveEvader instead.
    """

    def __init__(self, *_, **__):
        raise NotImplementedError(
            'Nav2Evader is the M7 stretch milestone (SDD v3 §6). '
            'Run with evader:=reactive.')
