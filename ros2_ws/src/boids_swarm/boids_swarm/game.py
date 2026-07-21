"""Capture conditions, scoring, episode bookkeeping (SDD v3 §4.5–4.6)."""

import math


# --- geometry helpers -------------------------------------------------------

def convex_hull(points):
    """Andrew's monotone chain. Returns hull vertices CCW."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def half(seq):
        h = []
        for p in seq:
            while len(h) >= 2 and (
                    (h[-1][0] - h[-2][0]) * (p[1] - h[-2][1])
                    - (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0])) <= 0:
                h.pop()
            h.append(p)
        return h

    lower, upper = half(pts), half(reversed(pts))
    return lower[:-1] + upper[:-1]


def point_in_polygon(p, poly):
    """Ray casting; poly is a list of vertices."""
    x, y = p
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            xin = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < xin:
                inside = not inside
    return inside


# --- capture conditions (§4.5) ---------------------------------------------

def capture_hull(target_xy, boid_positions, k, d_capture):
    """Containment: target inside the convex hull of >=k boids that are
    all within d_capture of it."""
    close = [(x, y) for (x, y) in boid_positions
             if math.hypot(x - target_xy[0], y - target_xy[1]) < d_capture]
    if len(close) < max(k, 3):          # a hull needs >=3 vertices
        return False, len(close)
    hull = convex_hull(close)
    if len(hull) < 3:
        return False, len(close)
    return point_in_polygon(target_xy, hull), len(close)


def capture_escape_blocked(target_xy, boid_positions, d_capture, n_dirs=8):
    """Sample escape directions; captured when every sector is blocked by
    a boid within d_capture."""
    sector = 2.0 * math.pi / n_dirs
    blocked = [False] * n_dirs
    n_close = 0
    for (x, y) in boid_positions:
        dx, dy = x - target_xy[0], y - target_xy[1]
        if math.hypot(dx, dy) >= d_capture:
            continue
        n_close += 1
        idx = int((math.atan2(dy, dx) % (2.0 * math.pi)) / sector) % n_dirs
        blocked[idx] = True
    return all(blocked), n_close


class TagHealth:
    """HP model: any boid within d_capture drains HP; regen otherwise."""

    def __init__(self, hp_max=3.0, drain_per_boid=1.0, regen=0.5):
        self.hp_max = hp_max
        self.hp = hp_max
        self.drain = drain_per_boid
        self.regen = regen

    def reset(self):
        self.hp = self.hp_max

    def update(self, target_xy, boid_positions, d_capture, dt):
        n_close = sum(
            1 for (x, y) in boid_positions
            if math.hypot(x - target_xy[0], y - target_xy[1]) < d_capture)
        if n_close:
            self.hp -= self.drain * n_close * dt
        else:
            self.hp = min(self.hp_max, self.hp + self.regen * dt)
        return self.hp <= 0.0, n_close


# --- scoring / episode state (§4.6, §7.4) -----------------------------------

class Scoreboard:
    def __init__(self):
        self.episode = 0
        self.captures = 0
        self.timeouts = 0
        self.t_episode = 0.0
        self.capture_times = []
        self.min_pairwise = float('inf')
        self.boids_lost = 0

    def start_episode(self):
        self.episode += 1
        self.t_episode = 0.0
        self.min_pairwise = float('inf')

    def tick(self, dt, boid_positions):
        self.t_episode += dt
        n = len(boid_positions)
        for i in range(n):
            x1, y1 = boid_positions[i]
            for j in range(i + 1, n):
                x2, y2 = boid_positions[j]
                d = math.hypot(x1 - x2, y1 - y2)
                if d < self.min_pairwise:
                    self.min_pairwise = d

    def end_episode(self, captured: bool):
        if captured:
            self.captures += 1
            self.capture_times.append(self.t_episode)
        else:
            self.timeouts += 1

    def summary_line(self, result: str) -> str:
        return (f'EPISODE {self.episode} result={result} '
                f't={self.t_episode:.2f}s min_pairwise={self.min_pairwise:.3f} '
                f'captures={self.captures}/{self.episode}')
