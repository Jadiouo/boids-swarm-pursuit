"""Core Boids behaviors (v3 §3; rationale in v2 §4.3).

Each returns a 2D vector in world coordinates.
A neighbor is a tuple (x, y, theta).
"""

import math

from ..geometry import unit


def separation(self_xy, neighbors, d_safe: float):
    """Repel from neighbors closer than d_safe, 1/d^2 falloff."""
    vx = vy = 0.0
    sx, sy = self_xy
    for (x, y, _th) in neighbors:
        dx, dy = sx - x, sy - y
        d = math.hypot(dx, dy)
        if 1e-9 < d < d_safe:
            vx += dx / (d * d)
            vy += dy / (d * d)
        elif d <= 1e-9:
            vx += 1.0e3          # coincident: strong deterministic nudge
    return vx, vy


def alignment(neighbors):
    """Mean neighbor heading, averaged as unit vectors (wrap-safe)."""
    if not neighbors:
        return 0.0, 0.0
    vx = sum(math.cos(th) for (_x, _y, th) in neighbors) / len(neighbors)
    vy = sum(math.sin(th) for (_x, _y, th) in neighbors) / len(neighbors)
    return unit(vx, vy)


def cohesion(self_xy, neighbors):
    """Unit vector toward the neighborhood center of mass."""
    if not neighbors:
        return 0.0, 0.0
    cx = sum(x for (x, _y, _th) in neighbors) / len(neighbors)
    cy = sum(y for (_x, y, _th) in neighbors) / len(neighbors)
    return unit(cx - self_xy[0], cy - self_xy[1])


def boundary(self_xy, bounds_min, bounds_max, margin: float):
    """Soft-wall inward push, proportional to margin intrusion."""
    x, y = self_xy
    vx = vy = 0.0
    if x < bounds_min[0] + margin:
        vx += (bounds_min[0] + margin - x) / margin
    if x > bounds_max[0] - margin:
        vx -= (x - (bounds_max[0] - margin)) / margin
    if y < bounds_min[1] + margin:
        vy += (bounds_min[1] + margin - y) / margin
    if y > bounds_max[1] - margin:
        vy -= (y - (bounds_max[1] - margin)) / margin
    return vx, vy


def obstacle_avoid(self_xy, obstacles, margin: float, desired=None,
                   swirl=1.1):
    """Repulsion from circular obstacles + a tangential slide-around.

    A pure radial push makes a boid heading at an obstacle stall in front
    of it (pursuit pulls in, push shoves out, they cancel — the "dumb"
    wall-plug behaviour). When `desired` (the boid's intended velocity) is
    given, we add a TANGENTIAL term along the obstacle surface, on the side
    that best continues `desired`, so the boid arcs smoothly around the
    plug instead of grinding to a halt.

    obstacles: iterable of (cx, cy, radius).
    """
    vx = vy = 0.0
    x, y = self_xy
    for (cx, cy, r) in obstacles:
        dx, dy = x - cx, y - cy
        d = math.hypot(dx, dy) - r          # distance to obstacle surface
        if d < margin:
            push = (margin - max(d, 0.0)) / margin
            ux, uy = unit(dx, dy, fallback=(1.0, 0.0))
            vx += ux * push
            vy += uy * push
            if desired is not None:
                # two tangents to the surface; pick the one going the way
                # the boid already wants to go, so it slides around
                t1 = (-uy, ux)
                t2 = (uy, -ux)
                dot1 = t1[0] * desired[0] + t1[1] * desired[1]
                tx, ty = t1 if dot1 >= 0.0 else t2
                vx += tx * push * swirl
                vy += ty * push * swirl
    return vx, vy


def wander_vec(theta: float, wander_angle: float):
    """Unit vector along the persistent wander heading."""
    return math.cos(theta + wander_angle), math.sin(theta + wander_angle)


def search_vec(self_xy, index, n_agents, world_size, t, sweep_rate=0.4,
               ring_frac=0.32):
    """Coverage sweep when the target is lost (SDD v4 A.6).

    Each agent owns a distinct angular sector (from its index) and steers
    toward a patrol point orbiting the arena centre; the whole angle set
    rotates slowly, so the swarm collectively scans the arena rather than
    every agent wandering independently. When the target re-enters any
    agent's cone, that agent drops search and re-engages (and, with comms,
    shares the sighting — M10).
    """
    cx = cy = world_size * 0.5
    phi = 2.0 * math.pi * index / max(n_agents, 1) + sweep_rate * t
    r = ring_frac * world_size
    wp = (cx + r * math.cos(phi), cy + r * math.sin(phi))
    return unit(wp[0] - self_xy[0], wp[1] - self_xy[1])
