"""Safe target spawn (pure Python, unit-testable; no ROS, no pygame).

Why this exists
    The legacy spawn (`PygameSimNode._free_pos(clear_of=swarm, min_dist=6)`)
    tries 200 random points and, when none keeps 6 m from every boid, silently
    returns the ARENA CENTRE. With 12 boids in a 20 x 20 m arena that happens
    for roughly half the seeds, so the target appears in the middle of the
    swarm and is hull-captured within ~1 s. (capture_hull needs >=k boids
    within d_capture, so "inside the hull at t=0" is not the mechanism;
    "centre fallback, <=2 m from several boids" is.)

Design (re-sampling, not a one-sided layout)
    Boids keep their legacy spawn so the swarm distribution is unchanged.
    The target is then drawn with a seeded RNG from many candidates:
      1. clearance to the nearest boid >= `min_clearance`, and not inside an
         obstacle: accepted at once. Because capture_hull/escape_blocked only
         count boids within d_capture and min_clearance > d_capture, no
         capture hull (of ANY k-subset) can contain the target at t=0. The
         full-swarm hull is deliberately NOT required to exclude the target:
         it covers most of the arena, so demanding it would be infeasible.
      2. nothing satisfies it: take the candidate with the LARGEST
         clearance and report `ok=False` (the caller logs a warning).
    A dense candidate budget (not 200) is what makes (1) succeed: a dense
    grid shows a 6 m free spot exists for 9 of the 10 test seeds.
"""

import math


def in_obstacle(x, y, obstacles, pad=0.8):
    return any(math.hypot(x - ox, y - oy) < r + pad
               for (ox, oy, r) in obstacles)


def clearance(x, y, pursuers):
    """Distance to the nearest pursuer (inf if there are none)."""
    return min((math.hypot(x - px, y - py) for (px, py) in pursuers),
               default=float('inf'))


def choose_target_spawn(pursuers, obstacles, world, rng, min_clearance=6.0,
                        margin=1.5, tries=1000, bounds=None):
    """Pick the target's start position.

    Returns ``(x, y, ok, clr)``: ``ok`` is False when no candidate reached
    `min_clearance` (the returned point then has the largest clearance seen);
    ``clr`` is its clearance. Deterministic for a given `rng` state. Falls
    back to the arena centre only when every candidate is inside an obstacle.
    """
    lo_x, lo_y, hi_x, hi_y = bounds or (0.0, 0.0, world, world)
    best, best_clr = None, -1.0
    for _ in range(max(1, int(tries))):
        x = rng.uniform(lo_x + margin, hi_x - margin)
        y = rng.uniform(lo_y + margin, hi_y - margin)
        if in_obstacle(x, y, obstacles):
            continue
        c = clearance(x, y, pursuers)
        if c >= min_clearance:
            return x, y, True, c
        if c > best_clr:
            best, best_clr = (x, y), c
    if best is None:
        return (lo_x + hi_x) / 2.0, (lo_y + hi_y) / 2.0, False, \
            clearance((lo_x + hi_x) / 2.0, (lo_y + hi_y) / 2.0, pursuers)
    return best[0], best[1], False, best_clr
