"""Pure-math Boids behaviors (SDD v2 §4.3–§4.5).

Every behavior returns a 2D vector (vx, vy) in world coordinates.
Kept free of ROS imports so they are unit-testable (SDD §8).

A neighbor is a tuple (x, y, theta).
"""

import math


def wrap_angle(a: float) -> float:
    """Normalize an angle into [-pi, pi] (SDD §4.5 step 2)."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def separation(self_xy, neighbors, d_safe: float):
    """§4.3.1 — repel from neighbors closer than d_safe, weight 1/d^2.

    V_sep = sum_{i: d_i < d_safe} (p_self - p_i) / d_i^2
    """
    vx = vy = 0.0
    sx, sy = self_xy
    for (x, y, _theta) in neighbors:
        dx, dy = sx - x, sy - y
        d = math.hypot(dx, dy)
        if 1e-9 < d < d_safe:
            vx += dx / (d * d)
            vy += dy / (d * d)
        elif d <= 1e-9:
            # Exactly coincident: no defined direction; strong nudge along
            # +x so the pair does not stay merged forever.
            vx += 1.0e3
    return vx, vy


def alignment(neighbors):
    """§4.3.2 — mean heading of neighbors, averaged as unit vectors.

    Never average raw angles (wrap-around trap): convert each theta_i to
    a unit vector, average, and return the (normalized) resultant.
    """
    if not neighbors:
        return 0.0, 0.0
    vx = sum(math.cos(th) for (_x, _y, th) in neighbors) / len(neighbors)
    vy = sum(math.sin(th) for (_x, _y, th) in neighbors) / len(neighbors)
    mag = math.hypot(vx, vy)
    if mag < 1e-9:
        # Headings cancel out (e.g. two opposite neighbors): no consensus.
        return 0.0, 0.0
    return vx / mag, vy / mag


def cohesion(self_xy, neighbors):
    """§4.3.3 — steer toward the neighborhood center of mass (unit-scaled)."""
    if not neighbors:
        return 0.0, 0.0
    cx = sum(x for (x, _y, _th) in neighbors) / len(neighbors)
    cy = sum(y for (_x, y, _th) in neighbors) / len(neighbors)
    vx, vy = cx - self_xy[0], cy - self_xy[1]
    mag = math.hypot(vx, vy)
    if mag < 1e-9:
        return 0.0, 0.0
    # Normalize so cohesion stays comparable to the other behaviors.
    return vx / mag, vy / mag


def boundary(self_xy, bounds_min, bounds_max, margin: float):
    """§4.3.4 — soft-wall push, proportional to margin intrusion."""
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


def wander(theta: float, wander_angle: float):
    """§4.3.5 — unit vector along the current wander heading.

    The caller owns the persistent wander_angle random walk.
    """
    return math.cos(theta + wander_angle), math.sin(theta + wander_angle)


def to_twist(v_desired, theta: float, kv: float, kw: float,
             v_min: float, v_max: float, w_max: float):
    """§4.5 — non-holonomic conversion, deadlock-fixed.

    Returns (linear_x, angular_z). Never commands reverse; a v_min floor
    keeps the agent creeping forward so it can rotate-while-nudging.
    """
    vx, vy = v_desired
    mag = math.hypot(vx, vy)
    if mag < 1e-6:
        # No desired motion: stop (target heading is undefined).
        return 0.0, 0.0
    theta_target = math.atan2(vy, vx)
    e = wrap_angle(theta_target - theta)
    wz = clamp(kw * e, -w_max, w_max)
    forward = max(0.0, math.cos(e))          # clamp at 0: no reversing
    v_lin = clamp(kv * forward * mag, v_min, v_max)
    return v_lin, wz
