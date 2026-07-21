"""Vector / angle helpers (SDD v3 §7.1). Pure math, no ROS imports."""

import math


def wrap_angle(a: float) -> float:
    """Normalize an angle into [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def norm(vx: float, vy: float) -> float:
    return math.hypot(vx, vy)


def unit(vx: float, vy: float, fallback=(0.0, 0.0)):
    m = math.hypot(vx, vy)
    if m < 1e-9:
        return fallback
    return vx / m, vy / m


def heading_mean(thetas):
    """Mean heading as unit-vector average (never average raw angles)."""
    if not thetas:
        return 0.0, 0.0
    vx = sum(math.cos(t) for t in thetas) / len(thetas)
    vy = sum(math.sin(t) for t in thetas) / len(thetas)
    return unit(vx, vy)


HYST_BAND = 2.6      # |e| beyond this: commit to previous turn direction


def to_twist(v_desired, theta, kv, kw, v_min, v_max, w_max, last_w=0.0):
    """Non-holonomic conversion, deadlock-fixed (v2 §4.5 / v3 §2).

    Returns (linear, angular). Never reverses; v_min floor lets the agent
    rotate-while-nudging out of a perpendicular stall.

    Anti-chatter hysteresis: when the target is nearly BEHIND (|e|~pi),
    tiny jitter in the desired vector flips the sign of e across the +-pi
    wrap, so a plain P controller alternates full-left/full-right forever
    (observed under sensing latency). Beyond HYST_BAND we keep turning in
    the direction already chosen (last_w), breaking the symmetry.
    """
    vx, vy = v_desired
    mag = math.hypot(vx, vy)
    if mag < 1e-6:
        return 0.0, 0.0
    e = wrap_angle(math.atan2(vy, vx) - theta)
    if abs(e) > HYST_BAND and last_w != 0.0:
        direction = 1.0 if last_w > 0.0 else -1.0
        wz = direction * min(kw * abs(e), w_max)
    else:
        wz = clamp(kw * e, -w_max, w_max)
    forward = max(0.0, math.cos(e))
    v_lin = clamp(kv * forward * mag, v_min, v_max)
    return v_lin, wz
