"""Cooperative pursuit strategy ladder T0–T4 (SDD v3 §4.4).

Each strategy is a pure function (ctx, params) -> desired unit vector.
Swappable at runtime via the `pursuit_strategy` parameter (strategy
pattern, §7.2): adding a tactic must not touch sim or controller code.
"""

import math
from dataclasses import dataclass

from ..geometry import clamp, unit


@dataclass
class PursuitContext:
    """Everything a strategy may look at. All world-frame."""
    self_xy: tuple          # (x, y)
    self_theta: float
    index: int              # this agent's index (deterministic ring slots)
    n_agents: int
    target_xy: tuple        # (x, y)
    target_theta: float
    target_speed: float     # scalar; velocity dir == target heading
    t_engaged: float        # seconds since this boid first saw the target
    # --- v4 M11: perimeter-circling analysis (from the shared track) ---
    center: tuple = (10.0, 10.0)   # arena centre
    circling: bool = False         # target sustained a wall loop?
    circ_dir: float = 0.0          # +1 CCW / -1 CW about centre / 0
    orbit_radius: float = 5.0      # target's current distance from centre


def _target_vel(ctx):
    return (ctx.target_speed * math.cos(ctx.target_theta),
            ctx.target_speed * math.sin(ctx.target_theta))


def _clamp_to_arena(p, params):
    bmin, bmax = params['bounds_min'], params['bounds_max']
    m = 0.5
    return (clamp(p[0], bmin[0] + m, bmax[0] - m),
            clamp(p[1], bmin[1] + m, bmax[1] - m))


def _predict(ctx, params, scale=1.0):
    """Lead point: target position + velocity * t_pred (T1 core).

    t_pred grows with distance (farther boids must lead more) and is
    capped so far boids don't chase absurd extrapolations. The point is
    clamped into the arena — the target cannot flee through a wall.
    """
    dx = ctx.target_xy[0] - ctx.self_xy[0]
    dy = ctx.target_xy[1] - ctx.self_xy[1]
    dist = math.hypot(dx, dy)
    v_max = max(params['agent_max_speed'], 1e-6)
    t_pred = min(params['lead_time'] * dist / v_max,
                 3.0 * params['lead_time']) * scale
    tvx, tvy = _target_vel(ctx)
    p = (ctx.target_xy[0] + tvx * t_pred, ctx.target_xy[1] + tvy * t_pred)
    return _clamp_to_arena(p, params)


def _seek(ctx, point):
    return unit(point[0] - ctx.self_xy[0], point[1] - ctx.self_xy[1])


# --- T0: naive tail-chase (control experiment; expected to FAIL vs 2x) ---
def naive(ctx, params):
    return _seek(ctx, ctx.target_xy)


# --- T1: predictive interception (lead pursuit) --------------------------
def intercept(ctx, params):
    return _seek(ctx, _predict(ctx, params))


# --- T2: pincer — decentralized chaser/interceptor split -----------------
def pincer(ctx, params):
    """Project relative position onto target velocity: boids ahead of the
    target become interceptors (loop far ahead), boids behind become
    chasers (short lead). No central coordinator (§4.4 T2)."""
    tvx, tvy = unit(*_target_vel(ctx), fallback=(math.cos(ctx.target_theta),
                                                 math.sin(ctx.target_theta)))
    rx = ctx.self_xy[0] - ctx.target_xy[0]
    ry = ctx.self_xy[1] - ctx.target_xy[1]
    ahead = (rx * tvx + ry * tvy) > 0.0
    return _seek(ctx, _predict(ctx, params, scale=1.8 if ahead else 0.4))


# --- T3: encirclement ring with shrinking radius --------------------------
def _ring_goal(ctx, params, phi, radius):
    center = _predict(ctx, params, scale=0.5)   # short lead: ring tracks
    goal = (center[0] + radius * math.cos(phi),
            center[1] + radius * math.sin(phi))
    return _clamp_to_arena(goal, params)


def encircle(ctx, params):
    """Deterministic world-frame angular slot from agent index; the ring
    radius shrinks with engagement time (§4.4 T3).

    Surround-then-squeeze: while a boid is still far away, its slot sits
    on a circle scaled to its own distance (radius ~ 0.45 x dist), so the
    swarm fans out and approaches from all compass directions instead of
    tail-chasing as one clump — the failure mode that makes a 2x target
    uncatchable. As boids converge the schedule radius takes over and
    the net tightens. Slots are world-frame (not target-heading-relative)
    so an evading turn doesn't whip every goal point around."""
    dist = math.hypot(ctx.target_xy[0] - ctx.self_xy[0],
                      ctx.target_xy[1] - ctx.self_xy[1])
    r_sched = max(params['ring_radius_min'],
                  params['ring_radius_start']
                  - params['ring_shrink_rate'] * ctx.t_engaged)
    r = max(r_sched, 0.45 * dist)
    phi = 2.0 * math.pi * ctx.index / max(ctx.n_agents, 1)
    return _seek(ctx, _ring_goal(ctx, params, phi, r))


# --- T4: herding — drive the target into a corner, then collapse ----------
def herd(ctx, params):
    """Approach from the open side only, weaponizing the target's own
    flee instinct to push it toward the nearest corner; once cornered,
    collapse into the shrinking encirclement ring (§4.4 T4)."""
    bmin, bmax = params['bounds_min'], params['bounds_max']
    tx, ty = ctx.target_xy
    corner = min(((bmin[0], bmin[1]), (bmin[0], bmax[1]),
                  (bmax[0], bmin[1]), (bmax[0], bmax[1])),
                 key=lambda c: math.hypot(tx - c[0], ty - c[1]))
    corner_dist = math.hypot(tx - corner[0], ty - corner[1])

    if corner_dist < params['herd_corner_dist']:
        return encircle(ctx, params)             # cornered: close the net

    # Arc of boids on the side OPPOSITE the corner (the open side).
    push_ang = math.atan2(corner[1] - ty, corner[0] - tx)
    open_ang = push_ang + math.pi
    arc = math.radians(210.0)
    frac = ctx.index / max(ctx.n_agents - 1, 1)
    phi = open_ang - arc / 2.0 + arc * frac
    return _seek(ctx, _ring_goal(ctx, params, phi,
                                 params['ring_radius_start']))


# ======================================================================
# v4 Part B — anti-perimeter tactics (defeat the wall-hugging loop, M11)
# All run on the shared/tracked circling estimate (ctx.circling/circ_dir);
# each falls back to a general tactic when the target is not circling.
# ======================================================================

def _target_angle(ctx):
    return math.atan2(ctx.target_xy[1] - ctx.center[1],
                      ctx.target_xy[0] - ctx.center[0])


def _self_angle(ctx):
    return math.atan2(ctx.self_xy[1] - ctx.center[1],
                      ctx.self_xy[0] - ctx.center[0])


def _ring_point(ctx, params, angle, radius):
    return _clamp_to_arena(
        (ctx.center[0] + radius * math.cos(angle),
         ctx.center[1] + radius * math.sin(angle)), params)


def counter_rotate(ctx, params):
    """B.1 — on a closed loop a counter-rotating interceptor is *guaranteed*
    to meet the runner head-on. Half the swarm runs the target's orbit the
    OPPOSITE way (interceptors); the rest chase from behind (chasers). The
    target must then reverse (into the chasers) or cut inward (into open-
    arena encirclement, which the swarm already wins). Role is picked from
    the boid's own index — fully decentralized."""
    if not ctx.circling:
        return encircle(ctx, params)
    r = max(ctx.orbit_radius, params['ring_radius_min'])
    if ctx.index % 2 == 0:                     # chaser: close from behind
        return intercept(ctx, params)
    # interceptor: advance along the ring OPPOSITE the target's circulation
    own = _self_angle(ctx)
    goal_ang = own - ctx.circ_dir * 0.6        # step opposite target motion
    return _seek(ctx, _ring_point(ctx, params, goal_ang, r))


def blockade(ctx, params):
    """B.2 — 2× speed is irrelevant against a stationary plug in front.
    On detecting circling, extrapolate the target's orbit far ahead and
    send 1–2 boids to ARRIVE EARLY and HOLD STATION there; the runner
    drives into them. The rest keep chasing so it can't simply reverse."""
    if not ctx.circling:
        return intercept(ctx, params)
    blockers = max(1, ctx.n_agents // 6)
    if ctx.index < blockers:
        # station point: far ahead along the target's circulation
        lead_ang = _target_angle(ctx) + ctx.circ_dir * (1.6 + 0.4 * ctx.index)
        station = _ring_point(ctx, params, lead_ang, ctx.orbit_radius)
        d = math.hypot(station[0] - ctx.self_xy[0],
                       station[1] - ctx.self_xy[1])
        if d < 0.6:
            return (0.0, 0.0)                  # arrived: hold station (plug)
        return _seek(ctx, station)
    return intercept(ctx, params)              # chasers deny the reverse


def _corners(params):
    bmin, bmax = params['bounds_min'], params['bounds_max']
    m = 2.0                                     # interior offset from corner
    return [(bmin[0] + m, bmin[1] + m), (bmin[0] + m, bmax[1] - m),
            (bmax[0] - m, bmin[1] + m), (bmax[0] - m, bmax[1] - m)]


def corner_trap(ctx, params):
    """B.3 — the target's turn radius (~v/ω_max) means it must swing wide at
    a sharp corner. Predict the next corner on its circling path, pre-stage
    some boids in the corner interior, and close as it swings wide."""
    if not ctx.circling:
        return encircle(ctx, params)
    ahead = _target_angle(ctx) + ctx.circ_dir * 0.9
    nxt = min(_corners(params),
              key=lambda c: math.hypot(c[0] - ctx.center[0] - math.cos(ahead)
                                       * ctx.orbit_radius,
                                       c[1] - ctx.center[1] - math.sin(ahead)
                                       * ctx.orbit_radius))
    if ctx.index % 3 == 0:                     # stagers wait in the corner
        return _seek(ctx, nxt)
    return intercept(ctx, params)              # rest herd it into the corner


def herd_inward(ctx, params):
    """B.4 — the inverse of v3's herd: get on the WALL side of the target so
    its flee response pushes it OFF the wall into the open centre, where
    encirclement works. Fixes T4's counterproductive push-toward-wall."""
    wall_dir = unit(ctx.target_xy[0] - ctx.center[0],
                    ctx.target_xy[1] - ctx.center[1], fallback=(1.0, 0.0))
    # spread boids in an arc just outside the target on the wall side
    spread = math.radians(70.0) * (ctx.index / max(ctx.n_agents - 1, 1) - 0.5)
    ca, sa = math.cos(spread), math.sin(spread)
    wd = (wall_dir[0] * ca - wall_dir[1] * sa,
          wall_dir[0] * sa + wall_dir[1] * ca)
    goal = _clamp_to_arena((ctx.target_xy[0] + wd[0] * 1.8,
                            ctx.target_xy[1] + wd[1] * 1.8), params)
    return _seek(ctx, goal)


# ======================================================================
# v4 Part B — new formations (M12): shrink the space, don't surround a
# point (sweep); collapse the ring asymmetrically; bait a lane.
# ======================================================================

def sweep(ctx, params):
    """B.5 — a cordon LINE, not a ring. Boids space themselves across the
    arena on the centre-side of the target, spanning wall-to-wall so the
    runner can't slip around the ends, and advance toward the wall it is
    nearest — shrinking its reachable region until it is pinned in a
    corner. A different paradigm from encircle, strong against perimeter
    runners. Fully decentralized: slot from index, push axis from the
    shared target estimate."""
    bmin, bmax = params['bounds_min'], params['bounds_max']
    tx, ty = ctx.target_xy
    dl, dr = tx - bmin[0], bmax[0] - tx
    db, dt = ty - bmin[1], bmax[1] - ty
    m = min(dl, dr, db, dt)
    frac = ctx.index / max(ctx.n_agents - 1, 1)
    if m in (dl, dr):                          # pin against a side wall
        # vertical cordon spanning y, sitting centre-side of the target
        line_x = tx - 1.6 if m == dr else tx + 1.6
        slot_y = bmin[1] + 0.5 + (bmax[1] - bmin[1] - 1.0) * frac
        goal = (line_x, slot_y)
    else:                                      # pin against top/bottom
        line_y = ty - 1.6 if m == dt else ty + 1.6
        slot_x = bmin[0] + 0.5 + (bmax[0] - bmin[0] - 1.0) * frac
        goal = (slot_x, line_y)
    return _seek(ctx, _clamp_to_arena(goal, params))


def role_encircle(ctx, params):
    """B.6 — asymmetric ring collapse. The boid whose slot lies where the
    target is FLEEING TOWARD holds at a larger radius (a blocker it runs
    into); boids behind press in at a smaller radius to close the gap.
    Captures faster than the uniform ring because it plugs the escape
    while squeezing from the rear."""
    r0 = max(params['ring_radius_min'],
             params['ring_radius_start']
             - params['ring_shrink_rate'] * ctx.t_engaged)
    phi = ctx.target_theta + 2.0 * math.pi * ctx.index / max(ctx.n_agents, 1)
    # how aligned is this slot with the target's flee heading?
    align = math.cos(phi - ctx.target_theta)   # +1 = right in its path
    # blockers (ahead) hold wide; rear boids press in
    radius = r0 * (1.25 if align > 0.3 else 0.7)
    center = _predict(ctx, params, scale=0.5)
    goal = (center[0] + radius * math.cos(phi),
            center[1] + radius * math.sin(phi))
    return _seek(ctx, _clamp_to_arena(goal, params))


def bait(ctx, params):
    """B.7 — leave one ring sector OPEN (a lane toward a corner kill-box)
    so the target flees into it, while the rest of the swarm waits to
    collapse. High-skill, emergent; kept as an explicit strategy."""
    # kill-box = corner nearest the target's flee heading
    lure = (ctx.target_xy[0] + math.cos(ctx.target_theta) * 4.0,
            ctx.target_xy[1] + math.sin(ctx.target_theta) * 4.0)
    killbox = min(_corners(params),
                  key=lambda c: math.hypot(c[0] - lure[0], c[1] - lure[1]))
    open_ang = math.atan2(killbox[1] - ctx.target_xy[1],
                          killbox[0] - ctx.target_xy[0])
    phi = ctx.target_theta + 2.0 * math.pi * ctx.index / max(ctx.n_agents, 1)
    gap = math.atan2(math.sin(phi - open_ang), math.cos(phi - open_ang))
    if abs(gap) < math.radians(35):
        # this slot is in the lane: pull aside to the flank, keep it open
        phi += math.copysign(math.radians(55), gap if gap != 0 else 1.0)
    r = max(params['ring_radius_min'],
            params['ring_radius_start']
            - params['ring_shrink_rate'] * ctx.t_engaged)
    center = _predict(ctx, params, scale=0.5)
    goal = (center[0] + r * math.cos(phi), center[1] + r * math.sin(phi))
    return _seek(ctx, _clamp_to_arena(goal, params))


# ======================================================================
# Terminal commit + automatic strategy selection
# ======================================================================

def terminal_commit(ctx, params, base_vec):
    """Kill the tail-chase orbit. Every ring/lead tactic offsets its aim
    from the target, so once a boid gets close the tangential component
    dominates and it circles the target instead of closing. Inside
    `commit_distance` we override with a straight pounce at the target's
    (barely-led) position — the net collapses instead of orbiting.
    Aggression scales the pounce so close boids commit hard."""
    dx = ctx.target_xy[0] - ctx.self_xy[0]
    dy = ctx.target_xy[1] - ctx.self_xy[1]
    d = math.hypot(dx, dy)
    cd = params.get('commit_distance', 2.5)
    if d >= cd or d < 1e-6:
        return base_vec
    lead = _predict(ctx, params, scale=0.25)   # tiny lead, aim through it
    pounce = _seek(ctx, lead)
    # blend from ring→pounce as distance closes (1 at contact, 0 at cd)
    a = 1.0 - d / cd
    bx = (1 - a) * base_vec[0] + a * pounce[0]
    by = (1 - a) * base_vec[1] + a * pounce[1]
    return unit(bx, by, fallback=pounce)


def auto(ctx, params):
    """Decentralized meta-strategy: each boid picks its tactic from the
    shared belief every cycle, so the swarm adapts without a coordinator.

    - target circling the perimeter  -> blockade (best anti-loop result)
    - target pinned near a corner     -> corner_trap
    - target loose in the open centre -> encircle (surround-then-squeeze)
    """
    if ctx.circling:
        return blockade(ctx, params)
    corner_d = min(math.hypot(ctx.target_xy[0] - c[0],
                              ctx.target_xy[1] - c[1])
                   for c in _corners(params))
    if corner_d < 3.0:
        return corner_trap(ctx, params)
    return encircle(ctx, params)


STRATEGIES = {
    'naive': naive,
    'intercept': intercept,
    'pincer': pincer,
    'encircle': encircle,
    'herd': herd,
    # v4 Part B (M11)
    'counter_rotate': counter_rotate,
    'blockade': blockade,
    'corner_trap': corner_trap,
    'herd_inward': herd_inward,
    # v4 Part B (M12)
    'sweep': sweep,
    'role_encircle': role_encircle,
    'bait': bait,
    # automatic selection
    'auto': auto,
}
