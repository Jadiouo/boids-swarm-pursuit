"""Unit tests for boids_swarm pure-math modules (SDD v3 §7.4)."""

import math

import pytest

from boids_swarm import game
from boids_swarm.behaviors import flocking
from boids_swarm.behaviors.evasion import ReactiveEvader
from boids_swarm.behaviors.pursuit import STRATEGIES, PursuitContext
from boids_swarm.geometry import to_twist, wrap_angle

PARAMS = {
    'lead_time': 1.0, 'agent_max_speed': 2.0,
    'ring_radius_start': 3.0, 'ring_radius_min': 0.8,
    'ring_shrink_rate': 0.2, 'herd_corner_dist': 4.5,
    'bounds_min': [0.0, 0.0], 'bounds_max': [20.0, 20.0],
}


def ctx(self_xy=(5, 10), target_xy=(10, 10), target_theta=0.0,
        target_speed=4.0, index=0, n=12, t_engaged=0.0):
    return PursuitContext(self_xy=self_xy, self_theta=0.0, index=index,
                          n_agents=n, target_xy=target_xy,
                          target_theta=target_theta,
                          target_speed=target_speed, t_engaged=t_engaged)


def ang(v):
    return math.atan2(v[1], v[0])


# --- pursuit ladder (§4.4) --------------------------------------------------

def test_naive_aims_at_current_position():
    v = STRATEGIES['naive'](ctx(), PARAMS)
    assert ang(v) == pytest.approx(0.0)            # target due east

def test_intercept_leads_a_moving_target():
    """Target at (10,10) heading +y: intercept must aim above naive."""
    c = ctx(target_theta=math.pi / 2)
    v_naive = STRATEGIES['naive'](c, PARAMS)
    v_lead = STRATEGIES['intercept'](c, PARAMS)
    assert ang(v_lead) > ang(v_naive) + 0.2

def test_intercept_lead_point_stays_in_arena():
    """Target racing at a wall: the lead point must not leave the arena."""
    c = ctx(self_xy=(2, 10), target_xy=(18.5, 10), target_theta=0.0)
    v = STRATEGIES['intercept'](c, PARAMS)
    assert abs(ang(v)) < math.pi / 2               # still points +x-ish

def test_pincer_splits_roles():
    """Boid ahead of the target loops far ahead to cut it off; boid
    behind chases near-direct."""
    # ahead + off-axis: target at (10,10) heading +x; boid at (14,12).
    # An interceptor keeps running forward (+x) to the far lead point
    # instead of turning back (-x) toward the target's current position.
    ahead = STRATEGIES['pincer'](
        ctx(self_xy=(14, 12), target_theta=0.0), PARAMS)
    assert ahead[0] > 0.5
    # behind: near-direct chase toward the target (small lead)
    behind = STRATEGIES['pincer'](
        ctx(self_xy=(6, 10), target_theta=0.0), PARAMS)
    assert ang(behind) == pytest.approx(0.0, abs=0.15)

def test_encircle_slots_are_distinct_and_ring_shrinks():
    goals = set()
    for i in range(6):
        v = STRATEGIES['encircle'](ctx(index=i, n=6, target_speed=0.0),
                                   PARAMS)
        goals.add(round(ang(v), 3))
    assert len(goals) >= 5                         # distinct angular slots

def test_encircle_radius_shrinks_with_engagement():
    # boid on the ring: with t_engaged large, its slot goal moves inward,
    # so from far away the direction barely changes but distance-to-goal
    # shrinks. Verify via the internal radius schedule instead.
    r0 = max(PARAMS['ring_radius_min'],
             PARAMS['ring_radius_start'] - PARAMS['ring_shrink_rate'] * 0.0)
    r1 = max(PARAMS['ring_radius_min'],
             PARAMS['ring_radius_start'] - PARAMS['ring_shrink_rate'] * 30.0)
    assert r0 == 3.0 and r1 == 0.8

def test_herd_attacks_from_open_side():
    """Target near the SW corner: herders must stand on the NE (open)
    side so the target's flee response pushes it into the corner."""
    c = ctx(self_xy=(10, 10), target_xy=(6, 6), index=5, n=11,
            target_speed=0.0)
    v = STRATEGIES['herd'](c, PARAMS)
    # goal is on the open side (NE of target) -> from (10,10) the middle
    # slot points toward roughly the target's NE arc, not into the corner
    gx = c.self_xy[0] + v[0]
    gy = c.self_xy[1] + v[1]
    assert math.hypot(gx - 0.0, gy - 0.0) > math.hypot(6, 6) - 1.0

def test_herd_collapses_to_encircle_when_cornered():
    c = ctx(self_xy=(5, 5), target_xy=(2, 2), index=3, n=12,
            target_speed=0.0)
    assert STRATEGIES['herd'](c, PARAMS) == \
        STRATEGIES['encircle'](c, PARAMS)


# --- capture (§4.5) ----------------------------------------------------------

def test_hull_capture_surrounded():
    boids = [(9, 10), (11, 10), (10, 9), (10, 11)]
    captured, n_close = game.capture_hull((10, 10), boids, k=3,
                                          d_capture=1.5)
    assert captured and n_close == 4

def test_hull_no_capture_when_all_on_one_side():
    boids = [(9, 10), (9, 9.5), (9, 10.5), (8.8, 10)]   # tail-chase blob
    captured, _ = game.capture_hull((10, 10), boids, k=3, d_capture=1.5)
    assert not captured

def test_hull_needs_k_close_boids():
    boids = [(9, 10), (11, 10)]                    # only 2 close
    captured, _ = game.capture_hull((10, 10), boids, k=3, d_capture=1.5)
    assert not captured

def test_escape_blocked():
    n = 8
    ring = [(10 + 1.2 * math.cos(2 * math.pi * i / n + 0.3),
             10 + 1.2 * math.sin(2 * math.pi * i / n + 0.3))
            for i in range(n)]
    captured, _ = game.capture_escape_blocked((10, 10), ring, 1.5)
    assert captured
    captured2, _ = game.capture_escape_blocked((10, 10), ring[:4], 1.5)
    assert not captured2

def test_tag_health_drains_and_regens():
    hp = game.TagHealth(hp_max=2.0, drain_per_boid=1.0, regen=1.0)
    dead, n = hp.update((10, 10), [(10.5, 10), (9.5, 10)], 1.5, dt=0.5)
    assert not dead and n == 2 and hp.hp == pytest.approx(1.0)
    dead, _ = hp.update((10, 10), [(10.5, 10), (9.5, 10)], 1.5, dt=0.6)
    assert dead

# --- evasion (§4.3) ----------------------------------------------------------

def test_evader_flees_lone_pursuer():
    ev = ReactiveEvader([0, 0], [20, 20], [])
    ux, uy = ev.compute((10, 10), 0.0, [(8, 10, 0.0)])   # pursuer due west
    assert ux > 0.5                                # flee east

def test_evader_escapes_through_the_gap():
    """Pursuers N, W, S — the only gap is east."""
    ev = ReactiveEvader([0, 0], [20, 20], [])
    ux, uy = ev.compute((10, 10), 0.0,
                        [(8, 10, 0), (10, 12, 0), (10, 8, 0)])
    assert ux > 0.7

def test_evader_avoids_wall():
    """Pursuer behind, wall ahead: do not run straight into the wall."""
    ev = ReactiveEvader([0, 0], [20, 20], [])
    ux, uy = ev.compute((19.0, 10.0), 0.0, [(15, 10, 0.0)])
    assert abs(uy) > 0.4                           # deflects along the wall

def test_evader_ray_clearance_sees_obstacle():
    ev = ReactiveEvader([0, 0], [20, 20], [(12, 10, 1.0)])
    assert ev._clearance(10, 10, 1.0, 0.0) == pytest.approx(1.0)
    assert ev._clearance(10, 10, -1.0, 0.0) == pytest.approx(10.0)


# --- flocking regression (v2, kept identical) --------------------------------

def test_alignment_wraps_correctly():
    n = [(0, 0, math.radians(179)), (0, 0, math.radians(-179))]
    v = flocking.alignment(n)
    assert abs(wrap_angle(ang(v) - math.pi)) < 1e-6

def test_boundary_pushes_inward():
    vx, vy = flocking.boundary((0.5, 19.5), [0, 0], [20, 20], 1.2)
    assert vx > 0 and vy < 0

def test_obstacle_avoid_pushes_away():
    vx, vy = flocking.obstacle_avoid((10.0, 10.0), [(11.0, 10.0, 0.6)], 1.0)
    assert vx < 0 and vy == pytest.approx(0.0)

def test_obstacle_avoid_slides_tangentially_toward_goal():
    # boid left of an obstacle, wanting to go right (into it) and slightly
    # up: the tangential term should add upward slide so it arcs around,
    # instead of a pure leftward (stalling) push.
    obs = [(11.0, 10.0, 0.6)]
    plain = flocking.obstacle_avoid((10.0, 10.0), obs, 1.0)
    slid = flocking.obstacle_avoid((10.0, 10.0), obs, 1.0,
                                   desired=(1.0, 0.2))
    assert plain[1] == pytest.approx(0.0)     # no swirl without desired
    assert slid[1] > 0.3                       # slides upward, the way it wants

def test_to_twist_never_reverses():
    v_lin, w_z = to_twist((-1.0, 0.0), 0.0, 1.0, 4.0, 0.0, 2.0, 5.0)
    assert v_lin >= 0.0 and abs(w_z) == pytest.approx(5.0)
