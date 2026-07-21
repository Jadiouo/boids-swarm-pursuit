"""Unit tests for M11 anti-perimeter tactics + circling detector."""

import math

import pytest

from boids_swarm.behaviors.pursuit import STRATEGIES, PursuitContext
from boids_swarm.tracking import CirclingDetector

PARAMS = {
    'lead_time': 1.0, 'agent_max_speed': 2.0,
    'ring_radius_start': 3.0, 'ring_radius_min': 0.8,
    'ring_shrink_rate': 0.2, 'herd_corner_dist': 4.5,
    'bounds_min': [0.0, 0.0], 'bounds_max': [20.0, 20.0],
}
CENTER = (10.0, 10.0)


def circ_ctx(index, target_xy, circ_dir=1.0, circling=True, self_xy=None):
    orbit = math.hypot(target_xy[0] - CENTER[0], target_xy[1] - CENTER[1])
    return PursuitContext(
        self_xy=self_xy or (10.0, 10.0), self_theta=0.0, index=index,
        n_agents=12, target_xy=target_xy, target_theta=math.pi / 2,
        target_speed=4.0, t_engaged=5.0, center=CENTER, circling=circling,
        circ_dir=circ_dir, orbit_radius=orbit)


# --- circling detector ------------------------------------------------------

def test_detector_flags_sustained_perimeter_loop():
    det = CirclingDetector(sustain=1.0, speed_thresh=0.3, perimeter_frac=0.45)
    # target orbits centre at radius 8 (arena half-extent 10), CCW
    circling = False
    for k in range(60):
        t = 0.1 * k
        ang = 0.6 * t                          # ~0.6 rad/s CCW
        xy = (10 + 8 * math.cos(ang), 10 + 8 * math.sin(ang))
        circling, cdir, r = det.update(t, xy, CENTER, 10.0)
    assert circling and cdir == pytest.approx(1.0) and r == pytest.approx(8.0)

def test_detector_ignores_interior_wandering():
    det = CirclingDetector(sustain=1.0, perimeter_frac=0.45)
    circling = True
    for k in range(60):                        # small circle near centre
        t = 0.1 * k
        ang = 0.6 * t
        xy = (10 + 1.0 * math.cos(ang), 10 + 1.0 * math.sin(ang))
        circling, _, _ = det.update(t, xy, CENTER, 10.0)
    assert not circling                        # not near the wall

def test_detector_direction_sign():
    det = CirclingDetector(sustain=1.0, perimeter_frac=0.45)
    cdir = 0.0
    for k in range(60):
        t = 0.1 * k
        ang = -0.7 * t                         # CW
        xy = (10 + 8 * math.cos(ang), 10 + 8 * math.sin(ang))
        _, cdir, _ = det.update(t, xy, CENTER, 10.0)
    assert cdir == pytest.approx(-1.0)


# --- tactics: role split & fallbacks ---------------------------------------

def test_counter_rotate_splits_roles():
    tgt = (10.0, 18.0)                          # target at top, orbit r=8
    chaser = STRATEGIES['counter_rotate'](circ_ctx(0, tgt), PARAMS)   # even
    inter = STRATEGIES['counter_rotate'](circ_ctx(1, tgt,
                                                  self_xy=(2.0, 10.0)),
                                         PARAMS)                       # odd
    assert isinstance(chaser, tuple) and isinstance(inter, tuple)
    # they are different behaviors ⇒ different desired directions
    assert chaser != inter

def test_counter_rotate_falls_back_when_not_circling():
    ctx_c = circ_ctx(1, (10.0, 18.0), circling=True, self_xy=(2.0, 10.0))
    ctx_n = circ_ctx(1, (10.0, 18.0), circling=False, self_xy=(2.0, 10.0))
    from boids_swarm.behaviors.pursuit import encircle
    assert STRATEGIES['counter_rotate'](ctx_n, PARAMS) == \
        encircle(ctx_n, PARAMS)

def test_blockade_stations_lead_boid_ahead_of_target():
    # index 0 is a blocker; it should aim ahead of the target on the ring
    ctx = circ_ctx(0, (18.0, 10.0), circ_dir=1.0, self_xy=(2.0, 10.0))
    v = STRATEGIES['blockade'](ctx, PARAMS)
    assert math.hypot(*v) > 0.1                # heading to its station

def test_blockade_holds_station_when_arrived():
    # place the blocker exactly on its station point ⇒ zero output (plug)
    ctx = circ_ctx(0, (18.0, 10.0), circ_dir=1.0)
    # its station is far ahead on the ring; put self there by searching
    lead_ang = math.atan2(10 - 10, 18 - 10) + 1.0 * 1.6
    station = (10 + 8 * math.cos(lead_ang), 10 + 8 * math.sin(lead_ang))
    ctx.self_xy = station
    assert STRATEGIES['blockade'](ctx, PARAMS) == (0.0, 0.0)

def test_herd_inward_pushes_from_wall_side():
    # target near the top wall; a herder should sit ABOVE it (wall side)
    # so the target flees downward (toward centre)
    ctx = circ_ctx(6, (10.0, 18.0), self_xy=(10.0, 12.0))
    v = STRATEGIES['herd_inward'](ctx, PARAMS)
    # goal is above the target ⇒ desired direction has +y component
    assert v[1] > 0.0

def test_all_tactics_return_unit_ish_vectors():
    for name in ('counter_rotate', 'blockade', 'corner_trap', 'herd_inward'):
        v = STRATEGIES[name](circ_ctx(4, (17.0, 10.0), self_xy=(3.0, 8.0)),
                             PARAMS)
        assert math.hypot(*v) <= 1.5           # bounded steering vector
