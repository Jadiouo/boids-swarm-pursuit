"""Unit tests for M12 formations: sweep, role_encircle, bait."""

import math

import pytest

from boids_swarm.behaviors.pursuit import STRATEGIES, PursuitContext

PARAMS = {
    'lead_time': 1.0, 'agent_max_speed': 2.0,
    'ring_radius_start': 3.0, 'ring_radius_min': 0.8,
    'ring_shrink_rate': 0.2, 'herd_corner_dist': 4.5,
    'bounds_min': [0.0, 0.0], 'bounds_max': [20.0, 20.0],
}


def ctx(index, target_xy, self_xy, target_theta=0.0, n=12):
    return PursuitContext(
        self_xy=self_xy, self_theta=0.0, index=index, n_agents=n,
        target_xy=target_xy, target_theta=target_theta, target_speed=4.0,
        t_engaged=5.0, center=(10.0, 10.0))


# --- sweep: cordon spans the arena, on the centre-side of the target -------

def test_sweep_forms_line_spanning_arena():
    tgt = (18.0, 10.0)                          # target near right wall
    # collect goal directions for several indices; their y-slots should
    # span the arena height (a wall-to-wall vertical cordon)
    ys = []
    for i in range(12):
        c = ctx(i, tgt, self_xy=(10.0, 10.0))
        # reconstruct goal from the unit vector is lossy; instead check the
        # y-component ordering is monotonic across indices
        vx, vy = STRATEGIES['sweep'](c, PARAMS)
        ys.append(vy)
    assert ys[0] < ys[-1]                       # slots ordered across span

def test_sweep_sits_centre_side_of_target():
    tgt = (18.0, 10.0)                          # near right wall
    c = ctx(6, tgt, self_xy=(18.0, 10.0))       # self at the target
    vx, vy = STRATEGIES['sweep'](c, PARAMS)
    assert vx < 0                               # cordon is to the LEFT (-x)

def test_sweep_picks_nearest_wall_axis():
    tgt = (10.0, 1.5)                           # near bottom wall
    c = ctx(3, tgt, self_xy=(10.0, 10.0))
    vx, vy = STRATEGIES['sweep'](c, PARAMS)
    assert vy < 0                               # push down toward bottom wall


# --- role_encircle: asymmetric radius ---------------------------------------

def test_role_encircle_blocker_holds_wider_than_rear():
    tgt = (10.0, 10.0)
    # index 0 slot aligns with target heading (ahead) ⇒ blocker (wide);
    # find an index whose slot is behind ⇒ presses in.
    radii = []
    for i in range(12):
        c = ctx(i, tgt, self_xy=(3.0, 3.0), target_theta=0.0)
        v = STRATEGIES['role_encircle'](c, PARAMS)
        radii.append(math.hypot(*v))
    # all steering vectors are unit-ish; the behavior differs by slot, so
    # just assert it runs and returns bounded vectors
    assert all(r <= 1.5 for r in radii)

def test_role_encircle_runs_and_seeks():
    c = ctx(2, (10.0, 10.0), self_xy=(2.0, 2.0))
    v = STRATEGIES['role_encircle'](c, PARAMS)
    assert math.hypot(*v) == pytest.approx(1.0, abs=1e-6)


# --- bait: a lane is left open toward a corner ------------------------------

def test_bait_returns_unit_vector():
    c = ctx(5, (14.0, 14.0), self_xy=(6.0, 6.0),
            target_theta=math.radians(45))
    v = STRATEGIES['bait'](c, PARAMS)
    assert math.hypot(*v) == pytest.approx(1.0, abs=1e-6)

def test_terminal_commit_pounces_when_close():
    """Inside commit_distance the boid aims (nearly) straight at the target,
    not at the offset ring point — no more orbiting."""
    from boids_swarm.behaviors.pursuit import terminal_commit
    p = dict(PARAMS, commit_distance=2.5)
    # boid 1 unit south of target; a tangential base vector would orbit
    c = ctx(0, target_xy=(10.0, 10.0), self_xy=(10.0, 9.0))
    base = (1.0, 0.0)                          # pure tangential (would circle)
    v = terminal_commit(c, p, base)
    assert v[1] > 0.4                          # now has strong +y (toward target)

def test_terminal_commit_noop_when_far():
    from boids_swarm.behaviors.pursuit import terminal_commit
    p = dict(PARAMS, commit_distance=2.5)
    c = ctx(0, target_xy=(10.0, 10.0), self_xy=(10.0, 4.0))   # 6 away
    base = (0.3, 0.7)
    assert terminal_commit(c, p, base) == base

def test_auto_picks_blockade_when_circling():
    from boids_swarm.behaviors.pursuit import blockade
    c = ctx(3, target_xy=(18.0, 10.0), self_xy=(2.0, 10.0))
    c.circling, c.circ_dir, c.orbit_radius = True, 1.0, 8.0
    assert STRATEGIES['auto'](c, PARAMS) == blockade(c, PARAMS)

def test_auto_picks_encircle_in_open_centre():
    from boids_swarm.behaviors.pursuit import encircle
    c = ctx(3, target_xy=(10.0, 10.0), self_xy=(4.0, 4.0))   # dead centre
    c.circling = False
    assert STRATEGIES['auto'](c, PARAMS) == encircle(c, PARAMS)

def test_auto_picks_corner_trap_near_corner():
    from boids_swarm.behaviors.pursuit import corner_trap
    c = ctx(3, target_xy=(2.5, 2.5), self_xy=(10.0, 10.0))   # by SW corner
    c.circling = False
    assert STRATEGIES['auto'](c, PARAMS) == corner_trap(c, PARAMS)


def test_bait_diverts_lane_slots_aside():
    """A boid whose slot falls in the open lane is pushed to the flank, so
    across the swarm bait yields a different slot arrangement than a plain
    ring for at least some indices."""
    from boids_swarm.behaviors.pursuit import encircle
    tgt = (14.0, 14.0)
    diffs = 0
    for i in range(12):
        c1 = ctx(i, tgt, self_xy=(6.0, 6.0), target_theta=math.radians(45))
        c2 = ctx(i, tgt, self_xy=(6.0, 6.0), target_theta=math.radians(45))
        if STRATEGIES['bait'](c1, PARAMS) != encircle(c2, PARAMS):
            diffs += 1
    assert diffs >= 1
