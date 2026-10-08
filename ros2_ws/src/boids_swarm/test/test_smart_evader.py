"""Pure unit tests for SmartEvader (evader:=smart). No ROS, no Nav2."""

import math
import random
import time

import pytest

from boids_swarm.behaviors.smart_evader import SmartConfig, SmartEvader
from boids_swarm.world_gen import WorldGenerator

BMIN, BMAX = (0.0, 0.0), (20.0, 20.0)


def run(ev, xy, theta, pursuers, steps=12):
    """Call compute repeatedly (the strategy layer is amortised over a few
    calls) and return the last output."""
    u = None
    for _ in range(steps):
        u = ev.compute(xy, theta, pursuers)
    return u


def _purs(*pts):
    return [(x, y, 0.0) for x, y in pts]


def test_output_is_unit_and_safe_without_pursuers():
    ev = SmartEvader(BMIN, BMAX, [])
    u = run(ev, (10.0, 10.0), 0.3, [])
    assert math.hypot(*u) == pytest.approx(1.0, abs=1e-6)


def test_corner_start_heads_out_not_into_corner():
    ev = SmartEvader(BMIN, BMAX, [])
    # pursuer sits along the bottom wall; the way out is up the left wall
    u = run(ev, (0.4, 0.4), math.pi * 1.25, _purs((3.5, 0.9), (2.5, 3.5)))
    assert u[0] > -0.05 and u[1] > -0.05          # not into either wall
    # whatever it picks, it must leave the corner (positive progress)
    assert u[0] + u[1] > 0.5


@pytest.mark.parametrize('xy,inward', [
    ((0.2, 10.0), (1.0, 0.0)),
    ((19.8, 10.0), (-1.0, 0.0)),
    ((10.0, 0.2), (0.0, 1.0)),
    ((10.0, 19.8), (0.0, -1.0)),
])
def test_wall_hugging_output_never_points_into_wall(xy, inward):
    ev = SmartEvader(BMIN, BMAX, [])
    # facing the wall, pursuer coming from the arena side: greedy flee = wall
    th = math.atan2(-inward[1], -inward[0])
    far = (xy[0] + 5 * inward[0], xy[1] + 5 * inward[1])
    u = run(ev, xy, th, _purs(far))
    assert u[0] * inward[0] + u[1] * inward[1] > -0.05
    # and it slides along the wall (tangent component), not just stands off
    tang = abs(u[0] * inward[1] - u[1] * inward[0])
    assert tang > 0.5


def test_hugging_obstacle_surface_never_points_into_it():
    ev = SmartEvader(BMIN, BMAX, [(10.0, 10.0, 2.0)])
    xy = (12.2, 10.0)                              # 0.2 off the surface
    u = run(ev, xy, math.pi, _purs((16.0, 10.0)))
    assert u[0] > -0.05
    assert abs(u[1]) > 0.5


def test_flanked_prefers_geodesic_exit_over_ramming_wall():
    # a barrier of discs at x=7 closes the left half except north of y=15;
    # pursuers west and south push the evader toward it. Straight flight
    # east hits the barrier; the real exit is north.
    barrier = [(7.0, y, 1.2) for y in (1.0, 3.2, 5.4, 7.6, 9.8, 12.0)]
    ev = SmartEvader(BMIN, BMAX, barrier)
    # heading north-east (the vehicle cannot pivot: 1.2 rad/s)
    u = run(ev, (4.0, 8.0), 0.8, _purs((0.8, 8.0), (4.0, 2.5)), steps=15)
    # the chosen escape point is round the barrier's open end (north) or
    # on this side of it, never in the walled-off pocket east of it
    gx, gy = ev.goal
    assert gy > 14.0 or gx < 5.5
    assert u[1] > 0.4
    # and it is not pushing east into the barrier head-on
    assert u[0] < 0.9


def test_flanked_in_open_runs_through_the_free_side():
    ev = SmartEvader(BMIN, BMAX, [])
    u = run(ev, (3.0, 10.0), 0.0, _purs((3.0, 14.0), (3.0, 6.0)))
    assert u[0] > 0.3                              # away from the wall side
    assert ev.goal[0] > 8.0


NO_UNSTICK = SmartConfig(stuck_steps=10 ** 6)   # fixed-pose tests stand still


def test_decision_is_stable_for_identical_input():
    ev = SmartEvader(BMIN, BMAX, [(10.0, 10.0, 2.0), (5.0, 14.0, 1.5)],
                     NO_UNSTICK)
    purs = _purs((4.0, 8.0), (6.0, 6.0), (8.0, 4.0))
    run(ev, (7.0, 9.0), 0.4, purs, steps=10)
    goals, dirs = [], []
    for _ in range(90):                            # 3 s of control
        dirs.append(ev.compute((7.0, 9.0), 0.4, purs))
        goals.append(ev.goal)
    assert len(set(goals)) == 1
    ang = [math.atan2(u[1], u[0]) for u in dirs]
    assert max(abs(a - ang[0]) for a in ang) < 0.05


def test_hysteresis_keeps_goal_under_small_pursuer_jitter():
    ev = SmartEvader(BMIN, BMAX, [(10.0, 10.0, 2.0)], NO_UNSTICK)
    rng = random.Random(3)
    base = [(3.0, 3.0), (4.0, 6.0), (6.0, 3.0)]
    run(ev, (8.0, 8.0), 0.0, _purs(*base), steps=10)
    switches, last = 0, ev.goal
    for _ in range(150):
        purs = _purs(*[(x + rng.uniform(-.1, .1), y + rng.uniform(-.1, .1))
                       for x, y in base])
        ev.compute((8.0, 8.0), 0.0, purs)
        if ev.goal != last:
            switches, last = switches + 1, ev.goal
    assert switches <= 1


def test_stuck_in_corner_triggers_escape_toward_open_space():
    ev = SmartEvader(BMIN, BMAX, [])
    ev.compute((0.15, 0.15), 0.0, _purs((6.0, 6.0)))
    for _ in range(ev.cfg.stuck_steps + 5):
        u = ev.compute((0.15, 0.15), math.pi * 1.25, _purs((6.0, 6.0)))
    assert ev.escaping
    assert u[0] > -0.05 and u[1] > -0.05


def _p95_once():
    layout = WorldGenerator(1, 20.0).generate('obstacle_field')
    ev = SmartEvader(BMIN, BMAX, layout.obstacles)
    rng = random.Random(1)
    pur = [[rng.uniform(1, 19), rng.uniform(1, 19)] for _ in range(12)]
    x, y, th = 10.0, 10.0, 0.0
    ts = []
    for k in range(400):
        for p in pur:
            p[0] = min(19.0, max(1.0, p[0] + rng.uniform(-.1, .1)))
            p[1] = min(19.0, max(1.0, p[1] + rng.uniform(-.1, .1)))
        t0 = time.perf_counter()
        u = ev.compute((x, y), th, [(a, b, 0.0) for a, b in pur])
        ts.append(time.perf_counter() - t0)
        th = math.atan2(u[1], u[0])
        x = min(19.0, max(1.0, x + u[0] * 0.06))
        y = min(19.0, max(1.0, y + u[1] * 0.06))
    ts.sort()
    return ts[int(0.95 * len(ts))]


def test_compute_time_p95_under_5ms():
    # best of three: a shared CI / dev box can stall one trial (the number
    # is a property of the code, not of whatever else is running)
    assert min(_p95_once() for _ in range(3)) < 0.005


def test_config_from_params_overrides():
    cfg = SmartConfig.from_params({'smart_decision_hz': 3.0,
                                   'smart_candidates': 17,
                                   'smart_w_lead': 2.5})
    assert cfg.decision_hz == 3.0 and cfg.candidates == 17
    assert cfg.w_lead == 2.5
