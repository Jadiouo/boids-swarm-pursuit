"""Dominance-region escape planning (behaviors/dominance.py): pure numpy
tests, no ROS."""

import math
import random
import time

import numpy as np
import pytest

from boids_swarm.behaviors.dominance import (
    DominanceConfig, compute_fields, hull_penalty, select_goal, widest_gap)
from boids_swarm.behaviors.escape_map import EscapeMap
from boids_swarm.world_gen import WorldGenerator

BMIN, BMAX = (0.0, 0.0), (20.0, 20.0)
# no turning cost, no margin: the textbook Apollonius setting
PLAIN = DominanceConfig(v_self=3.6, v_purs=2.0, omega=1e9, margin_s=0.0)


def _map(obstacles=(), res=0.1, robot_radius=0.15):
    return EscapeMap(BMIN, BMAX, list(obstacles), resolution=res,
                     robot_radius=robot_radius)


def _xy(m):
    iy, ix = np.mgrid[0:m.h, 0:m.w]
    return (m.origin[0] + (ix + 0.5) * m.res, m.origin[1] + (iy + 0.5) * m.res)


# ------------------------------------------------------------ the fields
def test_single_pursuer_boundary_is_apollonius_circle():
    m = _map()
    e, p = (6.0, 10.0), (14.0, 10.0)
    f = compute_fields(m, e, 0.0, [p], PLAIN)
    X, Y = _xy(m)
    de = np.hypot(X - e[0], Y - e[1])
    dp = np.hypot(X - p[0], Y - p[1])
    truth = de / 3.6 < dp / 2.0           # exact continuous dominance region
    # along the axis the boundary sits at x = 11.14 (d_e = 1.8 d_p)
    row = f.region[m.cell((0, 10.0))[0]]
    xs = X[0][row]
    assert xs.max() == pytest.approx(11.14, abs=0.25)
    # elsewhere the grid metric (8-neighbour, <= 8 % long) only disagrees
    # with the circle in a thin band around it
    gap = np.abs(de / 3.6 - dp / 2.0)
    band = gap < 0.12 * (de / 3.6 + 1e-9)
    bad = (f.region != truth) & m.free
    assert not (bad & ~band).any()
    assert bad.sum() < 0.03 * truth.sum()


def test_region_is_empty_of_cells_the_pursuer_reaches_first():
    m = _map()
    f = compute_fields(m, (6.0, 10.0), 0.0, [(14.0, 10.0)], PLAIN)
    assert not f.region[m.cell((13.0, 10.0))]
    assert f.region[m.cell((3.0, 10.0))]
    assert f.region[m.cell((6.0, 3.0))]


def test_obstacle_shadow_extends_region_behind_obstacle():
    e, p = (8.0, 14.0), (12.0, 6.0)
    free = _map()
    wall = [(10.0, -0.5 + 1.5 * k, 0.9) for k in range(9)]   # x=10, y up to 12.4
    walled = _map(wall)
    f0 = compute_fields(free, e, 0.0, [p], PLAIN)
    f1 = compute_fields(walled, e, 0.0, [p], PLAIN)
    # cells just behind the wall (from the pursuer's side) that the evader
    # reaches first only because the pursuer has to detour around its end
    gained = f1.region & ~f0.region & walled.free
    assert gained.sum() > 50
    X, Y = _xy(walled)
    assert (X[gained] < 10.0).mean() > 0.9


def test_unreachable_cells_are_never_in_the_region():
    ring = [(10.0 + 3.0 * math.cos(a), 10.0 + 3.0 * math.sin(a), 0.7)
            for a in [2 * math.pi * k / 40 for k in range(40)]]
    m = _map(ring)
    f = compute_fields(m, (3.0, 3.0), 0.0, [(17.0, 17.0)], PLAIN)
    assert not f.region[m.cell((10.0, 10.0))]
    assert not np.isfinite(f.t_e[m.cell((10.0, 10.0))])


def test_turning_costs_time_behind_the_evader():
    m = _map()
    cfg = DominanceConfig(margin_s=0.0)
    fa = compute_fields(m, (10.0, 10.0), 0.0, [], cfg)
    ahead = fa.t_e[m.cell((15.0, 10.0))]
    behind = fa.t_e[m.cell((5.0, 10.0))]
    assert behind - ahead == pytest.approx(math.pi / cfg.omega, abs=0.15)


def test_no_pursuers_whole_reachable_floor_is_dominated():
    m = _map()
    f = compute_fields(m, (10.0, 10.0), 0.0, [], PLAIN)
    assert f.region[m.free].all()


# ---------------------------------------------------------- goal choice
def test_goal_lies_in_the_region_with_margin_and_away_from_pursuers():
    m = _map()
    cfg = DominanceConfig()
    e, ps = (8.0, 10.0), [(14.0, 9.0), (14.0, 11.0), (15.0, 10.0)]
    f = compute_fields(m, e, 0.0, ps, cfg)
    c = select_goal(m, f, e, ps, None, cfg)
    assert c.goal is not None and not c.empty
    iy, ix = m.cell(c.goal)
    assert f.region[iy, ix]
    assert f.t_p[iy, ix] - f.t_e[iy, ix] >= cfg.margin_s
    assert c.goal[0] < e[0] + 2.0                   # not toward the swarm


def test_goal_avoids_the_inside_of_the_pursuer_hull():
    m = _map()
    cfg = DominanceConfig(margin_s=0.0)
    # a loose triangle of pursuers with the evader outside near one edge
    ps = [(6.0, 6.0), (14.0, 6.0), (10.0, 14.0), (10.0, 8.0)]
    e = (10.0, 2.5)
    f = compute_fields(m, e, math.pi / 2, ps, cfg)
    c = select_goal(m, f, e, ps, None, cfg)
    from boids_swarm.game import convex_hull, point_in_polygon
    assert not point_in_polygon(c.goal, convex_hull(ps))


def test_hull_penalty_inside_outside():
    X, Y = np.meshgrid(np.array([5.0, 0.0, 20.0]), np.array([5.0]))
    ps = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    pen = hull_penalty(X, Y, ps, soft=2.0)
    assert pen[0, 0] == pytest.approx(1.0)          # centre
    assert pen[0, 2] == pytest.approx(0.0)          # far outside
    assert hull_penalty(X, Y, ps[:2], soft=2.0).max() == 0.0   # no hull


def test_widest_gap_points_into_the_opening():
    ring = [(10.0 + math.cos(a), 10.0 + math.sin(a))
            for a in [2 * math.pi * k / 12 for k in range(3, 12)]]
    g = widest_gap((10.0, 10.0), ring, radius=5.0)
    center, width = g
    assert abs(math.atan2(math.sin(center - math.pi / 6),
                          math.cos(center - math.pi / 6))) < 0.2
    assert width == pytest.approx(math.radians(120), abs=0.1)
    assert widest_gap((10.0, 10.0), ring[:1], radius=5.0) is None


def test_surrounded_region_is_empty_and_goal_breaks_out_through_the_gap():
    m = _map()
    cfg = DominanceConfig()
    e = (10.0, 10.0)
    ps = [(10.0 + 1.0 * math.cos(a), 10.0 + 1.0 * math.sin(a))
          for a in [2 * math.pi * k / 12 for k in range(3, 12)]]
    f = compute_fields(m, e, 0.0, ps, cfg)
    assert not f.region.any()
    c = select_goal(m, f, e, ps, None, cfg)
    assert c.empty and c.goal is not None
    bearing = math.atan2(c.goal[1] - e[1], c.goal[0] - e[0])
    off = math.atan2(math.sin(bearing - math.pi / 6), math.cos(bearing - math.pi / 6))
    assert abs(off) < math.radians(65)              # through the opening (+-60)
    assert math.hypot(c.goal[0] - e[0], c.goal[1] - e[1]) >= cfg.min_goal_dist


def test_hysteresis_goal_does_not_jitter_under_noise():
    m = _map()
    cfg = DominanceConfig()
    rng = random.Random(4)
    base = [(14.0, 5.0), (15.0, 9.0), (14.0, 13.0), (16.0, 16.0)]
    e, goal, switches = (7.0, 10.0), None, 0
    for _ in range(40):
        ps = [(x + rng.gauss(0, 0.15), y + rng.gauss(0, 0.15)) for x, y in base]
        f = compute_fields(m, e, math.pi, ps, cfg)
        c = select_goal(m, f, e, ps, goal, cfg)
        if goal is not None and c.goal != goal:
            switches += 1
        goal = c.goal
    assert switches <= 2


def test_incumbent_is_dropped_when_it_leaves_the_region():
    m = _map()
    cfg = DominanceConfig()
    e = (8.0, 10.0)
    ps = [(16.0, 10.0)]
    f = compute_fields(m, e, 0.0, ps, cfg)
    inc = select_goal(m, f, e, ps, None, cfg).goal
    # pursuers jump next to the incumbent: it can no longer be dominated
    ps2 = [(inc[0] + 1.0, inc[1]), (inc[0], inc[1] + 1.0)]
    f2 = compute_fields(m, e, 0.0, ps2, cfg)
    c = select_goal(m, f2, e, ps2, inc, cfg)
    assert c.switched and c.goal != inc


def test_incumbent_kept_when_still_good_and_reached_one_is_replaced():
    m = _map()
    cfg = DominanceConfig()
    e, ps = (8.0, 10.0), [(16.0, 10.0)]
    f = compute_fields(m, e, 0.0, ps, cfg)
    first = select_goal(m, f, e, ps, None, cfg).goal
    again = select_goal(m, f, e, ps, first, cfg)
    assert again.goal == first and not again.switched
    e2 = first
    f2 = compute_fields(m, e2, 0.0, ps, cfg)
    c = select_goal(m, f2, e2, ps, first, cfg)
    assert c.goal != first                           # reached -> new goal


def test_excluded_discs_are_not_chosen():
    m = _map()
    cfg = DominanceConfig()
    e, ps = (8.0, 10.0), [(16.0, 10.0)]
    f = compute_fields(m, e, 0.0, ps, cfg)
    g0 = select_goal(m, f, e, ps, None, cfg).goal
    g1 = select_goal(m, f, e, ps, None, cfg,
                     exclude=[(g0[0], g0[1], 1.5)]).goal
    assert math.hypot(g1[0] - g0[0], g1[1] - g0[1]) > 1.5


def test_goal_is_free_floor_in_a_crowded_obstacle_field():
    obs = WorldGenerator(3, 20.0).generate('obstacle_field').obstacles
    m = _map(obs, robot_radius=0.35)
    cfg = DominanceConfig()
    rng = random.Random(1)
    for _ in range(15):
        ps = [(rng.uniform(1, 19), rng.uniform(1, 19)) for _ in range(12)]
        e = m.nearest_free((rng.uniform(2, 18), rng.uniform(2, 18)), 2.0)
        if e is None:
            continue
        f = compute_fields(m, e, rng.uniform(-3, 3), ps, cfg)
        c = select_goal(m, f, e, ps, None, cfg)
        if c.goal is not None:
            assert m.free[m.cell(c.goal)]


# --------------------------------------------------------------- timing
def _p95_once():
    obs = WorldGenerator(3, 20.0).generate('obstacle_field').obstacles
    m = _map(obs, res=0.1, robot_radius=0.35)
    assert m.h >= 200 and m.w >= 200
    cfg = DominanceConfig()
    rng = random.Random(2)
    goal, ts = None, []
    for _ in range(60):
        ps = [(rng.uniform(1, 19), rng.uniform(1, 19)) for _ in range(12)]
        e = m.nearest_free((rng.uniform(2, 18), rng.uniform(2, 18)), 2.0)
        if e is None:
            continue
        t0 = time.perf_counter()
        f = compute_fields(m, e, rng.uniform(-3, 3), ps, cfg)
        goal = select_goal(m, f, e, ps, goal, cfg).goal
        ts.append(time.perf_counter() - t0)
    assert len(ts) >= 40
    return np.percentile(ts, 95)


def test_full_recompute_p95_under_25ms_on_200x200():
    # best of 3: a loaded CI box must not flake a latency bound
    assert min(_p95_once() for _ in range(3)) < 0.025
