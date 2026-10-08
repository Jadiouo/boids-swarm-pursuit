"""Reachability-aware escape scoring (SDD v3 M7, red-team item C): pure
numpy tests, no ROS."""

import math
import time

import pytest

from boids_swarm.behaviors.escape_map import (
    EscapeMap, ReachConfig, lead_margin, pocket_penalty, reach_adjustment)
from boids_swarm.behaviors.nav2_evader import EscapeConfig, select_escape_goal
from boids_swarm.world_gen import WorldGenerator

import random

BMIN, BMAX = (0.0, 0.0), (20.0, 20.0)
RC = ReachConfig()


# ------------------------------------------------------- pure score parts
def test_lead_margin_signs():
    assert lead_margin(5.0, 20.0, 0.6) == pytest.approx(17.0)    # we are first
    assert lead_margin(10.0, 4.0, 0.6) == pytest.approx(-2.0)    # they are first
    assert lead_margin(math.inf, 4.0, 0.6) == -math.inf          # unreachable
    assert lead_margin(5.0, math.inf, 0.6) == math.inf


def test_pocket_penalty_orders_open_above_pocket():
    open_ = pocket_penalty(14.0, 2.0, RC)         # half a 3 m disc is ~14 m2
    wall = pocket_penalty(5.0, 0.3, RC)
    deadend = pocket_penalty(0.5, 0.3, RC)
    assert open_ == pytest.approx(0.0, abs=0.05)
    assert open_ < wall < deadend <= 1.0


def test_unreachable_is_rejected_and_losing_race_is_worse():
    assert reach_adjustment(math.inf, 5.0, 0.9, 2.0, RC) == -math.inf
    win = reach_adjustment(5.0, 20.0, 0.9, 2.0, RC)
    lose = reach_adjustment(10.0, 4.0, 0.9, 2.0, RC)
    assert win > lose


# ------------------------------------------------------------- map / BFS
def _walled_pocket():
    """A U of overlapping discs opening to the west around (14,10)."""
    return [(16.0 + 0.0, 6.0 + 0.5 * i, 0.8) for i in range(0, 17, 2)] + \
           [(13.0 + i * 0.5, 6.0, 0.8) for i in range(0, 7)] + \
           [(13.0 + i * 0.5, 14.0, 0.8) for i in range(0, 7)]


def test_enclosed_room_is_unreachable_open_room_is_reachable():
    ring = [(10.0 + 3.0 * math.cos(a), 10.0 + 3.0 * math.sin(a), 0.7)
            for a in [2 * math.pi * k / 40 for k in range(40)]]
    m = EscapeMap(BMIN, BMAX, ring)
    ctx = m.context((3.0, 3.0), [(17.0, 17.0)])
    ds_in, *_ = ctx.terms((10.0, 10.0))
    ds_out, *_ = ctx.terms((15.0, 3.0))
    assert math.isinf(ds_in)                       # sealed inside the ring
    assert math.isfinite(ds_out)
    assert reach_adjustment(*ctx.terms((10.0, 10.0)), RC) == -math.inf


def test_geodesic_detours_around_a_wall_euclid_would_cross():
    wall = [(10.0, 2.0 + 0.6 * i, 0.7) for i in range(24)]   # y 2..15.8, gap above
    m = EscapeMap(BMIN, BMAX, wall)
    ctx = m.context((7.0, 8.0), [])
    ds, *_ = ctx.terms((13.0, 8.0))
    assert ds > 6.0 * 1.8       # euclid is 6 m; must go over the top


def test_pursuer_behind_wall_is_far_in_geodesic_not_euclid():
    wall = [(10.0, 2.0 + 0.6 * i, 0.7) for i in range(24)]
    m = EscapeMap(BMIN, BMAX, wall)
    ctx = m.context((13.0, 5.0), [(7.0, 5.0)])     # 6 m apart through the wall
    ds, dp, *_ = ctx.terms((13.0, 5.0))
    assert dp > 10.0                              # euclid is 6


def test_wall_hugging_start_snaps_to_a_traversable_cell():
    m = EscapeMap(BMIN, BMAX, [])
    x, y = m.plan_start((0.15, 10.0))
    assert m.free[m.cell((x, y))]
    assert math.hypot(x - 0.15, y - 10.0) <= 0.2
    assert m.plan_start((10.0, 10.0)) == (10.0, 10.0)


# --------------------------------------------------- selection behaviour
def test_goal_not_placed_in_sealed_pocket_even_if_far_from_boids():
    ring = [(14.0 + 2.2 * math.cos(a), 10.0 + 2.2 * math.sin(a), 0.6)
            for a in [2 * math.pi * k / 40 for k in range(40)]]
    m = EscapeMap(BMIN, BMAX, ring)
    boids = [(2.0, 10.0, 0.0)]
    cfg = EscapeConfig(n_candidates=200)
    for seed in range(8):
        goal, _ = select_escape_goal(random.Random(seed), (8.0, 10.0), boids,
                                     ring, BMIN, BMAX, None, cfg, emap=m)
        assert math.hypot(goal[0] - 14.0, goal[1] - 10.0) > 2.2 + 0.6


def test_dead_end_alcove_scores_below_open_ground():
    # alcove open to the west (toward the pursuer), 2.4 m wide
    obs = [(13.0 + 0.5 * i, 8.0, 0.8) for i in range(0, 11)] \
        + [(13.0 + 0.5 * i, 12.0, 0.8) for i in range(0, 11)] \
        + [(18.2, 8.0 + 0.5 * i, 0.8) for i in range(0, 9)]
    m = EscapeMap(BMIN, BMAX, obs)
    ctx = m.context((9.0, 10.0), [(2.0, 10.0)])
    ds, dp, area, cl = ctx.terms((17.0, 10.0), pocket=True)
    ds2, dp2, area2, cl2 = ctx.terms((12.0, 16.5), pocket=True)
    assert area < 0.5 * area2, (area, area2)
    pocket = ctx.adjustment((17.0, 10.0), pocket=True)
    open_ = ctx.adjustment((12.0, 16.5), pocket=True)
    assert open_ > pocket


def test_blacklisted_goals_are_excluded_by_selection():
    boids = [(2.0, 10.0, 0.0)]
    cfg = EscapeConfig()
    g1, _ = select_escape_goal(random.Random(1), (8.0, 10.0), boids, [],
                               BMIN, BMAX, None, cfg)
    g2, _ = select_escape_goal(random.Random(1), (8.0, 10.0), boids, [],
                               BMIN, BMAX, None, cfg,
                               blocked=lambda p: math.hypot(
                                   p[0] - g1[0], p[1] - g1[1]) < 1.0)
    assert math.hypot(g2[0] - g1[0], g2[1] - g1[1]) >= 1.0


# ------------------------------------------------------------- timing
def test_200x200_geodesic_budget():
    obs = WorldGenerator(3, 20.0).generate('obstacle_field').obstacles
    t0 = time.perf_counter()
    m = EscapeMap(BMIN, BMAX, obs)
    t_build = time.perf_counter() - t0
    assert (m.w, m.h) == (202, 202)
    pursuers = [(3.0 + i, 18.0) for i in range(12)]
    m.context((10.0, 10.0), pursuers)                    # warm
    t0 = time.perf_counter()
    n = 10
    for _ in range(n):
        m.context((10.0, 10.0), pursuers)
    per = (time.perf_counter() - t0) / n
    print(f'build {t_build*1e3:.1f} ms, context (2 fields) {per*1e3:.1f} ms')
    assert per < 0.030, f'{per*1e3:.1f} ms per decision (2 geodesic fields)'


def test_full_goal_decision_budget_under_30ms():
    obs = WorldGenerator(3, 20.0).generate('obstacle_field').obstacles
    m = EscapeMap(BMIN, BMAX, obs)
    boids = [(3.0 + i, 18.0, 0.0) for i in range(12)]
    rng = random.Random(1)
    cfg = EscapeConfig()
    select_escape_goal(rng, (10.0, 10.0), boids, obs, BMIN, BMAX, None, cfg,
                       emap=m)
    n, t0 = 10, time.perf_counter()
    for _ in range(n):
        select_escape_goal(rng, (10.0, 10.0), boids, obs, BMIN, BMAX, None,
                           cfg, emap=m)
    per = (time.perf_counter() - t0) / n
    print(f'select_escape_goal with map: {per*1e3:.1f} ms')
    assert per < 0.030
