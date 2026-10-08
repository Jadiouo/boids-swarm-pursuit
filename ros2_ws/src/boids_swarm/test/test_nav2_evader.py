"""Pure unit tests for the Nav2 evader's decision logic (SDD v3 M7).

No ROS: escape-goal scoring/selection, reactive/Nav2 blend weights, and
obstacle synchronisation between sim, controllers and the Nav2 bridge.
"""

import math
import random

import pytest

from boids_swarm.behaviors.nav2_evader import (
    EscapeConfig, blend_cmd, blend_weight, mode_for, plan_start,
    sample_candidates, score_candidate, select_escape_goal)
from boids_swarm.launch_util import (
    flat_to_obstacles, format_obstacles, obstacles_for_env, parse_obstacles)
from boids_swarm.world_gen import WorldGenerator

BMIN, BMAX = (0.0, 0.0), (20.0, 20.0)
CFG = EscapeConfig()


# ---------------------------------------------------------------- scoring
def test_farther_from_boids_scores_higher():
    boids = [(5.0, 10.0, 0.0)]
    near = score_candidate((8.0, 10.0), (10.0, 10.0), boids, [], BMIN, BMAX, CFG)
    far = score_candidate((16.0, 10.0), (10.0, 10.0), boids, [], BMIN, BMAX, CFG)
    assert far > near


def test_open_space_beats_wall_hugging():
    boids = [(2.0, 10.0, 0.0)]
    wall = score_candidate((19.6, 10.0), (10.0, 10.0), boids, [], BMIN, BMAX, CFG)
    open_ = score_candidate((15.0, 10.0), (10.0, 10.0), boids, [], BMIN, BMAX, CFG)
    # same-ish boid distance class, but the wall candidate has no clearance
    assert open_ > wall


def test_obstacle_adjacent_candidate_loses_openness():
    obs = [(14.0, 10.0, 1.5)]
    boids = [(2.0, 10.0, 0.0)]
    hugging = score_candidate((15.9, 10.0), (10.0, 10.0), boids, obs, BMIN, BMAX, CFG)
    clear = score_candidate((10.0, 15.0), (10.0, 10.0), boids, obs, BMIN, BMAX, CFG)
    assert clear > hugging


def test_crossing_boids_is_penalised():
    self_xy = (10.0, 10.0)
    boids = [(13.0, 10.0, 0.0)]
    # both candidates are 6 m from self; the east one needs to pass the boid
    east = (16.0, 10.0)
    north = (10.0, 16.0)
    s_east = score_candidate(east, self_xy, boids, [], BMIN, BMAX, CFG)
    s_north = score_candidate(north, self_xy, boids, [], BMIN, BMAX, CFG)
    assert s_north > s_east


def test_invalid_candidates_are_rejected():
    obs = [(10.0, 10.0, 2.0)]
    assert score_candidate((10.5, 10.0), (3, 3), [], obs, BMIN, BMAX, CFG) == -math.inf
    assert score_candidate((-1.0, 5.0), (3, 3), [], [], BMIN, BMAX, CFG) == -math.inf


def test_no_boids_scores_are_finite():
    s = score_candidate((12.0, 12.0), (5.0, 5.0), [], [], BMIN, BMAX, CFG)
    assert math.isfinite(s)


# ---------------------------------------------------------------- sampling
def test_samples_are_in_free_space_and_deterministic():
    obs = [(10.0, 10.0, 3.0), (4.0, 15.0, 1.5)]
    a = sample_candidates(random.Random(1), 80, BMIN, BMAX, obs, CFG)
    b = sample_candidates(random.Random(1), 80, BMIN, BMAX, obs, CFG)
    assert a == b and len(a) == 80
    for (x, y) in a:
        assert CFG.wall_margin <= x <= 20 - CFG.wall_margin
        assert CFG.wall_margin <= y <= 20 - CFG.wall_margin
        for (cx, cy, r) in obs:
            assert math.hypot(x - cx, y - cy) >= r + CFG.obstacle_margin - 1e-9


def test_select_goal_runs_away_from_cluster():
    boids = [(5.0, 9.0, 0.0), (5.0, 11.0, 0.0), (6.0, 10.0, 0.0)]
    goal, _ = select_escape_goal(random.Random(3), (7.0, 10.0), boids, [],
                                 BMIN, BMAX, None, CFG)
    # farther from the cluster centroid than we are now
    assert math.hypot(goal[0] - 5.3, goal[1] - 10.0) > math.hypot(7 - 5.3, 0) + 4


def test_select_goal_avoids_dead_end_pocket():
    # a U-shaped pocket of obstacles open toward the boids on the west
    obs = [(16.0, 6.0, 1.5), (16.0, 10.0, 1.5), (16.0, 14.0, 1.5)]
    boids = [(2.0, 10.0, 0.0)]
    goal, _ = select_escape_goal(random.Random(5), (9.0, 10.0), boids, obs,
                                 BMIN, BMAX, None, CFG)
    for (cx, cy, r) in obs:
        assert math.hypot(goal[0] - cx, goal[1] - cy) >= r + CFG.obstacle_margin - 1e-9


def test_goal_hysteresis_keeps_incumbent():
    boids = [(5.0, 10.0, 0.0)]
    rng = random.Random(9)
    first, s1 = select_escape_goal(rng, (10.0, 10.0), boids, [], BMIN, BMAX,
                                   None, CFG)
    # an incumbent only slightly worse than the best must be kept
    kept, _ = select_escape_goal(random.Random(10), (10.0, 10.0), boids, [],
                                 BMIN, BMAX, first, CFG)
    assert kept == first


def test_goal_dropped_when_incumbent_becomes_bad():
    first = (16.0, 10.0)
    boids = [(15.0, 10.0, 0.0)]          # boid now sits on the old goal
    goal, _ = select_escape_goal(random.Random(2), (10.0, 10.0), boids, [],
                                 BMIN, BMAX, first, CFG)
    assert goal != first


# ---------------------------------------------------------------- blending
def test_blend_weight_endpoints_and_linearity():
    assert blend_weight(10.0, 4.0, 1.5) == 0.0
    assert blend_weight(4.0, 4.0, 1.5) == 0.0
    assert blend_weight(1.5, 4.0, 1.5) == 1.0
    assert blend_weight(0.2, 4.0, 1.5) == 1.0
    assert blend_weight(2.75, 4.0, 1.5) == pytest.approx(0.5)
    xs = [blend_weight(d, 4.0, 1.5) for d in (3.8, 3.0, 2.2, 1.6)]
    assert xs == sorted(xs)


def test_blend_weight_degenerate_radius():
    assert blend_weight(1.0, 1.0, 1.5) in (0.0, 1.0)


def _blend(nav2, react, react_heading, w_r, theta=0.0, speed=3.0):
    return blend_cmd(theta, nav2, react, react_heading, w_r, speed_cap=speed,
                     w_max=1.2)


def test_blend_endpoints_are_pure_sources():
    nav2, react = (3.0, 0.4), (1.0, -1.0)
    assert _blend(nav2, react, -0.5, 0.0) == pytest.approx(nav2)
    assert _blend(nav2, react, -0.5, 1.0) == pytest.approx(react)


def test_blend_opposite_turns_do_not_cancel_into_straight_driving():
    """Linear (v, w) averaging turned 'nav2 left + reactive right' into ~0
    rad/s, i.e. driving straight at the boid. Heading-space blending keeps
    a definite direction: it must lie between the two headings and follow
    the dominant source."""
    nav2 = (3.0, 1.2)                  # curving left
    react_heading = -1.0               # reactive wants to go right
    v_hi, w_hi = _blend(nav2, (3.0, -1.2), react_heading, 0.75)
    v_lo, w_lo = _blend(nav2, (3.0, -1.2), react_heading, 0.25)
    assert w_hi < 0.0 < w_lo           # follows the dominant side
    assert v_hi > 0.5 and v_lo > 0.5


def test_blend_is_left_right_symmetric():
    a = _blend((3.0, 1.0), (3.0, -1.2), -1.0, 0.4)
    b = _blend((3.0, -1.0), (3.0, 1.2), 1.0, 0.4)
    assert a[0] == pytest.approx(b[0])
    assert a[1] == pytest.approx(-b[1])


def test_blend_head_on_conflict_defers_to_reactive_safety():
    # nav2 wants to go straight at the boid's side while reactive flees the
    # opposite way: vectors cancel -> reactive heading wins, never a stall
    v, w = _blend((3.0, 0.0), (3.0, 1.2), math.pi - 1e-3, 0.5)
    assert v > 0.5
    assert w == pytest.approx(1.2)          # turning away like reactive


def test_mode_classification():
    assert mode_for(0.0, True) == 'nav2'
    assert mode_for(0.4, True) == 'blend'
    assert mode_for(1.0, True) == 'reactive'
    assert mode_for(0.0, False) == 'reactive'    # Nav2 unavailable -> fall back
    assert mode_for(0.4, False) == 'reactive'


# ---------------------------------------------------------- obstacle sync
def test_format_parse_roundtrip_is_exact():
    flat = WorldGenerator(7).generate('obstacle_field').obstacles_flat()
    assert parse_obstacles(format_obstacles(flat)) == flat


def test_empty_obstacles_use_sentinel():
    assert parse_obstacles('') == [0.0]
    assert format_obstacles([0.0]) == ''
    assert flat_to_obstacles([0.0]) == []


@pytest.mark.parametrize('seed', [1, 7, 99])
@pytest.mark.parametrize('env', ['open', 'obstacle_field', 'pillar'])
def test_sim_and_bridge_get_identical_obstacles(seed, env):
    """The launch hands ONE flat list to sim, controllers and (formatted)
    to the Nav2 bridge; the bridge's parse must reproduce it exactly."""
    flat = obstacles_for_env(env, seed, 20.0, '')
    layout = WorldGenerator(seed, 20.0).generate(env)
    assert flat == layout.obstacles_flat()
    assert parse_obstacles(format_obstacles(flat)) == flat


def test_custom_env_uses_string_argument():
    assert obstacles_for_env('custom', 7, 20.0, '5,5,1.5;15,12,2') == \
        [5.0, 5.0, 1.5, 15.0, 12.0, 2.0]


# ------------------------------------------------- limits from params
def test_target_limits_derive_from_params_not_hardcoded():
    from boids_swarm.launch_util import target_limits
    v, w, r = target_limits({'agent_max_speed': 2.0,
                             'target_speed_multiplier': 1.8,
                             'target_omega_max': 1.2,
                             'target_body_radius': 0.15})
    assert (v, w, r) == pytest.approx((3.6, 1.2, 0.15))
    v2, _, _ = target_limits({'agent_max_speed': 2.0,
                              'target_speed_multiplier': 1.8},
                             {'target_speed_multiplier': 2.0})
    assert v2 == pytest.approx(4.0)


def test_shipped_params_give_3_6_mps():
    import os
    from boids_swarm.launch_util import load_ros_params, target_limits
    here = os.path.dirname(os.path.abspath(__file__))
    params = load_ros_params(os.path.join(here, '..', 'config', 'params.yaml'))
    v, w, r = target_limits(params)
    assert v == pytest.approx(3.6) and w == pytest.approx(1.2)


# ------------------------------------------------- planner start pose
def test_plan_start_unchanged_in_open_space():
    assert plan_start((10.0, 10.0), BMIN, BMAX, [], 0.4) == (10.0, 10.0)


def test_plan_start_pulled_off_the_wall():
    # the target may stand against a wall (centre 0.15 away); the planner's
    # costmap treats that as inscribed, so the plan starts a margin inside
    x, y = plan_start((7.5, 0.15), BMIN, BMAX, [], 0.4)
    assert y >= 0.4 - 1e-9 and abs(x - 7.5) < 0.5
    x, y = plan_start((19.9, 19.9), BMIN, BMAX, [], 0.4)
    assert x <= 19.6 + 1e-9 and y <= 19.6 + 1e-9


def test_plan_start_pushed_off_obstacles():
    obs = [(10.0, 10.0, 2.0)]
    x, y = plan_start((12.15, 10.0), BMIN, BMAX, obs, 0.4)   # touching body
    assert math.hypot(x - 10.0, y - 10.0) >= 2.4 - 1e-9


def test_plan_start_is_never_far_from_the_robot():
    obs = [(10.0, 10.0, 2.0)]
    for p in [(0.1, 0.1), (12.1, 10.0), (19.95, 5.0), (10.0, 10.0)]:
        q = plan_start(p, BMIN, BMAX, obs, 0.4)
        assert math.hypot(q[0] - p[0], q[1] - p[1]) <= 0.7 + 1e-9


def test_plan_start_in_a_narrow_gap_picks_the_best_effort_point():
    # 0.5 m gap between the wall (y=0) and an obstacle: no point reaches the
    # 0.4 margin, so take the widest spot rather than failing outright
    obs = [(10.0, 3.1, 2.6)]
    q = plan_start((10.0, 0.15), BMIN, BMAX, obs, 0.4)
    assert _clearance_of(q, obs) > _clearance_of((10.0, 0.15), obs)


def _clearance_of(p, obs):
    x, y = p
    d = min(x, y, 20 - x, 20 - y)
    for cx, cy, r in obs:
        d = min(d, math.hypot(x - cx, y - cy) - r)
    return d


# ------------------------------------------------- footprint consistency (A)
def test_costmap_radius_is_derived_from_target_body_radius():
    import os
    import yaml
    from boids_swarm.launch_util import (load_ros_params, target_limits,
                                         with_robot_radius)
    pkg = os.path.join(os.path.dirname(__file__), '..')
    params = load_ros_params(os.path.join(pkg, 'config', 'params.yaml'))
    body_r = target_limits(params)[2]
    assert body_r == pytest.approx(0.15)
    for name in ('nav2_target.yaml', 'nav2_target_mppi.yaml'):
        with open(os.path.join(pkg, 'config', name)) as f:
            doc = yaml.safe_load(f)
        out = with_robot_radius(doc, 0.4)
        assert out['global_costmap']['global_costmap']['ros__parameters'][
            'robot_radius'] == 0.4
        assert out['local_costmap']['local_costmap']['ros__parameters'][
            'robot_radius'] == 0.4
        assert doc is not out          # input untouched
        # the shipped placeholders already agree with the body
        for sect in ('global_costmap', 'local_costmap'):
            assert doc[sect][sect]['ros__parameters']['robot_radius'] == \
                pytest.approx(body_r)


# ------------------------------------------------ failure handling (B)
from boids_swarm.behaviors.nav2_evader import (   # noqa: E402
    GoalBlacklist, PlanTracker, should_request_plan)


def test_blacklist_blocks_nearby_goals_until_expiry():
    bl = GoalBlacklist(ttl=5.0, radius=1.0)
    bl.add((10.0, 10.0), now=100.0)
    assert bl.blocked((10.5, 10.0), 101.0)
    assert not bl.blocked((12.0, 10.0), 101.0)
    assert not bl.blocked((10.0, 10.0), 105.1)       # expired
    assert bl.active(105.1) == []


def test_blacklist_repeat_failures_extend_ttl_and_cap_entries():
    bl = GoalBlacklist(ttl=5.0, radius=1.0, max_entries=3)
    bl.add((5.0, 5.0), 0.0)
    bl.add((5.2, 5.0), 3.0)                  # same spot again: one entry, ttl*2
    assert len(bl.active(3.0)) == 1
    assert bl.blocked((5.0, 5.0), 3.0 + 9.9)
    assert not bl.blocked((5.0, 5.0), 3.0 + 10.1)
    for i in range(6):
        bl.add((2.0 * i + 8.0, 15.0), 20.0 + i)
    assert len(bl.active(26.0)) <= 3


def test_plan_tracker_ignores_late_result_after_timeout():
    t = PlanTracker(timeout=2.0)
    s1 = t.begin(0.0)
    assert t.pending
    assert t.expire(1.0) is None             # not yet
    assert t.expire(2.5) == s1               # timed out -> caller cancels
    assert not t.pending
    assert not t.accept(s1)                  # the late result is dropped
    assert t.expire(9.0) is None             # and not expired twice


def test_plan_tracker_only_newest_request_is_accepted():
    t = PlanTracker(timeout=2.0)
    s1 = t.begin(0.0)
    s2 = t.begin(0.1)
    assert not t.accept(s1)
    assert t.accept(s2)
    t.close(s2)
    assert not t.pending
    assert not t.accept(s2)                  # a duplicate delivery


def test_no_plan_requests_while_in_fallback_hold():
    kw = dict(changed=True, stale=True, nav2_ok=False)
    assert should_request_plan(now=10.0, fallback_until=11.0, **kw) is False
    assert should_request_plan(now=11.0, fallback_until=11.0, **kw) is True
    assert should_request_plan(now=20.0, fallback_until=11.0, changed=False,
                               stale=False, nav2_ok=True) is False
    assert should_request_plan(now=20.0, fallback_until=11.0, changed=False,
                               stale=False, nav2_ok=False) is True
