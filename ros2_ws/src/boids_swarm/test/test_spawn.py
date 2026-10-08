"""Safe target spawn (spawn.py): pure functions, no ROS."""
import math
import random

from boids_swarm import spawn
from boids_swarm.game import capture_hull, capture_escape_blocked
from boids_swarm.world_gen import WorldGenerator

WORLD = 20.0


def _swarm(seed, n=12, obstacles=()):
    rng = random.Random(seed)
    out = []
    while len(out) < n:
        p = (rng.uniform(1.5, WORLD - 1.5), rng.uniform(1.5, WORLD - 1.5))
        if not spawn.in_obstacle(p[0], p[1], obstacles):
            out.append(p)
    return out


def test_clearance_met_and_outside_obstacles():
    for seed in range(1, 11):
        obs = list(WorldGenerator(seed, WORLD).generate(
            'obstacle_field').obstacles)
        sw = _swarm(seed, obstacles=obs)
        x, y, ok, clr = spawn.choose_target_spawn(
            sw, obs, WORLD, random.Random(seed), 5.0)
        assert not spawn.in_obstacle(x, y, obs)
        assert abs(clr - spawn.clearance(x, y, sw)) < 1e-9
        if ok:
            assert clr >= 5.0


def test_never_capturable_at_t0():
    d_cap = 1.5
    for seed in range(1, 31):
        sw = _swarm(seed)
        x, y, ok, _ = spawn.choose_target_spawn(
            sw, [], WORLD, random.Random(seed), 4.0)   # > d_cap
        assert ok
        for k in (3, 4, 6):
            assert not capture_hull((x, y), sw, k, d_cap)[0]
        assert not capture_escape_blocked((x, y), sw, d_cap)[0]


def test_reproducible_and_seed_dependent():
    sw = _swarm(3)
    a = spawn.choose_target_spawn(sw, [], WORLD, random.Random(9), 6.0)
    b = spawn.choose_target_spawn(sw, [], WORLD, random.Random(9), 6.0)
    c = spawn.choose_target_spawn(sw, [], WORLD, random.Random(10), 6.0)
    assert a == b and a[:2] != c[:2]


def test_infeasible_returns_best_effort_not_centre():
    # boids on a grid: no point is 6 m from all of them
    sw = [(x, y) for x in (3, 7, 11, 15, 18) for y in (3, 7, 11, 15, 18)]
    x, y, ok, clr = spawn.choose_target_spawn(
        sw, [], WORLD, random.Random(1), 6.0)
    assert not ok and clr < 6.0
    # best effort is clearly better than the arena centre fallback
    assert clr >= spawn.clearance(10.0, 10.0, sw) - 1e-9


def test_degenerate_inputs():
    # no pursuers: any in-bounds point, clearance inf
    x, y, ok, clr = spawn.choose_target_spawn(
        [], [], WORLD, random.Random(1), 6.0)
    assert ok and clr == float('inf') and 1.5 <= x <= WORLD - 1.5
    # everything blocked by one huge obstacle: falls back to the centre
    x, y, ok, _ = spawn.choose_target_spawn(
        [(1.0, 1.0)], [(10.0, 10.0, 50.0)], WORLD, random.Random(1), 6.0,
        tries=50)
    assert (x, y) == (10.0, 10.0) and not ok
    # honours shrinking bounds
    x, y, ok, _ = spawn.choose_target_spawn(
        [(5.0, 5.0)], [], WORLD, random.Random(2), 3.0,
        bounds=(8.0, 8.0, 14.0, 14.0))
    assert 9.5 <= x <= 12.5 and 9.5 <= y <= 12.5
    assert math.isfinite(x)
