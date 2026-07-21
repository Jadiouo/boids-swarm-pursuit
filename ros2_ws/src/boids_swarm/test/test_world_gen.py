"""Unit tests for procedural world generation (SDD v4 C.5, M13)."""

import math

import pytest

from boids_swarm.world_gen import WorldGenerator, Layout


def test_seed_determinism():
    a = WorldGenerator(42).generate('obstacle_field')
    b = WorldGenerator(42).generate('obstacle_field')
    assert a.obstacles == b.obstacles

def test_different_seeds_differ():
    # obstacle_field is now a fixed hand-authored map; zones stay seeded
    a = WorldGenerator(1).generate('zones')
    b = WorldGenerator(2).generate('zones')
    assert a.zones != b.zones

def test_open_is_empty():
    lay = WorldGenerator(7).generate('open')
    assert lay.obstacles == [] and lay.zones == [] and lay.shrink_rate == 0.0

def test_obstacle_field_is_fixed_regardless_of_seed():
    a = WorldGenerator(0, 20.0).generate('obstacle_field').obstacles
    b = WorldGenerator(999, 20.0).generate('obstacle_field').obstacles
    assert a == b                              # hand-authored, seed-independent

def test_obstacle_field_is_dense_varied_with_passages():
    obs = WorldGenerator(7, 20.0).generate('obstacle_field').obstacles
    assert len(obs) >= 10                      # a dense field, not a few plugs
    radii = [o[2] for o in obs]
    assert max(radii) > 1.7 and min(radii) < 1.2   # varied sizes (big + small)
    # every pair keeps a passage: centres are farther than r1+r2 (no overlap)
    for i, (x1, y1, r1) in enumerate(obs):
        for (x2, y2, r2) in obs[i + 1:]:
            assert math.hypot(x1 - x2, y1 - y2) > r1 + r2   # gap between them

def test_obstacles_do_not_overlap():
    lay = WorldGenerator(11).generate('obstacle_field')
    for i, (x1, y1, r1) in enumerate(lay.obstacles):
        for (x2, y2, r2) in lay.obstacles[i + 1:]:
            assert math.hypot(x1 - x2, y1 - y2) > r1 + r2 - 1e-6

def test_obstacles_inside_arena():
    lay = WorldGenerator(3, world_size=20.0).generate('obstacle_field')
    for (x, y, r) in lay.obstacles:
        assert 0 < x < 20 and 0 < y < 20

def test_pillar_is_central_and_single():
    lay = WorldGenerator(5, world_size=20.0).generate('pillar')
    assert len(lay.obstacles) == 1
    x, y, r = lay.obstacles[0]
    assert (x, y) == pytest.approx((10.0, 10.0))
    assert 2.0 <= r <= 3.0

def test_zones_capture_and_slow():
    lay = WorldGenerator(9).generate('zones')
    kinds = {z[3] for z in lay.zones}
    assert 'capture' in kinds and 'slow' in kinds

def test_shrink_layout_sets_rate():
    lay = WorldGenerator(1).generate('shrink')
    assert lay.shrink_rate > 0.0 and lay.obstacles == []

def test_flat_encodings_round_trip_shape():
    lay = WorldGenerator(2).generate('zones')
    zf = lay.zones_flat()
    assert len(zf) % 4 == 0                     # (x,y,r,kind) stride
    of = WorldGenerator(2).generate('obstacle_field').obstacles_flat()
    assert len(of) % 3 == 0

def test_unknown_env_type_raises():
    with pytest.raises(ValueError):
        WorldGenerator(1).generate('nope')
