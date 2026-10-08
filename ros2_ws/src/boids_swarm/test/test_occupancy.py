"""Unit tests for the OccupancyGrid rasterizer used by nav2_bridge (M7 spike)."""

import math

import pytest

from boids_swarm.occupancy import cell_of, rasterize_circles


def _at(g, ix, iy):
    return g.data[iy * g.width + ix]


def test_grid_dimensions_and_origin():
    g = rasterize_circles(20.0, 10.0, 0.1, [], border_cells=0)
    assert (g.width, g.height) == (200, 100)
    assert g.resolution == 0.1
    assert g.origin == (0.0, 0.0)
    assert len(g.data) == 200 * 100
    assert set(g.data) == {0}


def test_cell_of_respects_origin_and_resolution():
    assert cell_of(0.05, 0.05, 0.1) == (0, 0)
    assert cell_of(1.0, 2.0, 0.1) == (10, 20)
    assert cell_of(-0.5, 0.0, 0.5, origin=(-1.0, -1.0)) == (1, 2)


def test_circle_marks_center_and_not_far_cells():
    g = rasterize_circles(20.0, 20.0, 0.1, [(10.0, 10.0, 1.0)], border_cells=0)
    assert _at(g, *cell_of(10.0, 10.0, 0.1)) == 100
    assert _at(g, *cell_of(10.9, 10.0, 0.1)) == 100
    assert _at(g, *cell_of(11.3, 10.0, 0.1)) == 0
    assert _at(g, *cell_of(5.0, 5.0, 0.1)) == 0


def test_circle_area_close_to_pi_r2():
    g = rasterize_circles(20.0, 20.0, 0.1, [(10.0, 10.0, 2.0)], border_cells=0)
    area = sum(1 for v in g.data if v) * 0.1 * 0.1
    assert math.pi * 4.0 <= area <= math.pi * 4.0 * 1.1   # conservative, never under


def test_tiny_circle_still_marks_its_cell():
    g = rasterize_circles(5.0, 5.0, 0.5, [(2.3, 2.3, 0.01)], border_cells=0)
    assert _at(g, *cell_of(2.3, 2.3, 0.5)) == 100


def test_circle_clipped_at_boundary_no_crash():
    g = rasterize_circles(10.0, 10.0, 0.1, [(0.0, 0.0, 2.0), (10.5, 10.5, 1.0)],
                          border_cells=0)
    assert _at(g, 0, 0) == 100
    assert _at(g, 99, 99) == 100
    assert len(g.data) == 100 * 100


def test_border_ring():
    g = rasterize_circles(10.0, 10.0, 0.1, [], border_cells=2)
    for k in range(100):
        assert _at(g, k, 0) == 100 and _at(g, k, 1) == 100
        assert _at(g, k, 98) == 100 and _at(g, k, 99) == 100
        assert _at(g, 0, k) == 100 and _at(g, 99, k) == 100
    assert _at(g, 2, 2) == 0
    assert _at(g, 50, 50) == 0


def test_non_origin_grid():
    g = rasterize_circles(4.0, 4.0, 0.5, [(0.0, 0.0, 0.4)], border_cells=0,
                          origin=(-2.0, -2.0))
    assert _at(g, *cell_of(0.0, 0.0, 0.5, origin=(-2.0, -2.0))) == 100
    assert _at(g, 0, 0) == 0


# ----------------------------------------------- arena wall / footprint (A)

from boids_swarm.occupancy import (INSCRIBED, grid_array,   # noqa: E402
                                   inflation_cost, inscribed_radius,
                                   rasterize_arena)

BODY_R = 0.15      # params.yaml target_body_radius


def test_arena_border_is_outside_the_real_wall():
    g = rasterize_arena(20.0, 20.0, 0.1, [], border_cells=1)
    assert (g.width, g.height) == (202, 202)
    assert g.origin == pytest.approx((-0.1, -0.1))
    occ = grid_array(g)
    # every cell that lies inside the arena [0,20]^2 is free ...
    assert not occ[1:-1, 1:-1].any()
    # ... and the ring outside it is the (1 cell = 0.1 m) wall
    assert occ[0, :].all() and occ[-1, :].all()
    assert occ[:, 0].all() and occ[:, -1].all()
    ix, iy = cell_of(0.05, 10.0, 0.1, g.origin)
    assert not occ[iy, ix]               # the floor at the wall is not "wall"


def test_body_touching_wall_is_high_cost_not_inscribed():
    """The sim lets the target centre stand 0.15 m from the wall; that cell
    must be traversable (cost < INSCRIBED) but expensive, for every wall."""
    g = rasterize_arena(20.0, 20.0, 0.1, [], border_cells=1)
    c = inflation_cost(grid_array(g), g.resolution, BODY_R)
    for (x, y) in ((BODY_R, 10.0), (19.85, 10.0), (10.0, BODY_R),
                   (10.0, 19.85), (BODY_R, BODY_R)):
        ix, iy = cell_of(x, y, 0.1, g.origin)
        assert 100 < c[iy, ix] < INSCRIBED, (x, y, c[iy, ix])
    # cells whose centre the body cannot occupy ARE blocked
    ix, iy = cell_of(0.05, 10.0, 0.1, g.origin)
    assert c[iy, ix] >= INSCRIBED


def test_inscribed_radius_follows_the_16_gon_footprint():
    assert inscribed_radius(0.15) == pytest.approx(0.15 * math.cos(math.pi / 16))
    assert inscribed_radius(0.15) < 0.15


def test_old_inside_border_made_the_wall_standing_cell_inscribed():
    """Regression for the red-team finding: border drawn inside the arena
    (cells [0,0.1)) + radius 0.15 puts the legal body position at 0.15 m
    into the inscribed zone, so navfn refused to plan from there."""
    g = rasterize_circles(20.0, 20.0, 0.1, [], border_cells=1)
    c = inflation_cost(grid_array(g), g.resolution, BODY_R)
    ix, iy = cell_of(BODY_R, 10.0, 0.1)
    assert c[iy, ix] >= INSCRIBED
