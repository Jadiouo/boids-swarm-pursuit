"""Unit tests for the pure Boids math (SDD v2 §8)."""

import math

import pytest

from boids_turtlesim import behaviors as bh


def angle_of(v):
    return math.atan2(v[1], v[0])


# --- Alignment: the angle-wrap trap (§4.3.2 worked check) ------------------

def test_alignment_wraps_correctly():
    """Mean of 179° and -179° must be ≈180°, not 0°."""
    n = [(0, 0, math.radians(179)), (0, 0, math.radians(-179))]
    v = bh.alignment(n)
    assert abs(bh.wrap_angle(angle_of(v) - math.pi)) < 1e-6


def test_alignment_unit_magnitude():
    n = [(0, 0, 0.5), (0, 0, 0.7)]
    v = bh.alignment(n)
    assert math.hypot(*v) == pytest.approx(1.0)


def test_alignment_opposite_headings_cancel():
    n = [(0, 0, 0.0), (0, 0, math.pi)]
    assert bh.alignment(n) == (0.0, 0.0)


# --- Separation (§4.3.1) ----------------------------------------------------

def test_separation_points_away_and_scales():
    me = (5.0, 5.0)
    near = [(5.5, 5.0, 0.0)]       # 0.5 to the +x side
    far = [(5.9, 5.0, 0.0)]        # 0.9 to the +x side
    v_near = bh.separation(me, near, d_safe=1.0)
    v_far = bh.separation(me, far, d_safe=1.0)
    assert v_near[0] < 0 and v_far[0] < 0          # pushed in -x
    assert abs(v_near[0]) > abs(v_far[0])          # closer -> stronger

def test_separation_ignores_beyond_safe_distance():
    assert bh.separation((0, 0), [(2.0, 0.0, 0.0)], d_safe=1.0) == (0.0, 0.0)


# --- Cohesion (§4.3.3) ------------------------------------------------------

def test_cohesion_points_to_center_of_mass():
    v = bh.cohesion((0.0, 0.0), [(2.0, 0.0, 0.0), (2.0, 2.0, 0.0)])
    assert angle_of(v) == pytest.approx(math.atan2(1.0, 2.0))
    assert math.hypot(*v) == pytest.approx(1.0)


# --- Boundary (§4.3.4) ------------------------------------------------------

BMIN, BMAX, M = [0.5, 0.5], [10.5, 10.5], 1.5

def test_boundary_zero_in_interior():
    assert bh.boundary((5.5, 5.5), BMIN, BMAX, M) == (0.0, 0.0)

def test_boundary_pushes_inward_proportionally():
    vx, vy = bh.boundary((1.0, 10.4), BMIN, BMAX, M)
    assert vx == pytest.approx((0.5 + 1.5 - 1.0) / 1.5)     # push +x
    assert vy == pytest.approx(-((10.4 - (10.5 - 1.5)) / 1.5))  # push -y

def test_boundary_corner_pushes_diagonally():
    vx, vy = bh.boundary((0.6, 0.6), BMIN, BMAX, M)
    assert vx > 0 and vy > 0


# --- Kinematic conversion (§4.5): no reverse, deadlock fixed ---------------

def test_to_twist_never_reverses():
    """Target directly behind: linear must be >= 0, angular saturated."""
    v_lin, w_z = bh.to_twist((-1.0, 0.0), theta=0.0, kv=1.0, kw=4.0,
                             v_min=0.0, v_max=2.0, w_max=5.0)
    assert v_lin >= 0.0
    assert abs(w_z) == pytest.approx(5.0)          # clamped at w_max

def test_to_twist_vmin_floor_escapes_perpendicular_stall():
    """90° error: cos term is 0, but v_min keeps the agent creeping."""
    v_lin, _ = bh.to_twist((0.0, 1.0), theta=0.0, kv=1.0, kw=4.0,
                           v_min=0.2, v_max=2.0, w_max=5.0)
    assert v_lin == pytest.approx(0.2)

def test_to_twist_straight_ahead_full_speed():
    v_lin, w_z = bh.to_twist((2.0, 0.0), theta=0.0, kv=1.0, kw=4.0,
                             v_min=0.0, v_max=2.0, w_max=5.0)
    assert v_lin == pytest.approx(2.0)
    assert w_z == pytest.approx(0.0)

def test_to_twist_zero_vector_stops():
    assert bh.to_twist((0.0, 0.0), 0.0, 1.0, 4.0, 0.0, 2.0, 5.0) == (0.0, 0.0)


# --- wrap_angle -------------------------------------------------------------

@pytest.mark.parametrize('a', [0.0, 1.0, -1.0, 3.5, -3.5, 10.0])
def test_wrap_angle_range(a):
    w = bh.wrap_angle(a)
    assert -math.pi <= w <= math.pi
    assert math.cos(w) == pytest.approx(math.cos(a))
    assert math.sin(w) == pytest.approx(math.sin(a))
