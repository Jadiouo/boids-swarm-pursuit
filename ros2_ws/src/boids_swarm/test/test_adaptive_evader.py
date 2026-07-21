"""Unit tests for the adaptive utility-based evader (SDD v4 Part D, M14)."""

import math

import pytest

from boids_swarm.behaviors.evasion import AdaptiveEvader


def ev(obstacles=()):
    return AdaptiveEvader([0.0, 0.0], [20.0, 20.0], obstacles)


def test_returns_unit_direction():
    e = ev()
    d = e.compute((10, 10), 0.0, [(8, 10, 0)])
    assert math.hypot(*d) == pytest.approx(1.0, abs=1e-6)

def test_does_not_flee_toward_pursuer():
    e = ev()
    d = e.compute((10, 10), 0.0, [(8, 10, 0)])    # pursuer to the west
    # chosen direction must not point at the pursuer (west = -x); running
    # along the wall (perpendicular) or away (east) are both valid
    to_pursuer = (-1.0, 0.0)
    assert d[0] * to_pursuer[0] + d[1] * to_pursuer[1] < 0.2

def test_mode_is_logged_and_valid():
    e = ev()
    e.compute((10, 10), 0.0, [(8, 10, 0)])
    assert e.mode in ('retreat', 'perimeter', 'juke', 'gap_dash', 'shield')

def test_juke_available_when_pursuer_close():
    e = ev()
    # a pursuer inside PANIC distance should make juke a candidate and
    # often the choice; at minimum the evader still returns a valid move
    d = e.compute((10, 10), 0.0, [(10 - 1.5, 10, 0)])
    assert math.hypot(*d) == pytest.approx(1.0, abs=1e-6)

def test_encircled_prefers_gap_dash():
    e = ev()
    # ring of pursuers with a clear gap toward +x (east)
    pursuers = []
    for a in range(0, 360, 45):
        if 320 <= a or a <= 40:            # leave an eastward gap
            continue
        r = math.radians(a)
        pursuers.append((10 + 4 * math.cos(r), 10 + 4 * math.sin(r), 0))
    d = e.compute((10, 10), 0.0, pursuers)
    # the safest escape is roughly east through the gap
    assert d[0] > 0.3

def test_hysteresis_prevents_dither():
    e = ev()
    pursuers = [(8, 10, 0)]
    modes = [e.compute((10, 10), 0.0, pursuers) and e.mode
             for _ in range(6)]
    # once settled, the mode should be stable across identical inputs
    assert len(set(modes[2:])) == 1

def test_perimeter_dir_runs_along_wall():
    e = ev()
    d = e._perimeter_dir((0.5, 10.0), math.pi / 2)   # hugging left wall
    assert abs(d[0]) < 1e-9 and d[1] > 0             # tangent is vertical +y

def test_shield_uses_obstacle_between():
    e = ev([(12.0, 10.0, 1.0)])                       # obstacle to the east
    sd = e._shield_dir((10.0, 10.0), (8.0, 10.0, 0))
    assert sd[0] > 0.9                                # toward the obstacle
