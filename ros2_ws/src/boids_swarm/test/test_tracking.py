"""Unit tests for the track filter + association (SDD v4 A.5)."""

import math

import pytest

from boids_swarm.tracking import Track, Tracker
from boids_swarm.behaviors import flocking


# --- alpha-beta filter ------------------------------------------------------

def test_track_estimates_velocity_from_motion():
    tr = Track(0.0, 0.0, 0.0, tid=1)
    # feed a constant-velocity path (2 m/s +x) at 10 Hz
    for k in range(1, 30):
        tr.update(0.2 * k, 0.0, 0.1 * k, alpha=0.6, beta=0.2)
    assert tr.vx == pytest.approx(2.0, abs=0.15)
    assert abs(tr.vy) < 0.15
    assert tr.speed == pytest.approx(2.0, abs=0.15)
    assert abs(flocking.wrap_angle if False else tr.heading) < 0.1

def test_track_predict_extrapolates():
    tr = Track(0.0, 0.0, 0.0, tid=1)
    for k in range(1, 20):
        tr.update(0.2 * k, 0.1 * k, 0.1 * k, alpha=0.6, beta=0.2)
    px, py = tr.predict(tr.t + 0.5)
    assert px > tr.x and py > tr.y            # keeps moving +x,+y

def test_track_smooths_noise():
    import random
    rng = random.Random(1)
    tr = Track(0.0, 0.0, 0.0, tid=1)
    err = []
    for k in range(1, 60):
        true_x = 0.2 * k
        tr.update(true_x + rng.gauss(0, 0.3), rng.gauss(0, 0.3),
                  0.1 * k, alpha=0.4, beta=0.1)
        if k > 20:
            err.append(abs(tr.x - true_x))
    assert sum(err) / len(err) < 0.25         # smoother than raw sigma 0.3


# --- data association --------------------------------------------------------

def test_id_association_is_stable():
    tk = Tracker()
    tk.step([(0.0, 0.0, 5, False)], 0.0)
    tk.step([(0.2, 0.0, 5, False)], 0.1)
    neighbors, target = tk.step([(0.4, 0.0, 5, False)], 0.2)
    assert len(neighbors) == 1 and target is None
    assert neighbors[0].hits == 3

def test_target_flag_routes_to_target():
    tk = Tracker()
    tk.step([(1.0, 1.0, 9, True), (0.0, 0.0, 2, False)], 0.0)
    neighbors, target = tk.step(
        [(1.1, 1.0, 9, True), (0.1, 0.0, 2, False)], 0.1)
    assert target is not None and target.tid == 9
    assert len(neighbors) == 1 and neighbors[0].tid == 2

def test_stale_track_pruned():
    tk = Tracker(max_age=0.3)
    tk.step([(0.0, 0.0, 1, False)], 0.0)
    neighbors, _ = tk.step([(5.0, 5.0, 2, False)], 0.5)   # 1 unseen 0.5s
    ids = {n.tid for n in neighbors}
    assert ids == {2}                         # track 1 aged out

def test_nn_association_without_ids():
    tk = Tracker(gate=1.0)
    # anonymous blips (id = -1) associate by proximity across frames
    tk.step([(0.0, 0.0, -1, False)], 0.0)
    tk.step([(0.2, 0.0, -1, False)], 0.1)
    neighbors, _ = tk.step([(0.4, 0.0, -1, False)], 0.2)
    assert len(neighbors) == 1               # one persistent track, not three
    assert neighbors[0].hits == 3

def test_nn_spawns_new_beyond_gate():
    tk = Tracker(gate=1.0)
    tk.step([(0.0, 0.0, -1, False)], 0.0)
    neighbors, _ = tk.step([(5.0, 0.0, -1, False)], 0.1)  # far ⇒ new track
    # old one still within max_age, plus the new one
    assert len(neighbors) == 2


# --- search behavior ---------------------------------------------------------

def test_search_points_toward_patrol_ring():
    # agent at centre, index 0 at t=0 ⇒ patrol point at +x on the ring
    v = flocking.search_vec((10.0, 10.0), 0, 12, 20.0, 0.0)
    assert v[0] > 0.9 and abs(v[1]) < 0.1

def test_search_sectors_are_distinct():
    dirs = set()
    for i in range(12):
        vx, vy = flocking.search_vec((10.0, 10.0), i, 12, 20.0, 0.0)
        dirs.add(round(math.atan2(vy, vx), 2))
    assert len(dirs) >= 10                    # agents fan out to cover
