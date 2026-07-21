"""Unit tests for the sim-side sensor model (SDD v4 Part A)."""

import math
import random

import pytest

from boids_swarm import perception as perc


def cfg(**over):
    base = dict(fov=math.pi, sensor_range=8.0, occlusion=False,
                range_sigma=0.0, bearing_sigma=0.0, p_miss=0.0,
                emit_ids=True)
    base.update(over)
    return base


def noiseless_rng():
    r = random.Random(0)
    r.gauss = lambda mu, sigma: mu        # deterministic: no noise
    r.random = lambda: 1.0                # never below p_miss
    return r


# --- range & FOV gating -----------------------------------------------------

def test_detects_neighbor_in_range():
    obs = (0.0, 0.0, 0.0)
    cands = [(1, 3.0, 0.0, 0.0, 1.0, False)]
    d = perc.compute_detections(obs, cands, [], cfg(), noiseless_rng())
    assert len(d) == 1
    rng_, bear, rel_head, speed, is_tgt, cid = d[0]
    assert rng_ == pytest.approx(3.0)
    assert bear == pytest.approx(0.0)
    assert is_tgt == 0.0 and cid == 1.0

def test_beyond_range_invisible():
    obs = (0.0, 0.0, 0.0)
    cands = [(1, 9.0, 0.0, 0.0, 1.0, False)]
    assert perc.compute_detections(obs, cands, [], cfg(), noiseless_rng()) == []

def test_fov_cone_excludes_behind():
    obs = (0.0, 0.0, 0.0)                  # looking +x, 60° half-cone
    behind = [(1, -3.0, 0.0, 0.0, 1.0, False)]
    side = [(2, 0.0, 3.0, 0.0, 1.0, False)]     # 90° to the left
    c = cfg(fov=math.radians(60))
    assert perc.compute_detections(obs, behind, [], c, noiseless_rng()) == []
    assert perc.compute_detections(obs, side, [], c, noiseless_rng()) == []
    front = [(3, 3.0, 0.5, 0.0, 1.0, False)]
    assert len(perc.compute_detections(obs, front, [], c, noiseless_rng())) == 1

def test_bearing_is_relative_to_heading():
    obs = (0.0, 0.0, math.pi / 2)          # facing +y
    cands = [(1, 0.0, 3.0, 0.0, 1.0, False)]   # directly ahead
    d = perc.compute_detections(obs, cands, [], cfg(), noiseless_rng())
    assert d[0][1] == pytest.approx(0.0)   # bearing 0 in own frame


# --- occlusion --------------------------------------------------------------

def test_occluded_by_obstacle():
    obs = (0.0, 0.0, 0.0)
    cands = [(1, 6.0, 0.0, 0.0, 1.0, False)]
    wall = [(3.0, 0.0, 1.0)]               # obstacle mid-ray
    c = cfg(occlusion=True)
    assert perc.compute_detections(obs, cands, wall, c, noiseless_rng()) == []

def test_not_occluded_when_obstacle_off_ray():
    obs = (0.0, 0.0, 0.0)
    cands = [(1, 6.0, 0.0, 0.0, 1.0, False)]
    wall = [(3.0, 3.0, 1.0)]               # well off the ray
    c = cfg(occlusion=True)
    assert len(perc.compute_detections(obs, cands, wall, c,
                                       noiseless_rng())) == 1

def test_occluder_behind_candidate_does_not_block():
    obs = (0.0, 0.0, 0.0)
    cands = [(1, 3.0, 0.0, 0.0, 1.0, False)]
    wall = [(5.0, 0.0, 1.0)]               # past the candidate
    c = cfg(occlusion=True)
    assert len(perc.compute_detections(obs, cands, wall, c,
                                       noiseless_rng())) == 1


# --- dropout ----------------------------------------------------------------

def test_p_miss_drops_detection():
    obs = (0.0, 0.0, 0.0)
    cands = [(1, 4.0, 0.0, 0.0, 1.0, False)]
    r = random.Random(0)
    r.random = lambda: 0.0                 # always below p_miss
    r.gauss = lambda mu, s: mu
    assert perc.compute_detections(obs, cands, [], cfg(p_miss=1.0), r) == []


# --- target flag & ids ------------------------------------------------------

def test_target_flagged():
    obs = (0.0, 0.0, 0.0)
    cands = [(9, 2.0, 0.0, 1.2, 3.5, True)]
    d = perc.compute_detections(obs, cands, [], cfg(), noiseless_rng())
    assert d[0][4] == 1.0                   # is_target
    assert d[0][3] == pytest.approx(3.5)    # speed passthrough

def test_ids_suppressed_when_disabled():
    obs = (0.0, 0.0, 0.0)
    cands = [(5, 2.0, 0.0, 0.0, 1.0, False)]
    d = perc.compute_detections(obs, cands, [], cfg(emit_ids=False),
                                noiseless_rng())
    assert d[0][5] == -1.0


# --- round trip: pack + world reconstruction --------------------------------

def test_flat_roundtrip_reconstructs_world_position():
    obs = (5.0, 5.0, math.pi / 2)          # facing +y
    cands = [(2, 5.0, 9.0, math.pi / 2, 0.0, False),   # 4 ahead
             (9, 8.0, 5.0, 0.0, 2.0, True)]            # 3 to the right
    flat = perc.detections_to_flat(
        perc.compute_detections(obs, cands, [], cfg(), noiseless_rng()))
    neighbors, target = perc.flat_to_world(flat, obs)
    assert len(neighbors) == 1
    nx, ny, nth, nid = neighbors[0]
    assert (nx, ny) == pytest.approx((5.0, 9.0), abs=1e-6)
    assert target is not None
    tx, ty, tth, tspeed, tid = target
    assert (tx, ty) == pytest.approx((8.0, 5.0), abs=1e-6)
    assert tth == pytest.approx(0.0, abs=1e-6)     # world heading recovered
    assert tspeed == pytest.approx(2.0)

def test_noise_grows_with_distance():
    """Range noise sigma is proportional to true range."""
    obs = (0.0, 0.0, 0.0)
    near = [(1, 1.0, 0.0, 0.0, 1.0, False)]
    far = [(1, 7.0, 0.0, 0.0, 1.0, False)]
    c = cfg(range_sigma=0.1)
    samples_near, samples_far = [], []
    for s in range(400):
        rn = random.Random(s)
        samples_near.append(
            perc.compute_detections(obs, near, [], c, rn)[0][0] - 1.0)
        rf = random.Random(s)
        samples_far.append(
            perc.compute_detections(obs, far, [], c, rf)[0][0] - 7.0)
    var_near = sum(x * x for x in samples_near) / len(samples_near)
    var_far = sum(x * x for x in samples_far) / len(samples_far)
    assert var_far > 4.0 * var_near         # far markedly noisier
