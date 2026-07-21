"""Unit tests for the range-limited sighting mesh (SDD v4 A.6 / M10)."""

import math
import random

import pytest

from boids_swarm import comms
from boids_swarm import perception as perc


# --- comm graph / propagation -----------------------------------------------

def test_single_seer_informs_connected_chain():
    # a chain of 4 agents, each 3 apart, comm_range 4 ⇒ all linked
    pos = [(0, 0), (3, 0), (6, 0), (9, 0)]
    seers = [False, False, False, True]        # only the last sees it
    informed = comms.propagate_sightings(pos, 4.0, seers)
    assert informed == [True, True, True, True]

def test_gap_breaks_the_mesh():
    # agent 2 is out of range of the rest ⇒ its component is separate
    pos = [(0, 0), (3, 0), (20, 20), (6, 0)]
    seers = [True, False, False, False]        # agent 0 sees it
    informed = comms.propagate_sightings(pos, 4.0, seers)
    assert informed[0] and informed[1] and informed[3]
    assert not informed[2]                     # isolated, stays blind

def test_no_seer_informs_nobody():
    pos = [(0, 0), (1, 0), (2, 0)]
    assert comms.propagate_sightings(pos, 5.0, [False] * 3) == [False] * 3

def test_seer_in_isolated_component_only_informs_itself():
    pos = [(0, 0), (0, 1), (30, 30)]
    seers = [False, False, True]
    informed = comms.propagate_sightings(pos, 3.0, seers)
    assert informed == [False, False, True]

def test_comm_range_zero_isolates_all():
    pos = [(0, 0), (0.5, 0), (1, 0)]
    seers = [True, False, False]
    informed = comms.propagate_sightings(pos, 0.0, seers)
    assert informed == [True, False, False]


# --- relayed detection geometry ---------------------------------------------

def noiseless_rng():
    r = random.Random(0)
    r.gauss = lambda mu, s: mu
    return r

def test_relayed_detection_in_receiver_frame():
    observer = (5.0, 5.0, 0.0)                 # facing +x
    target = (5.0, 9.0, math.pi / 2, 2.0)      # 4 units north, heading +y
    d = perc.relayed_detection(observer, target, 12, 0.0, noiseless_rng())
    r, bearing, rel_head, speed, is_tgt, cid = d
    assert r == pytest.approx(4.0)
    assert bearing == pytest.approx(math.pi / 2)   # due left in own frame
    assert rel_head == pytest.approx(math.pi / 2)
    assert is_tgt == 1.0 and cid == 12.0 and speed == pytest.approx(2.0)

def test_relayed_detection_reconstructs_target_world_pos():
    observer = (2.0, 3.0, 1.1)
    target = (8.0, 7.0, -0.4, 1.5)
    d = perc.relayed_detection(observer, target, 12, 0.0, noiseless_rng())
    flat = perc.detections_to_flat([d])
    _, tgt = perc.flat_to_world(flat, observer)
    assert tgt is not None
    assert (tgt[0], tgt[1]) == pytest.approx((8.0, 7.0), abs=1e-6)
