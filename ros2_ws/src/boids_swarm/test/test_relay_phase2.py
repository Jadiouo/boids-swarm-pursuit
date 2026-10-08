"""Phase 2 (SDD_relay_phase2_prereg): counters, stats helpers, matrix runner."""

import json

import pytest

from boids_swarm.sighting_runtime import SightingCounters
from boids_swarm import relay_stats as rs
from boids_swarm import experiment_matrix as em


# --- R-03 receiver counters ---------------------------------------------------

def test_counters_start_at_zero():
    c = SightingCounters()
    snap = c.snapshot()
    assert snap == {'sightings_published': 0, 'sightings_received_total': 0,
                    'sightings_received_in_range': 0,
                    'sightings_received_out_of_range': 0,
                    'sightings_received_self': 0,
                    'sightings_received_unclassified': 0}

def test_counters_classify_self_in_out_and_unknown():
    c = SightingCounters()
    c.on_published()
    c.on_received(is_self=True, distance=None, radio_range=8.0)
    c.on_received(is_self=False, distance=3.0, radio_range=8.0)
    c.on_received(is_self=False, distance=8.0, radio_range=8.0)   # inclusive
    c.on_received(is_self=False, distance=8.01, radio_range=8.0)
    c.on_received(is_self=False, distance=None, radio_range=8.0)
    s = c.snapshot()
    assert s['sightings_published'] == 1
    assert s['sightings_received_total'] == 5
    assert s['sightings_received_self'] == 1
    assert s['sightings_received_in_range'] == 2
    assert s['sightings_received_out_of_range'] == 1
    assert s['sightings_received_unclassified'] == 1

def test_self_is_never_counted_as_in_range_even_with_distance():
    c = SightingCounters()
    c.on_received(is_self=True, distance=0.0, radio_range=8.0)
    s = c.snapshot()
    assert s['sightings_received_in_range'] == 0
    assert s['sightings_received_self'] == 1

def test_counter_total_is_sum_of_parts():
    c = SightingCounters()
    for d in (None, 1.0, 9.0, 2.0):
        c.on_received(is_self=False, distance=d, radio_range=8.0)
    c.on_received(is_self=True, distance=None, radio_range=8.0)
    s = c.snapshot()
    assert s['sightings_received_total'] == (
        s['sightings_received_in_range'] + s['sightings_received_out_of_range']
        + s['sightings_received_self'] + s['sightings_received_unclassified'])


# --- R-01 shared sighting QoS -------------------------------------------------

def test_shared_sighting_qos_depth_is_configurable_best_effort_volatile():
    pytest.importorskip('rclpy')  # ROS-only check; skipped on bare CI
    from boids_swarm.sighting_runtime import shared_sighting_qos_spec
    assert shared_sighting_qos_spec(1)['depth'] == 1         # depth=1 stays usable (phase-1 behaviour)
    spec = shared_sighting_qos_spec(10)
    assert spec == {'depth': 10, 'reliability': 'best_effort',
                    'durability': 'volatile'}

def test_shipped_default_shared_sighting_qos_depth_is_10():
    import re
    from pathlib import Path
    pkg = Path(__file__).resolve().parents[1]
    params = (pkg / 'config' / 'params.yaml').read_text()
    m = re.search(r'^\s*shared_sighting_qos_depth:\s*(\d+)', params, re.M)
    assert m and int(m.group(1)) == 10
    node = (pkg / 'boids_swarm' / 'boid_controller_node.py').read_text()
    assert "declare_parameter('shared_sighting_qos_depth', 10)" in node


def test_shared_sighting_qos_depth_rejects_nonpositive():
    from boids_swarm.sighting_runtime import shared_sighting_qos_spec
    with pytest.raises(ValueError):
        shared_sighting_qos_spec(0)


# --- R-05 stats helpers -------------------------------------------------------

def test_wilson_known_values():
    lo, hi = rs.wilson(0, 10)
    assert lo == 0.0 and hi == pytest.approx(0.2775, abs=1e-3)
    lo, hi = rs.wilson(10, 20)
    assert (lo, hi) == pytest.approx((0.299, 0.701), abs=2e-3)
    assert rs.wilson(0, 0) is None

def test_wilson_contains_point_estimate():
    for k, n in ((1, 20), (7, 20), (20, 20), (3, 10)):
        lo, hi = rs.wilson(k, n)
        assert 0.0 <= lo <= k/n <= hi <= 1.0

def test_median():
    assert rs.median([]) is None
    assert rs.median([3.0]) == 3.0
    assert rs.median([5, 1, 3]) == 3
    assert rs.median([1, 2, 3, 4]) == 2.5

def test_expected_count_respects_per_receiver_cutoff():
    events = [{'t': 1.0, 'exp': ['agent1', 'agent2']},
              {'t': 2.0, 'exp': ['agent1']},
              {'t': 3.0, 'exp': ['agent1', 'agent2']}]
    # agent2's last status at t=2.5, margin 0.05 -> only events t<=2.45 count
    exp = rs.expected_in_range_counts(events, {'agent1': 10.0, 'agent2': 2.5},
                                      margin=0.05)
    assert exp == {'agent1': 3, 'agent2': 1}

def test_per_receiver_drop_rate():
    drops = rs.per_receiver_drop({'agent1': 10, 'agent2': 4},
                                 {'agent1': 7, 'agent2': 4, 'agent3': 5})
    assert drops['agent1'] == pytest.approx(0.3)
    assert drops['agent2'] == 0.0
    assert 'agent3' not in drops          # nothing expected -> undefined

def test_pooled_drop_is_ratio_of_sums_not_mean_of_ratios():
    pooled = rs.pooled_drop({'a': 100, 'b': 1}, {'a': 90, 'b': 0})
    assert pooled == pytest.approx(1 - 90/101)

def test_pooled_drop_none_when_nothing_expected():
    assert rs.pooled_drop({}, {}) is None


# --- R-04 matrix runner -------------------------------------------------------

CFG = {
    'scenes': {'S12': {'num_agents': 12, 'seeds': [1, 2]},
               'S4': {'num_agents': 4, 'seeds': [1]}},
    'modes': {'off': {'sharing_mode': 'off'},
              'oracle-1h': {'sharing_mode': 'oracle', 'oracle_max_hops': 1},
              'ros-d10': {'sharing_mode': 'ros', 'shared_sighting_qos_depth': 10}},
    'shuffle_seed': 7,
}

def test_matrix_expansion_count_and_ids_unique():
    runs = em.expand_matrix(CFG)
    assert len(runs) == (2 + 1) * 3
    assert len({r.run_id for r in runs}) == len(runs)

def test_matrix_is_deterministic_and_covers_all_cells():
    a = [r.run_id for r in em.expand_matrix(CFG)]
    b = [r.run_id for r in em.expand_matrix(CFG)]
    assert a == b
    assert set(a) == {f'{s}-{m}-seed{k}' for s, ks in (('S12', (1, 2)), ('S4', (1,)))
                      for m in CFG['modes'] for k in ks}

def test_matrix_run_carries_launch_overrides():
    r = next(x for x in em.expand_matrix(CFG) if x.run_id == 'S12-ros-d10-seed2')
    assert r.num_agents == 12 and r.seed == 2
    assert r.launch_args['sharing_mode'] == 'ros'
    assert r.launch_args['shared_sighting_qos_depth'] == 10

def test_is_done_requires_completed_run_config(tmp_path):
    d = tmp_path / 'S12-off-seed1'
    assert not em.is_done(d)
    d.mkdir()
    assert not em.is_done(d)                       # partial dir, no config
    (d / 'run_config.json').write_text(json.dumps({'validity': 'valid'}))
    assert em.is_done(d)

def test_invalid_runs_count_as_done_not_silently_retried(tmp_path):
    d = tmp_path / 'x'
    d.mkdir()
    (d / 'run_config.json').write_text(
        json.dumps({'validity': 'invalid', 'invalid_reason': 'crash'}))
    assert em.is_done(d)

def test_classify_validity():
    ok = em.classify_validity(exit_code=0, result_found=True, wall_timeout=False)
    assert ok == ('valid', None)
    assert em.classify_validity(0, False, False)[0] == 'invalid'
    assert em.classify_validity(1, True, False)[0] == 'invalid'
    assert em.classify_validity(0, True, True) == ('invalid', 'wall_timeout')

def test_cell_invalid_fraction_flags_unreliable_over_ten_percent():
    assert em.cell_reliability(valid=18, invalid=2) == 'ok'           # 10% not >10%
    assert em.cell_reliability(valid=17, invalid=3) == 'unreliable'


def test_scene_level_launch_args_are_merged_into_runs():
    cfg = {'scenes': {'A': {'num_agents': 2, 'seeds': [1],
                            'launch_args': {'time_scale': 1}}},
           'modes': {'m': {'sharing_mode': 'ros'}}}
    (r,) = em.expand_matrix(cfg)
    assert r.launch_args == {'time_scale': 1, 'sharing_mode': 'ros'}


# --- E1 analysis fixes (exploratory) -----------------------------------------

def test_km_no_censoring_matches_empirical_survival():
    c = rs.km_curve([1, 2, 3, 4], [True] * 4)
    assert [round(r[1], 3) for r in c] == [1.0, 0.75, 0.5, 0.25, 0.0]

def test_km_right_censoring_keeps_censored_in_risk_set():
    # 4 runs: captures at 1 and 3, censored at 2 and 30
    c = rs.km_curve([1, 2, 3, 30], [True, False, True, False])
    s = {r[0]: r[1] for r in c}
    assert s[1] == pytest.approx(0.75)
    assert s[3] == pytest.approx(0.75 * (1 - 1 / 2))  # 2 at risk at t=3

def test_km_greenwood_band_contains_point_and_is_clipped():
    for t, s, lo, hi in rs.km_curve([1, 2, 3, 30, 30], [True, True, True, False, False]):
        assert 0.0 <= lo <= s <= hi <= 1.0
    assert km_value(rs.km_curve([1, 2], [True, True]), 5) == 0.0

def km_value(curve, t):
    return rs.km_at(curve, t)[0]

def test_bootstrap_ci_brackets_point_and_is_deterministic():
    pairs = [(100, 50), (100, 45), (100, 55), (100, 52)]
    a = rs.bootstrap_pooled_drop_ci(pairs, n_boot=2000, seed=3)
    assert a == rs.bootstrap_pooled_drop_ci(pairs, n_boot=2000, seed=3)
    pt, lo, hi = a
    assert pt == pytest.approx(1 - 202 / 400)
    assert lo <= pt <= hi and hi - lo < 0.1

def test_bootstrap_ci_none_without_traffic():
    assert rs.bootstrap_pooled_drop_ci([(0, 0)]) is None

def test_power_calc_known_value():
    # textbook: 0.25 vs 0.45, alpha .05, power .8 -> ~88 per group (no cc)
    n = rs.n_per_group_two_proportions(0.25, 0.45)
    assert 85 <= n <= 90
    assert rs.n_per_group_two_proportions(0.25, 0.45, continuity=True) > n

def test_window_fraction_uses_fixed_window():
    rows = [{'event': 'cycle', 'own_pose_fresh': True, 'stamp': t,
             'belief_valid': t < 2.0} for t in (0.0, 1.0, 2.0, 3.0, 9.0)]
    assert rs.window_fraction(rows, 3.0) == (0.5, 4)   # stamps 0,1,2,3
    assert rs.window_fraction(rows, 1.0) == (1.0, 2)
    assert rs.window_fraction([], 5.0) == (None, 0)
