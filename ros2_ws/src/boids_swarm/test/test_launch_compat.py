"""Backward-compatibility of the launch files after the control-panel work.

Experiment scripts (tools/run_relay_e1.py ...) and ros_test launch
pursuit.launch.py headless or with ui:=false; that path must start exactly
the nodes and parameters it did before. `golden/pursuit_launch.json` was
recorded from the pre-panel launch file (see gen_golden.py) and is compared
here with what the current file produces.

The panel path (ui:=true, headless:=false) launches only the sim and lets
the sim start swarm_stack.launch.py; the second test proves that, together,
they start the same controllers with the same parameters.

Needs `launch` / `launch_ros` (a sourced ROS install); skipped without.
"""

import json
import os
import sys

import pytest

pytest.importorskip('launch_ros')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from golden_cases import CASES  # noqa: E402
from launch_recorder import record  # noqa: E402
from boids_swarm import stack_config  # noqa: E402

GOLDEN = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      'golden', 'pursuit_launch.json')

with open(GOLDEN) as _f:
    _GOLD = json.load(_f)


def _names(acts):
    return [(a['kind'], a.get('name'), a.get('namespace')) for a in acts]


@pytest.mark.parametrize('case', sorted(CASES))
def test_headless_and_ui_off_launch_is_unchanged(case):
    assert record('pursuit.launch.py', CASES[case]) == _GOLD[case]


# keys that legitimately differ between the two launch paths
_SIM_ONLY = ('headless', 'ui_enabled', 'ui_scale', 'stack_managed',
             'stack_launch_args', *stack_config.UI_SIM_DEFAULTS)


def _strip_sim(act):
    params = [dict(p) if isinstance(p, dict) else p
              for p in act['parameters']]
    for p in params:
        if isinstance(p, dict):
            for k in _SIM_ONLY:
                p.pop(k, None)
    return dict(act, parameters=params)


@pytest.mark.parametrize('case', sorted(CASES))
def test_panel_path_starts_the_same_nodes_via_the_stack_launch(case):
    # The panel path defaults an unspecified evader to smart (see the test
    # below); pin it so this stays an equivalence check against the golden.
    ui_args = {'evader': 'reactive', **CASES[case],
               'ui': 'true', 'headless': 'false'}
    acts = record('pursuit.launch.py', ui_args)
    # the panel path launches the window-owning sim and nothing else
    assert len(acts) == 1 and acts[0]['name'] == 'pygame_sim'
    sim = acts[0]
    flat = {}
    for p in sim['parameters']:
        if isinstance(p, dict):
            flat.update(p)
    assert flat['stack_managed'] is True
    for k, v in stack_config.UI_SIM_DEFAULTS.items():     # window: fair start
        assert flat[k] == v, k
    stack_launch_args = json.loads(flat['stack_launch_args'])
    assert set(stack_launch_args) == set(stack_config.STACK_LAUNCH_KEYS)
    gold_sim, *gold_rest = _GOLD[case]
    assert _strip_sim(sim) == _strip_sim(gold_sim)
    # the sim's supervisor would run swarm_stack.launch.py with these args
    stack = record('swarm_stack.launch.py',
                   dict(stack_launch_args, fail_fast='false'))
    assert stack == gold_rest


def test_stack_launch_fail_fast_only_adds_shutdown_on_exit():
    args = {'num_agents': '3', 'evader': 'adaptive'}
    quiet = record('swarm_stack.launch.py', dict(args, fail_fast='false'))
    loud = record('swarm_stack.launch.py', dict(args, fail_fast='true'))
    assert _names(quiet) == _names(loud)
    for q, l_ in zip(quiet, loud):
        assert l_.pop('on_exit') == 'Shutdown'
        assert 'on_exit' not in q
        assert q == l_


def test_stack_launch_carries_live_params_into_controllers():
    acts = record('swarm_stack.launch.py', {
        'num_agents': '2', 'fail_fast': 'false',
        'agent_params': json.dumps({'w_cohesion': 1.25}),
        'target_params': json.dumps({'target_omega_max': 0.9})})
    tgt = next(a for a in acts if a['name'] == 'target_controller')
    boid = next(a for a in acts if a['name'] == 'boid_controller')
    assert tgt['parameters'][-1] == {'target_omega_max': 0.9}
    assert boid['parameters'][-1] == {'w_cohesion': 1.25}


def _evader_of(acts):
    for a in acts:
        for p in a['parameters']:
            if isinstance(p, dict) and 'evader' in p:
                return p['evader']
    return None


def test_unspecified_evader_is_smart_only_in_the_panel():
    smart = 'smart' in stack_config.available_evaders()
    ui = record('pursuit.launch.py', {'ui': 'true', 'headless': 'false'})
    flat = {}
    for p in ui[0]['parameters']:
        if isinstance(p, dict):
            flat.update(p)
    got = json.loads(flat['stack_launch_args'])['evader']
    assert got == ('smart' if smart else 'reactive')
    # headless / ui:=false: unchanged (golden covers the full node set)
    assert _evader_of(record('pursuit.launch.py', {'ui': 'false'})) \
        == 'reactive'
    # an explicit choice always wins in the panel path
    ui = record('pursuit.launch.py', {'ui': 'true', 'headless': 'false',
                                      'evader': 'reactive'})
    flat = {}
    for p in ui[0]['parameters']:
        if isinstance(p, dict):
            flat.update(p)
    assert json.loads(flat['stack_launch_args'])['evader'] == 'reactive'


def test_round_start_fairness_is_off_without_the_window():
    for ui_flags in ({'headless': 'true'}, {'ui': 'false'}):
        sim = record('pursuit.launch.py', ui_flags)[0]
        flat = {}
        for p in sim['parameters']:
            if isinstance(p, dict):
                flat.update(p)
        for k in stack_config.UI_SIM_DEFAULTS:
            assert k not in flat, (ui_flags, k)


def test_round_start_args_override_explicitly():
    sim = record('pursuit.launch.py', {
        'headless': 'true', 'spawn_safe': 'true', 'capture_grace': '1.5'})[0]
    flat = {}
    for p in sim['parameters']:
        if isinstance(p, dict):
            flat.update(p)
    assert flat['spawn_safe'] is True and flat['capture_grace'] == 1.5
