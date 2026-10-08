"""Mode -> stack-argument mapping, exclusion rules, override merging."""

import json

import pytest

from boids_swarm import stack_config as sc

ALL = sc.ALL_EVADERS


def test_baseline_is_perfect_legacy_smart():
    p = sc.mode_preset(sc.BASELINE, ALL)
    assert p == {'perception': 'perfect', 'sharing_mode': 'legacy',
                 'evader': 'smart'}


def test_sensor_ros_and_nav2_presets():
    assert sc.mode_preset(sc.SENSOR_ROS, ALL) == {
        'perception': 'sensor', 'sharing_mode': 'ros', 'evader': 'smart'}
    p = sc.mode_preset(sc.NAV2, ALL)
    assert (p['perception'], p['sharing_mode'], p['evader']) == \
        ('sensor', 'ros', 'nav2')


def test_smart_falls_back_to_reactive_when_target_controller_lacks_it():
    """Until the smart brain is merged, TargetController.BRAINS has no
    'smart'; asking for it would crash the target at construction."""
    core = ('reactive', 'adaptive', 'nav2')
    assert sc.mode_preset(sc.BASELINE, core)['evader'] == 'reactive'
    assert sc.mode_preset(sc.SENSOR_ROS, core)['evader'] == 'reactive'
    assert sc.mode_preset(sc.NAV2, core)['evader'] == 'nav2'


def test_unknown_mode_is_rejected():
    with pytest.raises(sc.ConfigError):
        sc.mode_preset('turbo')


def test_ros_sharing_requires_sensor_perception():
    errs = sc.validate({'perception': 'perfect', 'sharing_mode': 'ros',
                        'evader': 'reactive', 'env': 'open',
                        'num_agents': 8}, ALL)
    assert any('requires perception=sensor' in e for e in errs)


def test_valid_configs_have_no_errors():
    for m in sc.MODES:
        cfg = dict(sc.mode_preset(m, ALL), env='open', num_agents=8,
                   shared_sighting_qos_depth=10)
        assert sc.validate(cfg, ALL) == [], m


def test_unavailable_evader_is_rejected():
    cfg = dict(sc.mode_preset(sc.BASELINE, ALL), env='open', num_agents=8,
               evader='smart')
    assert any('smart' in e for e in sc.validate(cfg, ('reactive',)))


@pytest.mark.parametrize('n', [0, 1, 25, 'x'])
def test_num_agents_range_is_checked(n):
    cfg = dict(sc.mode_preset(sc.BASELINE, ALL), env='open', num_agents=n)
    assert sc.validate(cfg, ALL)


def test_bad_env_and_qos_depth_rejected():
    cfg = dict(sc.mode_preset(sc.BASELINE, ALL), env='lava', num_agents=8,
               shared_sighting_qos_depth=0)
    msgs = ' '.join(sc.validate(cfg, ALL))
    assert 'env' in msgs and 'qos' in msgs


def test_stack_args_for_each_mode():
    a = sc.stack_args(sc.SENSOR_ROS, base={'num_agents': 5, 'seed': 3})
    assert a['perception'] == 'sensor' and a['sharing_mode'] == 'ros'
    assert a['num_agents'] == '5' and a['seed'] == '3'
    b = sc.stack_args(sc.BASELINE, available=('reactive', 'adaptive'))
    assert (b['perception'], b['sharing_mode'], b['evader']) == \
        ('perfect', 'legacy', 'reactive')
    c = sc.stack_args(sc.NAV2)
    assert c['evader'] == 'nav2'


def test_nav2_gets_default_warmup_and_others_none():
    assert float(sc.stack_args(sc.NAV2)['warmup']) == sc.NAV2_WARMUP_S
    assert float(sc.stack_args(sc.SENSOR_ROS)['warmup']) == 0.0
    # an explicit launch warmup is respected
    assert float(sc.stack_args(sc.NAV2, base={'warmup': 9.0})['warmup']) == 9.0


def test_overrides_win_over_the_preset_but_are_still_validated():
    a = sc.stack_args(sc.SENSOR_ROS, overrides={'evader': 'adaptive',
                                                'num_agents': 6})
    assert a['evader'] == 'adaptive' and a['num_agents'] == '6'
    with pytest.raises(sc.ConfigError):
        sc.stack_args(sc.SENSOR_ROS, overrides={'perception': 'perfect'})


def test_mode_none_keeps_base_and_applies_overrides():
    base = {'perception': 'sensor', 'sharing_mode': 'oracle',
            'evader': 'reactive'}
    a = sc.stack_args(None, overrides={'num_agents': 4}, base=base)
    assert a['sharing_mode'] == 'oracle' and a['num_agents'] == '4'


def test_live_params_are_forwarded_as_json_and_filtered():
    a = sc.stack_args(sc.BASELINE, live={
        'w_cohesion': 0.9, 'target_omega_max': 1.4, 'not_a_param': 1})
    assert json.loads(a['agent_params']) == {'w_cohesion': 0.9}
    assert json.loads(a['target_params']) == {'target_omega_max': 1.4}


def test_all_values_are_strings_for_the_command_line():
    a = sc.stack_args(sc.NAV2, base={'num_agents': 8})
    assert all(isinstance(v, str) for v in a.values())
    cmd = sc.stack_command(a)
    assert cmd[:4] == ['ros2', 'launch', 'boids_swarm',
                       'swarm_stack.launch.py']
    assert 'evader:=nav2' in cmd


def test_stack_args_keys_cover_the_launch_keys():
    a = sc.stack_args(sc.BASELINE)
    assert set(sc.STACK_LAUNCH_KEYS) <= set(a)


def test_mode_of_roundtrips_and_ignores_the_brain_choice():
    for m in sc.MODES:
        assert sc.mode_of(sc.mode_preset(m, ALL)) == m
    assert sc.mode_of({'perception': 'perfect', 'sharing_mode': 'legacy',
                       'evader': 'adaptive'}) == sc.BASELINE
    assert sc.mode_of({'perception': 'sensor', 'sharing_mode': 'oracle',
                       'evader': 'reactive'}) is None


def test_expected_nodes_lists_controllers_and_target():
    names = sc.expected_nodes({'num_agents': 3, 'game_mode': 'ai'})
    assert names == ['/agent0/boid_controller', '/agent1/boid_controller',
                     '/agent2/boid_controller', '/target_controller']
    assert '/target_controller' not in sc.expected_nodes(
        {'num_agents': 2, 'game_mode': 'human'})


def test_available_evaders_always_has_the_core_pair():
    av = sc.available_evaders()
    assert set(sc.CORE_EVADERS) <= set(av)


# --- /ui/mode_request payloads ---------------------------------------------

def test_request_bare_mode_name():
    ov = sc.parse_mode_request('sensor_ros')
    assert ov['perception'] == 'sensor' and ov['sharing_mode'] == 'ros'


def test_request_json_with_overrides_and_no_mode():
    assert sc.parse_mode_request('{"overrides": {"num_agents": 4}}') == \
        {'num_agents': 4}
    ov = sc.parse_mode_request('{"mode":"nav2","overrides":{"seed":3}}')
    assert ov['evader'] == 'nav2' and ov['seed'] == 3


@pytest.mark.parametrize('bad', ['', 'warp', '{"mode": "warp"}', '{oops',
                                 '{"overrides": {"w_pursuit": 1}}'])
def test_bad_requests_raise_config_error(bad):
    with pytest.raises(sc.ConfigError):
        sc.parse_mode_request(bad)


def test_empty_values_are_not_passed_to_ros2_launch():
    """`ros2 launch obstacles:=` is rejected as a malformed argument."""
    a = sc.stack_args(sc.BASELINE)
    assert a['obstacles'] == '' and a['relay_log_dir'] == ''
    cmd = sc.stack_command(a)
    assert not any(c.endswith(':=') for c in cmd)
    assert 'perception:=perfect' in cmd
