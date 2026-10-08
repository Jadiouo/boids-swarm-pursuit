"""Launch-argument cases that must keep producing the same node set from
pursuit.launch.py: every case here is headless or ui:=false, i.e. the path
the experiment scripts and ros_test use."""

CASES = {
    'default_ui_off': {'ui': 'false'},
    'headless_default': {'headless': 'true'},
    'sensor_ros': {'headless': 'true', 'perception': 'sensor',
                   'sharing_mode': 'ros', 'num_agents': '5', 'seed': '3',
                   'shared_sighting_qos_depth': '4', 'oracle_max_hops': '2',
                   'relay_log_dir': '/tmp/relay', 'time_scale': '1.0',
                   'strategy': 'pincer'},
    'sensor_oracle_obst': {'ui': 'false', 'perception': 'sensor',
                           'sharing_mode': 'oracle', 'env': 'obstacle_field',
                           'seed': '11', 'evader': 'adaptive'},
    'nav2_warm': {'headless': 'true', 'evader': 'nav2', 'env': 'pillar',
                  'perception': 'sensor', 'sharing_mode': 'ros',
                  'warmup': '8.0', 'pursuer_delay': '1.5',
                  'time_scale': '1.0', 'num_agents': '4'},
    'human_custom_obstacles': {'ui': 'false', 'game_mode': 'human',
                               'obstacles': '5,5,1.5;15,12,2'},
    'zones_shrink': {'headless': 'true', 'env': 'shrink', 'trails': 'true',
                     'stamina': 'false', 'capture_mode': 'tag',
                     'episodes_max': '3', 'time_limit': '40'},
}
