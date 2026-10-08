"""M3+ — full pursuit game: sim + N boids + evading target (SDD v3 §7.1).

    ros2 launch boids_swarm pursuit.launch.py \\
        num_agents:=12 strategy:=intercept game_mode:=ai seed:=7 \\
        headless:=false trails:=false obstacles:="5,5,1.5;15,12,2"
    # Nav2 evader (M7): also launches planner/controller/bridge
    ros2 launch boids_swarm pursuit.launch.py evader:=nav2 env:=obstacle_field
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown,
    TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from boids_swarm.launch_util import format_obstacles, obstacles_for_env


def _world_size(params_file):
    """Read world_size from params.yaml so the launch-side WorldGenerator
    scales to the SAME arena the sim will use. Hard-coding 20.0 here made
    a changed world_size silently produce a mis-scaled procedural map."""
    try:
        with open(params_file) as f:
            doc = yaml.safe_load(f) or {}
        return float(doc['/**']['ros__parameters']['world_size'])
    except (OSError, KeyError, TypeError, ValueError):
        return 20.0


def launch_setup(context):
    cfg = {k: LaunchConfiguration(k).perform(context)
           for k in ('num_agents', 'strategy', 'game_mode', 'seed',
                     'headless', 'trails', 'capture_mode', 'episodes_max',
                     'time_limit', 'stamina', 'obstacles', 'evader',
                     'perception', 'env', 'shrink_rate', 'ui',
                     'screenshot_dir', 'screenshot_period', 'sharing_mode',
                     'shared_sighting_qos_depth', 'oracle_max_hops',
                     'relay_log_dir', 'time_scale', 'nav2_config', 'warmup',
                     'pursuer_delay')}
    n = int(cfg['num_agents'])
    if cfg['sharing_mode'] == 'ros' and cfg['perception'] != 'sensor':
        raise ValueError('sharing_mode=ros requires perception=sensor')
    # Procedural env (v4 M13): generate the layout in the launch so the sim
    # AND the controllers share the same obstacles; zones/shrink go to the
    # sim only. env=custom keeps the obstacles string arg (v3 behavior).
    params_file = os.path.join(get_package_share_directory('boids_swarm'),
                               'config', 'params.yaml')
    world_size = _world_size(params_file)
    zones = [0.0]
    shrink_rate = float(cfg['shrink_rate'])
    # ONE obstacle list for sim, controllers and the Nav2 bridge (M7).
    obstacles = obstacles_for_env(cfg['env'], int(cfg['seed']), world_size,
                                  cfg['obstacles'])
    if cfg['env'] != 'custom':
        from boids_swarm.world_gen import WorldGenerator
        layout = WorldGenerator(int(cfg['seed']), world_size).generate(
            cfg['env'])
        zones = layout.zones_flat()
        if layout.shrink_rate > 0.0:
            shrink_rate = layout.shrink_rate
    common = {
        'num_agents': n,
        'seed': int(cfg['seed']),
        'obstacles': obstacles,
        'target_enabled': True,
        'perception_mode': cfg['perception'],
        'sharing_mode': cfg['sharing_mode'],
    }
    # Controllers follow the sim's /clock so headless fast-forward stays
    # a fair benchmark (§7.4); the sim itself is the clock source.
    ctrl_common = dict(common, use_sim_time=True)
    # Phase 2: empty launch value = keep the params.yaml value.
    sim_extra, ctrl_extra = {}, {}
    if cfg['shared_sighting_qos_depth'] != '':
        ctrl_extra['shared_sighting_qos_depth'] = int(
            cfg['shared_sighting_qos_depth'])
    if cfg['oracle_max_hops'] != '':
        sim_extra['oracle_max_hops'] = int(cfg['oracle_max_hops'])
    if cfg['time_scale'] != '':
        sim_extra['time_scale'] = float(cfg['time_scale'])
    if cfg['relay_log_dir'] != '':
        sim_extra['relay_log_dir'] = cfg['relay_log_dir']
        ctrl_extra['relay_log_dir'] = cfg['relay_log_dir']

    actions = [
        Node(package='boids_swarm', executable='pygame_sim',
             name='pygame_sim', output='screen',
             on_exit=Shutdown(),        # episodes_max done ⇒ end the run
             parameters=[params_file, common, sim_extra, {
                 'headless': cfg['headless'].lower() == 'true',
                 'render_trails': cfg['trails'].lower() == 'true',
                 'game_mode': cfg['game_mode'],
                 'capture_mode': cfg['capture_mode'],
                 'episodes_max': int(cfg['episodes_max']),
                 'episode_time_limit': float(cfg['time_limit']),
                 'target_stamina_enabled': cfg['stamina'].lower() == 'true',
                 'env_type': 'custom',   # layout already generated in launch
                 'zones': zones,
                 'shrink_rate': shrink_rate,
                 # Mirrors for the control panel (M15): these parameters
                 # really live on the controllers, but the sim draws them,
                 # so it must start from the SAME launch args they got.
                 'pursuit_strategy': cfg['strategy'],
                 'evader': cfg['evader'],
                 'ui_enabled': cfg['ui'].lower() == 'true',
                 # Display-only: env_type stays 'custom' because the layout
                 # was already generated here (regenerating in the sim would
                 # be the double-generation trap from log.md #11), but the
                 # panel should still name the env the user asked for.
                 'env_label': cfg['env'],
                 'screenshot_dir': cfg['screenshot_dir'],
                 'screenshot_period': float(cfg['screenshot_period']),
             }]),
    ]
    # Controllers can be held back `warmup` seconds so the Nav2 lifecycle
    # (~5 s) is active before anything moves (benchmarks); 0 = start at once.
    warmup = float(cfg['warmup'])
    delay = float(cfg['pursuer_delay'])
    ctrl_nodes, boid_nodes = [], []
    if cfg['game_mode'] == 'ai':
        ctrl_nodes.append(
            Node(package='boids_swarm', executable='target_controller',
                 name='target_controller', output='screen',
                 parameters=[params_file, ctrl_common,
                             {'evader': cfg['evader']}]))
    if cfg['evader'] == 'nav2':
        # Nav2 stack for the target: bridge + planner + controller. Its cmd
        # goes to /target/nav2_cmd_vel; target_controller blends it with the
        # reactive term and is the sole publisher of /target/cmd_vel.
        share = get_package_share_directory('boids_swarm')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                share, 'launch', 'nav2_target.launch.py')),
            launch_arguments={
                'obstacles': format_obstacles(obstacles),
                'world_size': repr(world_size),
                'cmd_topic': '/target/nav2_cmd_vel',
                'config': cfg['nav2_config'],
            }.items()))
    for i in range(n):
        boid_nodes.append(
            Node(package='boids_swarm', executable='boid_controller',
                 namespace=f'agent{i}', name='boid_controller',
                 output='screen',
                 parameters=[params_file, ctrl_common, ctrl_extra,
                             {'pursuit_strategy': cfg['strategy']}]))
    # `pursuer_delay` additionally holds the boids back after the target is
    # up: the Nav2 target process starts slower than a boid, and without it the
    # pursuers get a startup head start that depends on process timing.
    for t, nodes in ((warmup, ctrl_nodes), (warmup + delay, boid_nodes)):
        if t > 0.0:
            actions.append(TimerAction(period=t, actions=nodes))
        else:
            actions += nodes
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument('num_agents', default_value='8'),
        DeclareLaunchArgument(
            'strategy', default_value='auto',
            description='auto|naive|intercept|pincer|encircle|herd|'
                        'counter_rotate|blockade|corner_trap|herd_inward|'
                        'sweep|role_encircle|bait'),
        DeclareLaunchArgument('game_mode', default_value='ai',
                              description='ai|human'),
        DeclareLaunchArgument(
            'evader', default_value='reactive',
            description='reactive|adaptive (v4 M14)|nav2 (M7: also starts the '
                        'Nav2 stack for the target)'),
        DeclareLaunchArgument('nav2_config', default_value='nav2_target.yaml',
                              description='Nav2 yaml for evader:=nav2'),
        DeclareLaunchArgument('seed', default_value='7'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('trails', default_value='false'),
        DeclareLaunchArgument('capture_mode', default_value='hull'),
        DeclareLaunchArgument('episodes_max', default_value='0'),
        DeclareLaunchArgument('time_limit', default_value='90.0'),
        DeclareLaunchArgument('stamina', default_value='true'),
        DeclareLaunchArgument('obstacles', default_value='',
                              description='"x,y,r;x,y,r" circles'),
        DeclareLaunchArgument('sharing_mode', default_value='legacy',
                              description='legacy|off|oracle|ros'),
        DeclareLaunchArgument(
            'shared_sighting_qos_depth', default_value='',
            description='controller shared-sighting QoS depth (empty = '
                        'params.yaml, default 10)'),
        DeclareLaunchArgument(
            'oracle_max_hops', default_value='',
            description='oracle relay hop limit, 0 = unlimited (empty = '
                        'params.yaml)'),
        DeclareLaunchArgument(
            'relay_log_dir', default_value='',
            description='write per-run relay accounting files here'),
        DeclareLaunchArgument(
            'time_scale', default_value='',
            description='headless fast-forward factor (empty = sim default 4.0; '
                        '1 = real time; evader:=nav2 is only fair at 1.0 '
                        'because Nav2 runs on wall-clock CPU)'),
        DeclareLaunchArgument('perception', default_value='perfect',
                              description='perfect|sensor (v4 Part A)'),
        DeclareLaunchArgument(
            'env', default_value='custom',
            description='custom|open|obstacle_field|pillar|zones|shrink'),
        DeclareLaunchArgument('shrink_rate', default_value='0.0',
                              description='arena shrink units/s (v4 C.4)'),
        DeclareLaunchArgument(
            'ui', default_value='true',
            description='in-window control panel (M15); ignored if headless'),
        DeclareLaunchArgument(
            'pursuer_delay', default_value='0.0',
            description='extra seconds the boids wait after the target '
                        'controller started (benchmark fairness)'),
        DeclareLaunchArgument(
            'warmup', default_value='0.0',
            description='seconds to hold back target+boid controllers '
                        '(let Nav2 activate before the chase starts)'),
        DeclareLaunchArgument('screenshot_dir', default_value='',
                              description='save periodic PNG frames here'),
        DeclareLaunchArgument('screenshot_period', default_value='5.0'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
