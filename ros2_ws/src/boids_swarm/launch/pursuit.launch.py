"""M3+ — full pursuit game: sim + N boids + evading target (SDD v3 §7.1).

    ros2 launch boids_swarm pursuit.launch.py \\
        num_agents:=12 strategy:=intercept game_mode:=ai seed:=7 \\
        headless:=false trails:=false obstacles:="5,5,1.5;15,12,2"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_obstacles(text: str):
    """'x,y,r;x,y,r' -> flat [x, y, r, x, y, r]. Empty -> sentinel [0.0]."""
    flat = []
    for chunk in text.split(';'):
        parts = [p for p in chunk.replace(' ', '').split(',') if p]
        if len(parts) == 3:
            flat += [float(v) for v in parts]
    return flat if flat else [0.0]


def launch_setup(context):
    cfg = {k: LaunchConfiguration(k).perform(context)
           for k in ('num_agents', 'strategy', 'game_mode', 'seed',
                     'headless', 'trails', 'capture_mode', 'episodes_max',
                     'time_limit', 'stamina', 'obstacles', 'evader',
                     'perception', 'env', 'shrink_rate',
                     'screenshot_dir', 'screenshot_period')}
    n = int(cfg['num_agents'])
    # Procedural env (v4 M13): generate the layout in the launch so the sim
    # AND the controllers share the same obstacles; zones/shrink go to the
    # sim only. env=custom keeps the obstacles string arg (v3 behavior).
    zones = [0.0]
    shrink_rate = float(cfg['shrink_rate'])
    if cfg['env'] != 'custom':
        from boids_swarm.world_gen import WorldGenerator
        layout = WorldGenerator(int(cfg['seed']), 20.0).generate(cfg['env'])
        obstacles = layout.obstacles_flat()
        zones = layout.zones_flat()
        if layout.shrink_rate > 0.0:
            shrink_rate = layout.shrink_rate
    else:
        obstacles = _parse_obstacles(cfg['obstacles'])
    params_file = os.path.join(get_package_share_directory('boids_swarm'),
                               'config', 'params.yaml')
    common = {
        'num_agents': n,
        'seed': int(cfg['seed']),
        'obstacles': obstacles,
        'target_enabled': True,
        'perception_mode': cfg['perception'],
    }
    # Controllers follow the sim's /clock so headless fast-forward stays
    # a fair benchmark (§7.4); the sim itself is the clock source.
    ctrl_common = dict(common, use_sim_time=True)

    actions = [
        Node(package='boids_swarm', executable='pygame_sim',
             name='pygame_sim', output='screen',
             on_exit=Shutdown(),        # episodes_max done ⇒ end the run
             parameters=[params_file, common, {
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
                 'screenshot_dir': cfg['screenshot_dir'],
                 'screenshot_period': float(cfg['screenshot_period']),
             }]),
    ]
    if cfg['game_mode'] == 'ai':
        actions.append(
            Node(package='boids_swarm', executable='target_controller',
                 name='target_controller', output='screen',
                 parameters=[params_file, ctrl_common,
                             {'evader': cfg['evader']}]))
    for i in range(n):
        actions.append(
            Node(package='boids_swarm', executable='boid_controller',
                 namespace=f'agent{i}', name='boid_controller',
                 output='screen',
                 parameters=[params_file, ctrl_common,
                             {'pursuit_strategy': cfg['strategy']}]))
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
        DeclareLaunchArgument('evader', default_value='reactive'),
        DeclareLaunchArgument('seed', default_value='7'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('trails', default_value='false'),
        DeclareLaunchArgument('capture_mode', default_value='hull'),
        DeclareLaunchArgument('episodes_max', default_value='0'),
        DeclareLaunchArgument('time_limit', default_value='90.0'),
        DeclareLaunchArgument('stamina', default_value='true'),
        DeclareLaunchArgument('obstacles', default_value='',
                              description='"x,y,r;x,y,r" circles'),
        DeclareLaunchArgument('perception', default_value='perfect',
                              description='perfect|sensor (v4 Part A)'),
        DeclareLaunchArgument(
            'env', default_value='custom',
            description='custom|open|obstacle_field|pillar|zones|shrink'),
        DeclareLaunchArgument('shrink_rate', default_value='0.0',
                              description='arena shrink units/s (v4 C.4)'),
        DeclareLaunchArgument('screenshot_dir', default_value='',
                              description='save periodic PNG frames here'),
        DeclareLaunchArgument('screenshot_period', default_value='5.0'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
