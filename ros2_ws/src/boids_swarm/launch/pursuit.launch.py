"""M3+ — full pursuit game: sim + N boids + evading target (SDD v3 §7.1).

    ros2 launch boids_swarm pursuit.launch.py \\
        num_agents:=12 strategy:=intercept game_mode:=ai seed:=7 \\
        headless:=false trails:=false obstacles:="5,5,1.5;15,12,2"
    # Nav2 evader (M7): also launches planner/controller/bridge
    ros2 launch boids_swarm pursuit.launch.py evader:=nav2 env:=obstacle_field
"""

import json

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, OpaqueFunction, Shutdown)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from boids_swarm import stack_config
from boids_swarm.stack_config import STACK_LAUNCH_KEYS
from boids_swarm.stack_launch import (
    params_file_path, resolve_world, stack_actions)


def launch_setup(context):
    cfg = {k: LaunchConfiguration(k).perform(context)
           for k in ('num_agents', 'strategy', 'game_mode', 'seed',
                     'headless', 'trails', 'capture_mode', 'episodes_max',
                     'time_limit', 'stamina', 'obstacles', 'evader',
                     'perception', 'env', 'shrink_rate', 'ui',
                     'ui_scale',
                     'screenshot_dir', 'screenshot_period', 'sharing_mode',
                     'shared_sighting_qos_depth', 'oracle_max_hops',
                     'relay_log_dir', 'time_scale', 'nav2_config', 'warmup',
                     'pursuer_delay')}
    ui_default_evader = cfg['evader'] == ''
    if ui_default_evader:
        cfg['evader'] = 'reactive'          # headless / ui:=false default
    if cfg['sharing_mode'] == 'ros' and cfg['perception'] != 'sensor':
        raise ValueError('sharing_mode=ros requires perception=sensor')
    # Procedural env (v4 M13): generate the layout in the launch so the sim
    # AND the controllers share the same obstacles; zones/shrink go to the
    # sim only. env=custom keeps the obstacles string arg (v3 behavior).
    params_file = params_file_path()
    world_size, obstacles = resolve_world(cfg, params_file)
    zones = [0.0]
    shrink_rate = float(cfg['shrink_rate'])
    if cfg['env'] != 'custom':
        from boids_swarm.world_gen import WorldGenerator
        layout = WorldGenerator(int(cfg['seed']), world_size).generate(
            cfg['env'])
        zones = layout.zones_flat()
        if layout.shrink_rate > 0.0:
            shrink_rate = layout.shrink_rate
    common = {
        'num_agents': int(cfg['num_agents']),
        'seed': int(cfg['seed']),
        'obstacles': obstacles,
        'target_enabled': True,
        'perception_mode': cfg['perception'],
        'sharing_mode': cfg['sharing_mode'],
    }
    sim_extra = {}
    if cfg['oracle_max_hops'] != '':
        sim_extra['oracle_max_hops'] = int(cfg['oracle_max_hops'])
    if cfg['time_scale'] != '':
        sim_extra['time_scale'] = float(cfg['time_scale'])
    if cfg['relay_log_dir'] != '':
        sim_extra['relay_log_dir'] = cfg['relay_log_dir']

    headless = cfg['headless'].lower() == 'true'
    ui_on = cfg['ui'].lower() == 'true'
    # Control-panel path: the window-owning sim is the only process launched
    # here; its StackSupervisor starts (and, on a mode switch, restarts) the
    # controllers / target / Nav2 as a separate swarm_stack.launch.py so the
    # window never closes. Headless and ui:=false keep launching everything
    # directly, exactly as before (experiment scripts depend on that).
    managed = ui_on and not headless
    if managed:
        # Window-only; omitted when headless / ui:=false so those launches
        # stay byte-identical to the pre-ui_scale golden.
        sim_extra['ui_scale'] = min(2.5, max(1.0, float(cfg['ui_scale'])))
    if managed and ui_default_evader:
        # Only the panel path defaults to the smart evader (when installed);
        # headless / ui:=false stay reactive for experiment reproducibility.
        cfg['evader'] = stack_config.resolve_evader(
            stack_config.PREFERRED_EVADER[stack_config.BASELINE],
            stack_config.available_evaders())
    if managed:
        sim_extra['stack_managed'] = True
        sim_extra['stack_launch_args'] = json.dumps(
            {k: cfg[k] for k in STACK_LAUNCH_KEYS}, sort_keys=True)

    actions = [
        Node(package='boids_swarm', executable='pygame_sim',
             name='pygame_sim', output='screen',
             on_exit=Shutdown(),        # episodes_max done => end the run
             parameters=[params_file, common, sim_extra, {
                 'headless': headless,
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
                 'ui_enabled': ui_on,
                 # Display-only: env_type stays 'custom' because the layout
                 # was already generated here (regenerating in the sim would
                 # be the double-generation trap from log.md #11), but the
                 # panel should still name the env the user asked for.
                 'env_label': cfg['env'],
                 'screenshot_dir': cfg['screenshot_dir'],
                 'screenshot_period': float(cfg['screenshot_period']),
             }]),
    ]
    if managed:
        return actions
    return actions + stack_actions(cfg, params_file, world_size, obstacles)


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
            'evader', default_value='',
            description='default: smart in the control-panel window, '
                        'reactive for headless / ui:=false. '
                        'reactive|adaptive (v4 M14)|smart (geodesic escape '
                        'planner, no Nav2)|nav2 (M7: also starts the '
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
            'ui_scale', default_value='1.5',
            description='scale of every UI size (fonts, panel, HUD): '
                        '1.0 = compact, 1.5 = default, up to 2.5 for '
                        'hi-dpi / far-away screens'),
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
