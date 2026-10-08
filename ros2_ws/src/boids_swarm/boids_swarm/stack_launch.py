"""Launch-time builders shared by pursuit.launch.py and swarm_stack.launch.py.

pursuit.launch.py (headless / ui:=false) and the panel-managed
swarm_stack.launch.py must start the SAME controllers with the SAME
parameters, so both call `stack_actions` / `resolve_world` here instead of
keeping two copies. Imports launch / launch_ros, so only launch files (and
the launch-compat test) import this module.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    IncludeLaunchDescription, Shutdown, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from .launch_util import format_obstacles, obstacles_for_env


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


def params_file_path():
    return os.path.join(get_package_share_directory('boids_swarm'),
                        'config', 'params.yaml')


def resolve_world(cfg, params_file):
    """(world_size, obstacles_flat) for a launch cfg of strings."""
    world_size = _world_size(params_file)
    # ONE obstacle list for sim, controllers and the Nav2 bridge (M7).
    obstacles = obstacles_for_env(cfg['env'], int(cfg['seed']), world_size,
                                  cfg['obstacles'])
    return world_size, obstacles


def stack_actions(cfg, params_file, world_size, obstacles,
                  agent_params=None, target_params=None, fail_fast=False):
    """target_controller, (Nav2 stack), N x boid_controller.

    `agent_params` / `target_params` are extra live-parameter dicts layered
    last (the panel carries tuning across restarts through them); empty for
    the plain launch, which keeps its parameter lists unchanged.
    `fail_fast` shuts the whole stack down if a controller dies, so the
    panel's supervisor sees the exit and reports why instead of running
    half a swarm.
    """
    n = int(cfg['num_agents'])
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
    ctrl_extra = {}
    if cfg['shared_sighting_qos_depth'] != '':
        ctrl_extra['shared_sighting_qos_depth'] = int(
            cfg['shared_sighting_qos_depth'])
    if cfg['relay_log_dir'] != '':
        ctrl_extra['relay_log_dir'] = cfg['relay_log_dir']
    exit_kw = {'on_exit': Shutdown()} if fail_fast else {}

    actions = []
    # Controllers can be held back `warmup` seconds so the Nav2 lifecycle
    # (~5 s) is active before anything moves (benchmarks); 0 = start at once.
    warmup = float(cfg['warmup'])
    delay = float(cfg['pursuer_delay'])
    ctrl_nodes, boid_nodes = [], []
    if cfg['game_mode'] == 'ai':
        tparams = [params_file, ctrl_common, {'evader': cfg['evader']}]
        if target_params:
            tparams.append(dict(target_params))
        ctrl_nodes.append(
            Node(package='boids_swarm', executable='target_controller',
                 name='target_controller', output='screen',
                 parameters=tparams, **exit_kw))
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
        bparams = [params_file, ctrl_common, ctrl_extra,
                   {'pursuit_strategy': cfg['strategy']}]
        if agent_params:
            bparams.append(dict(agent_params))
        boid_nodes.append(
            Node(package='boids_swarm', executable='boid_controller',
                 namespace=f'agent{i}', name='boid_controller',
                 output='screen', parameters=bparams, **exit_kw))
    # `pursuer_delay` additionally holds the boids back after the target is
    # up: the Nav2 target process starts slower than a boid, and without it the
    # pursuers get a startup head start that depends on process timing.
    for t, nodes in ((warmup, ctrl_nodes), (warmup + delay, boid_nodes)):
        if t > 0.0:
            actions.append(TimerAction(period=t, actions=nodes))
        else:
            actions += nodes
    return actions
