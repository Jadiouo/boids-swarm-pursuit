"""The restartable half of the pursuit game: controllers, target, Nav2.

    ros2 launch boids_swarm swarm_stack.launch.py \\
        num_agents:=8 evader:=smart perception:=sensor sharing_mode:=ros

Started and restarted by the sim's StackSupervisor when the control panel
switches mode (pursuit.launch.py with the panel on launches only the sim).
It takes the same argument names as pursuit.launch.py and builds the nodes
with the same `stack_actions`, so a panel-launched stack is parameter-for-
parameter the stack the headless launch would have started. Extra:

    agent_params / target_params   JSON dicts of live parameters carried
                                   over from the previous stack
    fail_fast                      exit the whole stack if a controller dies
                                   (default true: the panel reports it)
"""

import json

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from boids_swarm.stack_config import STACK_LAUNCH_KEYS
from boids_swarm.stack_supervisor import watch_parent
from boids_swarm.stack_launch import (
    params_file_path, resolve_world, stack_actions)

_DEFAULTS = {
    'num_agents': '8', 'strategy': 'auto', 'seed': '7',
    'evader': 'reactive', 'perception': 'perfect', 'sharing_mode': 'legacy',
    'env': 'custom', 'obstacles': '', 'game_mode': 'ai',
    'shared_sighting_qos_depth': '', 'relay_log_dir': '',
    'nav2_config': 'nav2_target.yaml', 'warmup': '0.0',
    'pursuer_delay': '0.0',
}


def launch_setup(context):
    watch_parent()     # sim gone (even SIGKILLed) => shut the stack down
    cfg = {k: LaunchConfiguration(k).perform(context)
           for k in STACK_LAUNCH_KEYS}
    if cfg['sharing_mode'] == 'ros' and cfg['perception'] != 'sensor':
        raise ValueError('sharing_mode=ros requires perception=sensor')
    params_file = params_file_path()
    world_size, obstacles = resolve_world(cfg, params_file)
    return stack_actions(
        cfg, params_file, world_size, obstacles,
        agent_params=json.loads(
            LaunchConfiguration('agent_params').perform(context) or '{}'),
        target_params=json.loads(
            LaunchConfiguration('target_params').perform(context) or '{}'),
        fail_fast=LaunchConfiguration('fail_fast').perform(
            context).lower() == 'true')


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=_DEFAULTS[k])
            for k in STACK_LAUNCH_KEYS]
    args += [
        DeclareLaunchArgument('agent_params', default_value='{}'),
        DeclareLaunchArgument('target_params', default_value='{}'),
        DeclareLaunchArgument('fail_fast', default_value='true'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
