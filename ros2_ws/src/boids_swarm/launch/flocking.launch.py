"""M2 — pure flocking, no target (SDD v3 §7.1).

    ros2 launch boids_swarm flocking.launch.py num_agents:=12
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    n = int(LaunchConfiguration('num_agents').perform(context))
    seed = int(LaunchConfiguration('seed').perform(context))
    headless = LaunchConfiguration('headless').perform(context)
    perception = LaunchConfiguration('perception').perform(context)
    params_file = os.path.join(get_package_share_directory('boids_swarm'),
                               'config', 'params.yaml')
    common = {'num_agents': n, 'seed': seed, 'target_enabled': False,
              'perception_mode': perception}

    actions = [Node(package='boids_swarm', executable='pygame_sim',
                    name='pygame_sim', output='screen',
                    parameters=[params_file, common, {
                        'headless': headless.lower() == 'true'}])]
    for i in range(n):
        actions.append(
            Node(package='boids_swarm', executable='boid_controller',
                 namespace=f'agent{i}', name='boid_controller',
                 output='screen',
                 # params.yaml tunes alignment/cohesion DOWN for pursuit
                 # (0.2); the pure-flocking demo reads better with the
                 # classic weights, so override them here.
                 parameters=[params_file, common,
                             {'w_alignment': 1.0, 'w_cohesion': 1.0,
                              'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_agents', default_value='8'),
        DeclareLaunchArgument('seed', default_value='7'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('perception', default_value='perfect'),
        OpaqueFunction(function=launch_setup),
    ])
