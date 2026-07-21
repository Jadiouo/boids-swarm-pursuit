"""Launch the full Boids swarm (SDD v2 §2.2).

    ros2 launch boids_turtlesim boids.launch.py num_turtles:=8

Starts turtlesim_node, the swarm_spawner, and one boid_controller per
turtle (namespaces /turtle1../turtleN).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    n = int(LaunchConfiguration('num_turtles').perform(context))
    seed = int(LaunchConfiguration('seed').perform(context))
    names = [f'turtle{i}' for i in range(1, n + 1)]
    params_file = os.path.join(
        get_package_share_directory('boids_turtlesim'),
        'config', 'boids_params.yaml')

    actions = [
        Node(package='turtlesim', executable='turtlesim_node',
             name='turtlesim', output='screen'),
        Node(package='boids_turtlesim', executable='swarm_spawner',
             name='swarm_spawner', output='screen',
             parameters=[{'num_turtles': n,
                          'kill_default_turtle': True,
                          'seed': seed}]),
    ]
    for name in names:
        actions.append(
            Node(package='boids_turtlesim', executable='boid_controller',
                 namespace=name, name='boid_controller', output='screen',
                 parameters=[params_file, {'peers': names}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_turtles', default_value='6',
                              description='Number of boid agents N'),
        DeclareLaunchArgument('seed', default_value='-1',
                              description='Spawn RNG seed (-1 = random)'),
        OpaqueFunction(function=launch_setup),
    ])
