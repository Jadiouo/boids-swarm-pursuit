"""M7: Nav2 stack for the target (SDD v3 §6.4), approach B.

    ros2 launch boids_swarm nav2_target.launch.py [obstacles:="x,y,r;..."] \
        [cmd_topic:=/target/cmd_vel] [config:=nav2_target.yaml]

Starts nav2_bridge (map/TF/odom/boid cloud) + planner_server (navfn) +
controller_server (RegulatedPurePursuit by default, MPPI via
config:=nav2_target_mppi.yaml) + lifecycle_manager. All on sim
time. Controller output goes to `cmd_topic` (default /target/cmd_vel;
pursuit.launch.py passes /target/nav2_cmd_vel so target_controller can blend
it with the reactive evader). The controller speed/turn limits are NOT taken
from the yaml: they are derived from params.yaml
(agent_max_speed x target_speed_multiplier, target_omega_max) and override it.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from boids_swarm.launch_util import (
    load_ros_params, parse_obstacles, target_limits,
    write_derived_nav2_config)


def controller_overrides(cfg_path, v_max, w_max):
    """Controller-plugin parameter overrides carrying the sim's limits."""
    with open(cfg_path) as f:
        doc = yaml.safe_load(f)
    plugin = doc['controller_server']['ros__parameters'][
        'FollowPath']['plugin']
    if 'MPPI' in plugin:
        return {'FollowPath.vx_max': v_max, 'FollowPath.wz_max': w_max}
    if 'RegulatedPurePursuit' in plugin:
        # regulate to the tightest curve the target can actually turn:
        # R_min = v_max / omega_max (v = omega * R)
        return {'FollowPath.desired_linear_vel': v_max,
                'FollowPath.rotate_to_heading_angular_vel': w_max,
                'FollowPath.regulated_linear_scaling_min_radius':
                    v_max / w_max}
    return {}


def launch_setup(context):
    share = get_package_share_directory('boids_swarm')
    cfg_name = LaunchConfiguration('config').perform(context)
    cfg = cfg_name if os.path.isabs(cfg_name) else os.path.join(
        share, 'config', cfg_name)
    obstacles = parse_obstacles(
        LaunchConfiguration('obstacles').perform(context))
    world = float(LaunchConfiguration('world_size').perform(context))
    params = load_ros_params(os.path.join(share, 'config', 'params.yaml'))
    v_max, w_max, body_r = target_limits(params)
    # costmap footprint = the body the sim collides with (not a yaml literal)
    cfg = write_derived_nav2_config(cfg, body_r)
    ov = LaunchConfiguration('v_max').perform(context)
    v_max = float(ov) if ov else v_max
    sim = {'use_sim_time': True}
    remaps = [('cmd_vel', LaunchConfiguration('cmd_topic').perform(context))]
    ctrl_params = [cfg, sim, controller_overrides(cfg, v_max, w_max)]
    return [
        Node(package='boids_swarm', executable='nav2_bridge',
             name='nav2_bridge', output='screen',
             parameters=[sim, {'obstacles': obstacles, 'world_size': world}]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[cfg]),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=ctrl_params, remappings=remaps),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_target', output='screen',
             parameters=[cfg]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('obstacles', default_value=''),
        DeclareLaunchArgument('world_size', default_value='20.0'),
        DeclareLaunchArgument('cmd_topic', default_value='/target/cmd_vel'),
        DeclareLaunchArgument('config', default_value='nav2_target.yaml'),
        DeclareLaunchArgument('v_max', default_value='',
                              description='override derived target v_max'),
        OpaqueFunction(function=launch_setup),
    ])
