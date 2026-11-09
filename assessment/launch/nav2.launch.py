#!/usr/bin/env python3
"""
Nav2 for task1, with its nodes idle until a node starts them. Included by assessment.launch.py.

Everything is under one lifecycle manager with autostart off, so its nodes come up unconfigured,
with simulation time set, and wait. A single call starts them all, in order:

  ros2 service call /lifecycle_manager_navigation/manage_nodes \
      nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"

Do that after /simulation_ready turns true: see assessment/nav2_startup.py for a node that does.

Nothing in the startup is limited by time. The manager goes through the nodes in order, each
transition finishing before the next starts, and what a node needs from the others is waited for
until it is there. Discovery between nodes is asynchronous, so no order alone could guarantee this.
A node that dies during startup therefore leaves it waiting, and the error below says why.

With keepout_mask set, the costmaps also keep out of the mask's black areas, as those nodes start
with the rest, and config/keepout_enabled.yaml enables the filter in nav2_params.yaml.

Bonds are off (bond_timeout 0, bond_heartbeat_period 0). slam_toolbox 2.10 fails to activate with
one, and bond timers run on wall time, so a process that stalls for 4 s on a busy machine would
make the manager reset all of Nav2, even if the simulation had slowed down with it. Instead, the
exit of a Nav2 process is logged as an error here. Nodes are not respawned: a respawned node comes
back unconfigured, with nothing to tell the lifecycle manager that it needs starting again.
"""

import os

from ament_index_python.packages import get_package_share_directory
from assessment.tasks import LIFECYCLE_MANAGER, nav2_lifecycle_nodes
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import launch.logging
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def on_exit(event, context):
    """Report a Nav2 process that exits while the launch is running."""
    if not context.is_shutdown:
        launch.logging.get_logger('assessment').error(
            f'{event.action.name} exited with code {event.returncode}: Nav2 will not work '
            'properly until it is launched again')


def nav2_nodes(context):
    mode = LaunchConfiguration('mode').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    keepout_mask = LaunchConfiguration('keepout_mask').perform(context)
    common = {'use_sim_time': True, 'bond_heartbeat_period': 0.0}
    params = [params_file, common]
    simulation_params = LaunchConfiguration('simulation_params').perform(context)
    if simulation_params:
        params.insert(1, simulation_params)
    if keepout_mask:
        params.insert(1, os.path.join(get_package_share_directory('assessment'), 'config',
                                      'keepout_enabled.yaml'))

    # Velocity commands go through the smoother on their way to the robot
    to_smoother = [('cmd_vel', 'cmd_vel_nav')]
    specs = [
        ('nav2_planner', 'planner_server', 'planner_server', params, []),
        ('nav2_controller', 'controller_server', 'controller_server', params, to_smoother),
        ('nav2_behaviors', 'behavior_server', 'behavior_server', params, to_smoother),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', params,
         to_smoother + [('cmd_vel_smoothed', 'cmd_vel')]),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', params, []),
        ('nav2_waypoint_follower', 'waypoint_follower', 'waypoint_follower', params, []),
    ]
    if mode == 'slam':
        specs.insert(0, ('slam_toolbox', 'async_slam_toolbox_node', 'slam_toolbox', params, []))
    else:
        map_file = LaunchConfiguration('map').perform(context)
        specs[:0] = [
            ('nav2_map_server', 'map_server', 'map_server',
             params + [{'yaml_filename': map_file}], []),
            ('nav2_amcl', 'amcl', 'amcl', params, []),
        ]

    if keepout_mask:
        specs += [
            ('nav2_map_server', 'map_server', 'keepout_filter_mask_server',
             params + [{'yaml_filename': keepout_mask}], []),
            ('nav2_map_server', 'costmap_filter_info_server',
             'keepout_costmap_filter_info_server', params, []),
        ]

    nodes = [Node(package=package, executable=executable, name=name, parameters=parameters,
                  remappings=remappings, output='screen')
             for package, executable, name, parameters, remappings in specs]
    manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name=LIFECYCLE_MANAGER,
        parameters=[{
            'use_sim_time': True,
            'autostart': False,
            'node_names': nav2_lifecycle_nodes(mode, keepout=bool(keepout_mask)),
            'bond_timeout': 0.0,
            'attempt_respawn_reconnection': False,
            # Each node gets as long as it needs: no limit on reaching it or on its transitions
            'service_timeout': -1.0,
        }],
        output='screen',
    )
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=on_exit))
                for node in nodes + [manager]]
    return handlers + nodes + [manager]


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('assessment'), 'config',
                                  'nav2_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('mode', choices=['slam', 'map'],
                              description='Build the map with SLAM, or localise on a map'),
        DeclareLaunchArgument('map', default_value='',
                              description="Map's YAML file, for mode map"),
        DeclareLaunchArgument('keepout_mask', default_value='',
                              description="Keepout mask's YAML file; empty: no keepout areas"),
        DeclareLaunchArgument('simulation_params', default_value='',
                              description='Parameters to apply on top of params_file'),
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Nav2 parameters file'),
        OpaqueFunction(function=nav2_nodes),
    ])
