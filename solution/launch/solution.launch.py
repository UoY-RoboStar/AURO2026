#!/usr/bin/env python3
"""
Runs the assessment with this solution.

  ros2 launch solution solution.launch.py              (task2)
  ros2 launch solution solution.launch.py task:=task1

To change anything, add to the tables below: an entry becomes a launch argument with that default,
so it can also be given on the command line:

  ros2 launch solution solution.launch.py task:=task1 nav2:=package://solution/maps/my_map.yaml

Refer to files in this package as package://solution/<path>, and install them in setup.py. Absolute
paths are strongly discouraged as they may stop working when the workspace moves.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

TASKS = ('task1', 'task2')

# Arguments of assessment.launch.py, with their defaults here. To see all that it has:
#   ros2 launch assessment assessment.launch.py --show-args
# All of them have to be here, so that the solution can be run with any of them: check.py, at the
# root of the workspace, checks. An empty value means not set: the argument is not passed on, so
# assessment.launch.py's own default applies. An argument can have a different default for each
# task, by being in both tasks' tables.
ASSESSMENT_ARGUMENTS = {
    'all': {
        'rviz': 'false',
        'rviz_config_file': '',
        'rviz_log_level': 'info',
        'random_seed': '',
        'headless': 'false',
        'headless_rendering': 'false',
        'limit_real_time_factor': 'true',
        'gazebo_verbose': '2',
        'sim_time_log': 'false',
        'sim_time_log_file': '',
    },
    'task1': {
        'nav2': 'false',
        # Not set: the assessment's default, auto, applies: with Nav2, the launch starts it once
        # the simulation is ready. manual: Nav2 waits for a node to start it
        'nav2_start': '',
        'nav2_params': '',
        'keepout_mask': '',
        'end_effectors': 'true',
        'time_limit': '300',  # s of simulation time: 5 minutes
    },
    'task2': {
        'time_limit': '900',  # s of simulation time: 15 minutes
        'target_power_level': '50',
        'maximum_power_level': '',
        'control_rod_coefficient': '0.20',
        'power_display': 'true',
        'reactor_fill_percentage': '80',
        'replacement_percentage': '20',
        'initial_mean_level': '50',
        'initial_level_distribution': 'uniform',
        'initial_level_spread': '25',
        'replacement_level': '100',
        'winch_camera_rate': '10',
        'winch_camera_noise': '0',
        'winch_failure_probability': '0',
        'winch_cable': 'true',
    },
}

# Arguments for this solution's own nodes, which get them as ROS parameters. Values are strings
# here, as on the command line: declare them in the node with the type it needs.
NODE_ARGUMENTS = {
    'task1': {
    },
    'task2': {
    },
}


def arguments_of(table, task):
    return {**table.get('all', {}), **table[task]}


def launch_task(context):
    task = LaunchConfiguration('task').perform(context)

    def values(table):
        # Not given, an argument with a default for each task has the task's (see
        # generate_launch_description)
        return {name: LaunchConfiguration(name).perform(context) or default
                for name, default in arguments_of(table, task).items()}

    # Those not set are left out, for assessment.launch.py's defaults
    assessment = {name: value for name, value in values(ASSESSMENT_ARGUMENTS).items()
                  if value != ''}
    parameters = values(NODE_ARGUMENTS)

    if task == 'task1':
        # What tells the node how Nav2 finds out where the robot is
        parameters['localizer'] = 'slam_toolbox' if assessment.get('nav2') == 'slam' else 'amcl'

    return [
        # Included launch files see the arguments of this one: without forwarding off, task2 would
        # get task1's as well
        GroupAction(forwarding=False, launch_configurations={'task': task, **assessment}, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('assessment'), 'launch', 'assessment.launch.py'))),
        ]),
        Node(package='solution', executable=task, parameters=[parameters], output='screen'),
    ]


def generate_launch_description():
    # An argument whose default differs between tables, i.e. tasks, is declared without one
    # (empty): given no value, it takes the task's, in launch_task
    defaults = {}
    for table in (ASSESSMENT_ARGUMENTS, NODE_ARGUMENTS):
        for arguments in table.values():
            for name, value in arguments.items():
                defaults[name] = value if defaults.get(name, value) == value else ''
    return LaunchDescription([
        # !!DO NOT REMOVE!! use_sim_time as true.
        SetParameter(name='use_sim_time', value=True),
        # Declaring other launch arguments.
        DeclareLaunchArgument('task', default_value='task2', choices=list(TASKS),
                              description='The task to run'),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=launch_task),
    ])
