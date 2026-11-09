#!/usr/bin/env python3
"""
Simulate a TurtleBot3 in an arena, publishing wheel odometry and ground truth side by side.

  /odom        from the DiffDrive system, integrated from the wheels; also odom -> base_footprint
  /odom_world  from the OdometryPublisher system: the true pose, in the world frame
  /odom_path, /true_path  the same two, as paths for RViz
  /scan_true   /scan, from the true pose, for RViz: /scan moves with the odometry's errors

Two options add things to look at, each with a node publishing a vision_msgs/Detection2DArray:

  ros2 launch week_6 launch.py aruco:=true    (tags on the walls, /aruco_detections)
  ros2 launch week_6 launch.py barrel:=true   (a red barrel, /barrel_detections)

  ros2 launch week_6 launch.py encoder_bias_left:=1.05   (a fixed ratio)
  ros2 launch week_6 launch.py wheel_slip_left:=10       (real slip, varying with speed and load)
"""

import atexit
import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from week_6.aruco_tags import write_tags
from week_6.robot_model import configure_robot_model

SPAWN_Z = '0.01'
# In view from the default start pose, without blocking the way forwards
BARREL_X = '1.5'
BARREL_Y = '-0.6'


def temporary_file(suffix):
    """Return the path of a new file, removed when the launch exits."""
    fd, path = tempfile.mkstemp(prefix='week_6_', suffix=suffix)
    os.close(fd)
    atexit.register(os.remove, path)
    return path


def generate_launch_description():
    # turtlebot3_gazebo's robot_state_publisher.launch.py reads this variable
    turtlebot3_model = 'waffle_pi'
    os.environ['TURTLEBOT3_MODEL'] = turtlebot3_model
    model_folder = f'turtlebot3_{turtlebot3_model}'

    turtlebot3_share = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    week_6_share = get_package_share_directory('week_6')

    def number(context, name):
        value = LaunchConfiguration(name).perform(context)
        try:
            return float(value)
        except ValueError:
            raise RuntimeError(f'{name} must be a number, got {value!r}')

    def spawn_robot(context):
        tree = ET.parse(os.path.join(turtlebot3_share, 'models', model_folder, 'model.sdf'))
        configure_robot_model(tree, number(context, 'encoder_bias_left'),
                              number(context, 'encoder_bias_right'),
                              number(context, 'wheel_slip_left'),
                              number(context, 'wheel_slip_right'))
        robot_model = temporary_file('.sdf')
        tree.write(robot_model, encoding='utf-8', xml_declaration=True)

        return [
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', turtlebot3_model,
                           '-file', robot_model,
                           '-x', LaunchConfiguration('x_pose'),
                           '-y', LaunchConfiguration('y_pose'),
                           '-z', SPAWN_Z,
                           '-Y', LaunchConfiguration('yaw_pose')],
                output='screen',
            ),
            Node(
                package='week_6',
                executable='path_publisher',
                parameters=[{'x_pose': number(context, 'x_pose'),
                             'y_pose': number(context, 'y_pose'),
                             'yaw': number(context, 'yaw_pose')}],
                output='screen',
            ),
        ]

    def spawn_tags(context):
        return [Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', 'aruco_tags', '-file', write_tags(temporary_file)],
            output='screen',
        )]

    ld = LaunchDescription()

    ld.add_action(SetParameter(name='use_sim_time', value=True))

    for name, default, description in (
        ('encoder_bias_left', '1.0',
         'Left wheel travel the odometry reports per metre really travelled; 1.0: no slip'),
        ('encoder_bias_right', '1.0',
         'Right wheel travel the odometry reports per metre really travelled; 1.0: no slip'),
        ('wheel_slip_left', '0',
         'Slip compliance of the left wheel, for real slip that varies with speed and load; '
         '0: none, 10: about 4% at 0.2 m/s'),
        ('wheel_slip_right', '0',
         'Slip compliance of the right wheel, for real slip that varies with speed and load; '
         '0: none, 10: about 4% at 0.2 m/s'),
        ('x_pose', '0.53', 'Robot start position in x (m)'),
        ('y_pose', '-1.0', 'Robot start position in y (m)'),
        ('yaw_pose', '0.0', 'Robot start heading (rad)'),
        ('headless', 'false', 'Whether to run without the Gazebo GUI'),
        ('use_rviz', 'true', 'Whether to start RViz'),
        ('aruco', 'false', 'Whether to add ArUco tags to the walls and run aruco_detector'),
        ('barrel', 'false', 'Whether to add a barrel to the arena and run visual_sensor'),
    ):
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))

    ld.add_action(SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [os.path.join(turtlebot3_share, 'models'), ':',
         EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')]))

    # First, so that a bad argument stops the launch before there is a Gazebo to leave behind
    ld.add_action(OpaqueFunction(function=spawn_robot))
    ld.add_action(OpaqueFunction(function=spawn_tags,
                                 condition=IfCondition(LaunchConfiguration('aruco'))))

    gz_sim_launch = PythonLaunchDescriptionSource(
        os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py'))
    ld.add_action(IncludeLaunchDescription(
        gz_sim_launch,
        launch_arguments={
            'gz_args': '-r -s -v2 ' + os.path.join(week_6_share, 'worlds', 'week_6.world'),
            'on_exit_shutdown': 'true'}.items()))
    ld.add_action(IncludeLaunchDescription(
        gz_sim_launch,
        condition=UnlessCondition(LaunchConfiguration('headless')),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_share, 'launch', 'robot_state_publisher.launch.py')),
        # A node parameter there, which would otherwise override the SetParameter above
        launch_arguments={'use_sim_time': 'true'}.items()))

    # The true/* frames for RViz: see week_6/true_frames.py
    with open(os.path.join(turtlebot3_share, 'urdf', f'{model_folder}.urdf')) as f:
        robot_description = f.read()
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='true',
        parameters=[{'robot_description': robot_description, 'frame_prefix': 'true/'}],
        remappings=[('joint_states', '/joint_states')],
        output='screen',
    ))
    ld.add_action(Node(package='week_6', executable='true_frames', output='screen'))

    ld.add_action(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='turtlebot3_bridge',
        parameters=[{'config_file': os.path.join(turtlebot3_share, 'params',
                                                 f'{model_folder}_bridge.yaml')}],
        output='screen',
    ))
    ld.add_action(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_world_bridge',
        parameters=[{'config_file': os.path.join(week_6_share, 'config', 'bridge.yaml')}],
        output='screen',
    ))
    ld.add_action(Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    ))

    ld.add_action(Node(
        condition=IfCondition(LaunchConfiguration('aruco')),
        package='week_6',
        executable='aruco_detector',
        output='screen',
    ))

    ld.add_action(Node(
        condition=IfCondition(LaunchConfiguration('barrel')),
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'barrel',
                   '-file', os.path.join(week_6_share, 'models', 'barrel.sdf'),
                   '-x', BARREL_X,
                   '-y', BARREL_Y],
        output='screen',
    ))
    ld.add_action(Node(
        condition=IfCondition(LaunchConfiguration('barrel')),
        package='week_6',
        executable='visual_sensor',
        output='screen',
    ))

    ld.add_action(Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(week_6_share, 'rviz', 'week_6.rviz')],
        output='screen',
    ))

    return ld
