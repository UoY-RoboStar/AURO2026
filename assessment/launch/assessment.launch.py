#!/usr/bin/env python3
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Pedro Ribeiro

import atexit
import copy
import os
import random
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from assessment.end_effector_layout import (carry_reach, check_target, find_end_effectors,
                                            remove_end_effectors, set_tag_height,
                                            shuffle_end_effectors)
from assessment.fuel_layout import generate_world, serve_manifest
from assessment.tasks import (configure_robot_model, configure_task1_world,
                              crane_control_description, CRANE_CONTROL_PLUGIN, existing_file,
                              FLOOR_Z, load_simulation_config, nav2_lifecycle_nodes,
                              nav2_simulation_params, parse_keepout_mask, parse_nav2,
                              parse_nav2_params, parse_nav2_start, parse_replacement_level,
                              parse_time_limit, remove_beacons, running_gazebo,
                              set_crane_effort_limits, shared_memory_cleanup_command,
                              stop_gazebo_server,
                              TASKS)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, Shutdown)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch.logging
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
import yaml


def crane_description(world_file, model_name='crane'):
    """
    Return the crane model from the world as a standalone SDF string.

    The string is for robot_state_publisher (parsed by sdformat_urdf). Plugins, sensors and the
    beacons are dropped, and the joints to `world` are re-rooted on a new `crane_base` link (the
    crane's model frame).
    """
    root = ET.parse(world_file).getroot()
    model = copy.deepcopy(root.find(f"./world/model[@name='{model_name}']"))
    remove_beacons(model)
    for parent in model.iter():
        for child in list(parent):
            if child.tag in ('plugin', 'sensor'):
                parent.remove(child)
    model.insert(0, ET.fromstring('<link name="crane_base"/>'))
    for parent in model.findall('./joint/parent'):
        if parent.text.strip() == 'world':
            parent.text = 'crane_base'
    return '<sdf version="%s">%s</sdf>' % (root.get('version'),
                                           ET.tostring(model, encoding='unicode'))


def temporary_file(suffix):
    """Return the path of a new file, removed when the launch exits."""
    fd, path = tempfile.mkstemp(prefix='assessment_', suffix=suffix)
    os.close(fd)
    atexit.register(os.remove, path)
    return path


def prepare_world(world_file, args, task, config, end_effectors=True, real_time=True):
    """
    Write the world Gazebo loads to a temporary file; serve the fuel manifest to reactor_monitor.

    Returns (world path, manifest path). The world file is removed when the launch exits. The
    manifest is never in a file: its path is a named pipe that can be read once (see
    assessment/fuel_layout.py). The world gets the random fuel rod layout (see
    assessment/fuel_layout.py) and, if config (simulation.yaml) says so, the end effectors and
    their tags shuffled (see assessment/end_effector_layout.py); in task1, if end_effectors is
    false, it has neither the end effectors nor their tags instead. If real_time is false, its
    physics is not held to real time: it runs as fast as it can. In task2 it also gets the winch
    camera's <update_rate> and pixel noise, the winch attacher's failure probability and seed, the
    path of the crane's ros2_control parameters (config/crane_controllers.yaml), the effort limits
    of its joints (crane_effort_limits in config),
    and loses the winch cable GUI plugin if winch_cable is not true. In task1 it loses all control
    of the crane (see assessment/tasks.py), and reactor_monitor does not run, so there is no
    manifest and the manifest path is None.
    All URIs in the world are package:// or https://, so the copy loads from anywhere.
    """
    def number(name, cast=float):
        try:
            return cast(args[name])
        except ValueError:
            raise RuntimeError(f'{name} must be a number, got {args[name]!r}')

    rate = number('winch_camera_rate')
    if rate <= 0:
        raise RuntimeError(f'winch_camera_rate must be > 0, got {rate}')
    noise = number('winch_camera_noise')
    if noise < 0:
        raise RuntimeError(f'winch_camera_noise must be >= 0, got {noise}')
    failure_probability = number('winch_failure_probability')
    if not 0.0 <= failure_probability <= 1.0:
        raise RuntimeError(
            f'winch_failure_probability must be between 0 and 1, got {failure_probability}')
    seed = (number('random_seed', int) if args['random_seed']
            else random.SystemRandom().randrange(2**31))

    tree, manifest = generate_world(
        world_file, seed,
        reactor_fill_percentage=number('reactor_fill_percentage'),
        replacement_percentage=number('replacement_percentage'),
        initial_mean_level=number('initial_mean_level'),
        initial_level_distribution=args['initial_level_distribution'],
        initial_level_spread=number('initial_level_spread'),
        replacement_level=parse_replacement_level(args['replacement_level']))
    world = tree.getroot().find('world')
    try:
        check_target(world, config['target_end_effector'])
    except ValueError as error:
        raise RuntimeError(f'{error} (config/simulation.yaml)')
    if config['randomise_end_effectors']:
        shuffle_end_effectors(world, seed)
    set_tag_height(world, config['aruco_tag_height'], FLOOR_Z)
    if task == 'task1' and not end_effectors:
        remove_end_effectors(world)
    if not real_time:
        # Gazebo does not throttle the simulation when the factor is 0
        world.find('physics/real_time_factor').text = '0'
    if task == 'task1':
        configure_task1_world(tree, config['level_pool_rim'])
    else:
        set_crane_effort_limits(world, config['crane_effort_limits'])
        control = tree.getroot().find(f".//plugin[@name='{CRANE_CONTROL_PLUGIN}']")
        control.find('parameters').text = os.path.join(
            get_package_share_directory('assessment'), 'config', 'crane_controllers.yaml')
        attacher = tree.getroot().find(".//plugin[@name='auro::LinkAttacher']")
        ET.SubElement(attacher, 'failure_probability').text = f'{failure_probability:g}'
        ET.SubElement(attacher, 'random_seed').text = str(seed % 2**31)
        camera = tree.getroot().find(".//sensor[@name='winch_camera']")
        camera.find('update_rate').text = f'{rate:g}'
        if noise > 0:
            # Gaussian pixel noise; the only noise type Gazebo's camera sensor supports
            camera.find('camera').append(ET.fromstring(
                f'<noise><type>gaussian</type><mean>0</mean><stddev>{noise:g}</stddev></noise>'))
        tree.getroot().find(".//sensor[@name='winch_camera']/update_rate").text = f'{rate:g}'
        if args['winch_cable'] != 'true':
            gui = tree.getroot().find('.//gui')
            gui.remove(gui.find("plugin[@filename='WinchCable']"))

    world_path = temporary_file('.world')
    tree.write(world_path, encoding='utf-8', xml_declaration=True)
    # A named pipe that reactor_monitor reads it from, once: it is never in a file
    manifest_path = serve_manifest(manifest) if task == 'task2' else None
    launch.logging.get_logger('assessment').info(
        f"random_seed {seed}: {len(manifest['fuel_rods'])} fuel rods")
    return world_path, manifest_path


# Launch arguments read when the world is generated: name -> (default, description)
WORLD_ARGS = {
    'winch_camera_rate': ('10', 'Winch camera frame rate in Hz'),
    'winch_camera_noise': ('0', 'Standard deviation of Gaussian pixel noise on the winch camera, '
                                'relative to the full pixel range (e.g. 0.01); 0: no noise'),
    'winch_failure_probability': ('0', 'Chance (0-1) that a valid winch attach or detach fails'),
    'random_seed': ('', 'Seed for the fuel rod layout and levels, for the end effector layout '
                        'and for winch failures'),
    'reactor_fill_percentage': ('80', 'Share of the reactor fuel cells that hold a rod (%)'),
    'replacement_percentage': ('20', 'Share of the storage cells holding a replacement rod (%)'),
    'initial_mean_level': ('50', 'Mean power level of the rods in the reactor (%)'),
    'initial_level_distribution': ('uniform', 'Reactor rod levels: uniform or normal'),
    'initial_level_spread': ('25', 'Half-width (uniform) or standard deviation (normal) of the '
                                   'reactor rod levels, in percentage points'),
    'replacement_level': ('100', 'Power level of the replacement rods: a percentage, all_full '
                                 '(100) or half_full (50)'),
    'winch_cable': ('true', 'Show the cable from the winch block to the winch in the Gazebo GUI'),
}


def generate_launch_description():
    turtlebot3_share = get_package_share_directory('turtlebot3_gazebo')
    launch_file_dir = os.path.join(turtlebot3_share, 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    task_arg = DeclareLaunchArgument(
        'task', default_value='task2', choices=list(TASKS),
        description='task1: mobile robot, with an uncontrolled crane; task2: crane and reactor')
    nav2_arg = DeclareLaunchArgument(
        'nav2', default_value='false',
        description='task1 only. false: no Nav2; slam: Nav2 building a map as the robot moves; '
                    "otherwise a map's YAML file, as a path or package://<package>/<path>. "
                    'nav2_start says what starts it')
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params', default_value='',
        description='Nav2 parameters file; empty: config/nav2_params.yaml')
    nav2_start_arg = DeclareLaunchArgument(
        'nav2_start', default_value='auto', choices=['manual', 'auto'],
        description='task1 with Nav2 only. auto: the launch starts Nav2, as soon as '
                    '/simulation_ready is true; manual: a node starts it (see '
                    'launch/nav2.launch.py). Without Nav2, it makes no difference')
    end_effectors_arg = DeclareLaunchArgument(
        'end_effectors', default_value='true', choices=['true', 'false'],
        description='task1 only. false: no end effectors, nor their ArUco tags, in the world, '
                    'and no end_effector_manager (/pickup, /offload, /collected)')
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false', choices=['true', 'false'],
        description='true: no Gazebo GUI. The power display (power_display), which the GUI '
                    'draws, is not shown either. The server still renders the camera and the '
                    'lidar through the X display, unless headless_rendering is true')
    headless_rendering_arg = DeclareLaunchArgument(
        'headless_rendering', default_value='false', choices=['true', 'false'],
        description='true: the Gazebo server renders the camera and the lidar with EGL, straight '
                    'on the GPU, with no X display needed (e.g. over SSH): the user needs access '
                    'to its render device (/dev/dri/renderD*); false: through the X display')
    limit_real_time_factor_arg = DeclareLaunchArgument(
        'limit_real_time_factor', default_value='true', choices=['true', 'false'],
        description='true: the simulation runs at real time at most; false: as fast as it can')
    gazebo_verbose_arg = DeclareLaunchArgument(
        'gazebo_verbose', default_value='2', choices=['0', '1', '2', '3', '4'],
        description="Verbosity of Gazebo's server and GUI: 0 none, 1 errors, 2 warnings, "
                    '3 information, 4 debug')
    keepout_mask_arg = DeclareLaunchArgument(
        'keepout_mask', default_value='',
        description='task1 with Nav2 only. YAML file of a mask of areas Nav2 must keep out of')

    # The robot's start pose is robot_start_pose in config/simulation.yaml. The floor's top is at
    # z = 0.1 m.
    spawn_z = '0.12'
    # The assessment's robot, whatever the environment says. turtlebot3_gazebo's
    # robot_state_publisher.launch.py reads the variable, so it is set here too.
    turtlebot3_model = 'waffle_pi'
    os.environ['TURTLEBOT3_MODEL'] = turtlebot3_model
    model_folder = f'turtlebot3_{turtlebot3_model}'

    assessment_share = get_package_share_directory('assessment')
    package_parent_dir = os.path.dirname(assessment_share)

    world = os.path.join(
        assessment_share,
        'worlds',
        'assessment.world'
    )

    world_args = [DeclareLaunchArgument(name, default_value=default, description=description)
                  for name, (default, description) in WORLD_ARGS.items()]
    target_power_level_arg = DeclareLaunchArgument(
        'target_power_level', default_value='50',
        description='Target published by reactor_monitor, in % of maximum_power_level')
    maximum_power_level_arg = DeclareLaunchArgument(
        'maximum_power_level', default_value='',
        description='Power level that counts as 100% (sum of rod levels); empty: a full reactor '
                    'with all control rods in, i.e. 100 x the number of reactor fuel cells')
    control_rod_coefficient_arg = DeclareLaunchArgument(
        'control_rod_coefficient', default_value='0.20',
        description='Power increase per control rod removed from the reactor (0.20 = +20%)')
    power_display_arg = DeclareLaunchArgument(
        'power_display', default_value='true', choices=['true', 'false'],
        description='Show current and target power level on the north wall in the Gazebo GUI')
    time_limit_arg = DeclareLaunchArgument(
        'time_limit', default_value='900',
        description='Simulation time, in seconds, after which the simulation ends, counted from '
                    'when /simulation_ready turns true: the whole launch shuts down (900: 15 '
                    'minutes); 0: no limit')
    sim_time_log_arg = DeclareLaunchArgument(
        'sim_time_log', default_value='false', choices=['true', 'false'],
        description="true: also print every node's log messages stamped with simulation time "
                    '(ROS stamps them with wall time); see assessment/sim_time_log.py')
    sim_time_log_file_arg = DeclareLaunchArgument(
        'sim_time_log_file', default_value='',
        description='With sim_time_log: a file to write those messages to, instead of the '
                    'terminal; empty: the terminal')
    # Started first, to catch the messages of everything else as it starts. On wall time: it reads
    # /clock itself
    sim_time_log_cmd = Node(
        condition=IfCondition(LaunchConfiguration('sim_time_log')),
        package='assessment',
        executable='sim_time_log',
        parameters=[{'use_sim_time': False,
                     'file': LaunchConfiguration('sim_time_log_file')}],
        output='screen',
    )

    # The Gazebo server and reactor_monitor share the generated world's fuel manifest
    def gzserver(context):
        # Two simulations at once share Gazebo's topics and services, and their nodes would talk
        # to either. Checked here, before anything is started, rather than when the launch
        # description is made, so that --show-args still works.
        running = running_gazebo()
        if running:
            raise RuntimeError(
                'Gazebo is already running; stop it before launching the assessment (Ctrl-C in '
                'its terminal, or kill the processes):\n' + '\n'.join(
                    f'  pid {pid}: {command[:150]}' for pid, command in running))
        args = {name: LaunchConfiguration(name).perform(context) for name in WORLD_ARGS}
        task = LaunchConfiguration('task').perform(context)
        nav2_mode, map_file = parse_nav2(task, LaunchConfiguration('nav2').perform(context))
        start_nav2 = parse_nav2_start(nav2_mode,
                                      LaunchConfiguration('nav2_start').perform(context))
        keepout_mask = parse_keepout_mask(
            nav2_mode, LaunchConfiguration('keepout_mask').perform(context))
        nav2_params = parse_nav2_params(LaunchConfiguration('nav2_params').perform(context))
        time_limit = parse_time_limit(LaunchConfiguration('time_limit').perform(context))
        config = load_simulation_config(
            os.path.join(assessment_share, 'config', 'simulation.yaml'))
        end_effectors = LaunchConfiguration('end_effectors').perform(context) == 'true'
        headless = LaunchConfiguration('headless').perform(context) == 'true'
        headless_rendering = (
            LaunchConfiguration('headless_rendering').perform(context) == 'true')
        real_time = LaunchConfiguration('limit_real_time_factor').perform(context) == 'true'
        verbose = LaunchConfiguration('gazebo_verbose').perform(context)
        world_file, manifest_file = prepare_world(world, args, task, config, end_effectors,
                                                  real_time)

        # When the launch process ends, whichever launch file included this one: everything it
        # started has gone by then, including what it had to kill, which leaves its share behind
        cleanup_command = shared_memory_cleanup_command()
        if cleanup_command:
            atexit.register(subprocess.run, cleanup_command, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL)
        # Before that (atexit runs the last registered first): the Gazebo server, if ros2 launch
        # left it running (see stop_gazebo_server)
        atexit.register(stop_gazebo_server, world_file)

        # The one node on wall time: it has to report on a clock that is not running
        monitor_params = {'task': task, 'use_sim_time': False}
        if nav2_mode:
            monitor_params['nav2_nodes'] = nav2_lifecycle_nodes(nav2_mode, bool(keepout_mask))
        start = config['robot_start_pose']
        start_pose = (start['x'], start['y'], start['yaw'])
        robot_model = os.path.join(turtlebot3_share, 'models', model_folder, 'model.sdf')
        if task == 'task1':
            tree = ET.parse(robot_model)
            # The carrier's own limit, for when its services are called directly: as far as
            # end_effector_manager could pick an end effector up, or put one down, from. From
            # the shipped world, which has them even when the generated one does not: their
            # sizes are the same wherever they are.
            reach = carry_reach(config['collect_distance'], find_end_effectors(world).values())
            configure_robot_model(tree, config['lidar_range'], config['max_speed'],
                                  config['odometry_source'], start_pose, reach)
            robot_model = temporary_file('.sdf')
            tree.write(robot_model, encoding='utf-8', xml_declaration=True)

        actions = [
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', turtlebot3_model,
                           '-file', robot_model,
                           '-x', f'{start_pose[0]:g}',
                           '-y', f'{start_pose[1]:g}',
                           '-z', spawn_z,
                           '-Y', f'{start_pose[2]:g}'],
                output='screen',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
                ),
                launch_arguments={'gz_args': [f'-r -s -v{verbose} ',
                                              '--headless-rendering ' if headless_rendering
                                              else '',
                                              world_file],
                                  'on_exit_shutdown': 'true'}.items()
            ),
            Node(
                package='assessment',
                executable='simulation_monitor',
                parameters=[monitor_params],
                output='screen',
            ),
        ]
        if nav2_mode:
            simulation_params = ''
            if not nav2_params:
                nav2_params = os.path.join(assessment_share, 'config', 'nav2_params.yaml')
                simulation_params = temporary_file('.yaml')
                with open(simulation_params, 'w') as f:
                    yaml.safe_dump(nav2_simulation_params(config), f)
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(assessment_share, 'launch', 'nav2.launch.py')
                ),
                launch_arguments={'mode': nav2_mode, 'map': map_file or '',
                                  'keepout_mask': keepout_mask,
                                  'params_file': nav2_params,
                                  'simulation_params': simulation_params}.items()
            ))
        if start_nav2:
            actions.append(Node(package='assessment', executable='nav2_startup', output='screen'))
        if time_limit:
            # It exits time_limit s of simulation time after the simulation is ready, which ends
            # everything. On wall time: it reads /clock itself
            actions.append(Node(
                package='assessment', executable='time_limit',
                parameters=[{'time_limit': time_limit, 'use_sim_time': False}],
                on_exit=[Shutdown(reason=f'time limit of {time_limit:g} s of simulation time '
                                         'reached')],
                output='screen'))
        if task != 'task2':
            if not end_effectors:
                return actions
            return actions + [
                Node(
                    package='assessment',
                    executable='end_effector_manager',
                    parameters=[{
                        'world_file': world_file,
                        'robot_name': turtlebot3_model,
                        'target_end_effector': config['target_end_effector'],
                        'collect_distance': float(config['collect_distance']),
                        'max_speed': float(config['max_speed']),
                    }],
                    output='screen',
                ),
            ]
        return actions + [
            # The URDF of the crane's joints that its controller manager (in Gazebo, see
            # assessment.world) needs, and the controller that it runs, once it has the URDF
            Node(
                package='assessment',
                executable='description_publisher',
                name='control_description_publisher',
                namespace='crane',
                parameters=[{'description': crane_control_description(
                    ET.parse(world_file).getroot().find('world'))}],
                remappings=[('robot_description', 'control_description')],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                # In the crane's namespace: it looks for /crane/effort_controller's parameters in
                # the file under its own namespace
                namespace='crane',
                arguments=['effort_controller', '--controller-manager',
                           '/crane/controller_manager', '--controller-manager-timeout', '120',
                           '--param-file', os.path.join(assessment_share, 'config',
                                                        'crane_controllers.yaml')],
                output='screen',
            ),
            Node(
                package='assessment',
                executable='reactor_monitor',
                parameters=[{
                    'rod_manifest': manifest_file,
                    'target_power_level': float(
                        LaunchConfiguration('target_power_level').perform(context)),
                    'maximum_power_level': float(
                        LaunchConfiguration('maximum_power_level').perform(context) or 0),
                    'control_rod_coefficient': float(
                        LaunchConfiguration('control_rod_coefficient').perform(context)),
                }],
                output='screen',
            ),
        ] + ([Node(package='assessment', executable='power_display', output='screen')]
             if LaunchConfiguration('power_display').perform(context) == 'true' and not headless
             else [])

    gzserver_cmd = OpaqueFunction(function=gzserver)

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        condition=UnlessCondition(LaunchConfiguration('headless')),
        launch_arguments={'gz_args': ['-g -v', LaunchConfiguration('gazebo_verbose'), ' '],
                          'on_exit_shutdown': 'true'}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        # A node parameter there, which would otherwise override the SetParameter below
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # As turtlebot3_gazebo's spawn_turtlebot3.launch.py, with the robot spawned in gzserver above:
    # that one's fixed z of 0.01 m is below the floor. The robot's bridge also provides /clock.
    robot_bridges_cmd = GroupAction(actions=[
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': os.path.join(turtlebot3_share, 'params',
                                                     f'{model_folder}_bridge.yaml')}],
            output='screen',
        ),
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['/camera/image_raw'],
            output='screen',
        ),
    ])

    # Everything below up to RViz is for the crane, so only in task2
    task2 = IfCondition(EqualsSubstitution(LaunchConfiguration('task'), 'task2'))

    # ROS <-> Gazebo bridge for the winch attacher (services /crane/winch/{attach,detach})
    # and the winch camera's camera_info
    bridge_cmd = Node(
        condition=task2,
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(assessment_share, 'config', 'bridge.yaml')}],
        output='screen',
    )

    # Winch camera image through image_transport (raw plus compressed/theora/zstd);
    # its camera_info goes through bridge.yaml
    winch_camera_cmd = Node(
        condition=task2,
        package='ros_gz_image',
        executable='image_bridge',
        name='winch_camera_bridge',
        arguments=['/crane/winch_camera/image'],
        output='screen',
    )

    # TF for the crane, from its joint states (bridged from Gazebo, see bridge.yaml): frames
    # crane/crane_base (at the world origin), crane/rail_*, crane/bridge, crane/winch_block and
    # crane/winch. The crane/ prefix keeps them apart from the robot's frames.
    crane_state_publisher_cmd = Node(
        condition=task2,
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='crane',
        parameters=[{
            'robot_description': crane_description(world),
            'frame_prefix': 'crane/',
        }],
        output='screen',
    )
    # The winch camera's optical frame (z out of the lens, i.e. down; x to the image's right, -y
    # world; y down the image, -x world), 0.12 m above the winch link's origin as in
    # assessment.world. Its images carry this frame_id.
    winch_camera_frame_cmd = Node(
        condition=task2,
        package='tf2_ros',
        executable='static_transform_publisher',
        name='winch_camera_frame',
        arguments=['--z', '0.12', '--qx', '0.7071068', '--qy', '-0.7071068', '--qz', '0',
                   '--qw', '0', '--frame-id', 'crane/winch',
                   '--child-frame-id', 'crane/winch_camera_optical'],
        output='screen',
    )

    # Optional RViz view (rviz:=true): the crane and the winch camera image in task2, the robot and
    # Nav2 in task1
    rviz = LaunchConfiguration('rviz')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Start RViz showing the crane and winch camera (task2) or the robot (task1)')
    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file', default_value='',
        description='RViz configuration file, as a path or package://<package>/<path>; empty: '
                    "the assessment's own for the task (rviz/task1.rviz or rviz/crane.rviz)")
    rviz_log_level_arg = DeclareLaunchArgument(
        'rviz_log_level', default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal'],
        description="Verbosity of RViz's log")

    def rviz_node(context):
        task = LaunchConfiguration('task').perform(context)
        config_file = LaunchConfiguration('rviz_config_file').perform(context)
        if config_file:
            config_file = existing_file('rviz_config_file', config_file)
        else:
            config_file = os.path.join(assessment_share, 'rviz',
                                       'task1.rviz' if task == 'task1' else 'crane.rviz')
        return [Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', config_file, '--ros-args', '--log-level',
                       LaunchConfiguration('rviz_log_level').perform(context)],
            output='screen',
        )]

    rviz_cmd = GroupAction(condition=IfCondition(rviz), actions=[
        OpaqueFunction(function=rviz_node)])

    resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = f'{package_parent_dir}:{resource_path}'

    ld = LaunchDescription()

    # Every node here runs on simulation time, however this file is launched: a launch file that
    # includes it with forwarding=False does not pass on its own SetParameter
    ld.add_action(SetParameter(name='use_sim_time', value=True))

    # Declared only so that --show-args reports it as true: otherwise it lists the declaration in
    # turtlebot3's robot_state_publisher.launch.py, with its default of false, which is overridden
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Always true: every node here runs on simulation time, whatever the value '
                    'given')

    # Add the commands to the launch description
    for arg in world_args + [use_sim_time_arg, task_arg, nav2_arg, nav2_start_arg,
                             nav2_params_arg, keepout_mask_arg, end_effectors_arg,
                             target_power_level_arg, maximum_power_level_arg,
                             control_rod_coefficient_arg, power_display_arg, headless_arg,
                             headless_rendering_arg,
                             limit_real_time_factor_arg, gazebo_verbose_arg, rviz_arg,
                             rviz_config_file_arg, rviz_log_level_arg, time_limit_arg,
                             sim_time_log_arg,
                             sim_time_log_file_arg]:
        ld.add_action(arg)

    for action in [sim_time_log_cmd, gzserver_cmd, gzclient_cmd, bridge_cmd, winch_camera_cmd,
                   crane_state_publisher_cmd, winch_camera_frame_cmd, robot_state_publisher_cmd,
                   robot_bridges_cmd, rviz_cmd]:
        ld.add_action(action)

    return ld
