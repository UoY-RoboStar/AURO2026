"""
What differs between the assessment's tasks, for assessment.launch.py.

task2 is the crane task and uses the world as it is. task1 is the mobile robot task: the crane
stays in the world but is not controlled, and Nav2 can be set up to run with a map or with SLAM.
"""

import os
import shutil
import signal
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from assessment.end_effector_layout import TAG_HALF_HEIGHT
import rclpy
import yaml

TASKS = ('task1', 'task2')
ODOMETRY_SOURCES = ('encoder', 'world')

LIFECYCLE_MANAGER = 'lifecycle_manager_navigation'
MODEL_CARRIER = 'auro::ModelCarrier'

# Started in this order: the map frame has to exist before the costmaps are activated.
NAV2_MAPPING_NODES = {'slam': ['slam_toolbox'], 'map': ['map_server', 'amcl']}
NAV2_KEEPOUT_NODES = ['keepout_filter_mask_server', 'keepout_costmap_filter_info_server']
NAV2_NAVIGATION_NODES = ['planner_server', 'controller_server', 'behavior_server',
                         'velocity_smoother', 'bt_navigator', 'waypoint_follower']

# Heights of the floor, and of the pool's rim
# Links and joints of the crane's rotating beacons, which are not part of how it is operated
BEACON_PREFIX = 'beacon_'
# The crane's joints under ros2_control, in task2, in the order effort_controller takes efforts
CRANE_JOINTS = ('bridge_slide', 'winch_block_slide', 'winch_lift')
CRANE_CONTROL_PLUGIN = 'gz_ros2_control::GazeboSimROS2ControlPlugin'
FLOOR_Z = 0.1
RIM_Z = 0.11

# Keeps obstacle marking within what the lidar can return, as rays past its range come back as inf
OBSTACLE_RANGE_MARGIN = 0.5
MAX_ANGULAR_SPEED = 1.8


def shared_memory_cleanup_command():
    """
    Return the command that frees shared memory left behind by processes that are gone, or None.

    Processes that get killed cannot free their segments, and the container's /dev/shm is small.
    This is specific to Fast DDS: other middlewares have no such command, and nothing to do here.
    It only removes what no running process is using. Nodes starting while it runs can fail to
    open a port, though, so it is for when things have stopped.
    """
    if rclpy.get_rmw_implementation_identifier() != 'rmw_fastrtps_cpp':
        return None
    return ['fastdds', 'shm', 'clean'] if shutil.which('fastdds') else None


def load_simulation_config(path):
    """Return the settings in config/simulation.yaml, checked."""
    with open(path) as f:
        config = yaml.safe_load(f)
    expected = {'lidar_range', 'max_speed', 'odometry_source', 'set_initial_pose',
                'level_pool_rim', 'collect_distance', 'target_end_effector',
                'randomise_end_effectors', 'aruco_tag_height', 'robot_start_pose',
                'crane_effort_limits'}
    if set(config) != expected:
        raise RuntimeError(f'{path} must set exactly {sorted(expected)}, got {sorted(config)}')
    for name in ('lidar_range', 'max_speed', 'collect_distance'):
        if not isinstance(config[name], (int, float)) or config[name] <= 0:
            raise RuntimeError(f'{name} must be a number > 0, got {config[name]!r}')
    if config['lidar_range'] <= OBSTACLE_RANGE_MARGIN:
        raise RuntimeError(f'lidar_range must be > {OBSTACLE_RANGE_MARGIN}')
    if config['odometry_source'] not in ODOMETRY_SOURCES:
        raise RuntimeError(f'odometry_source must be one of {ODOMETRY_SOURCES}, '
                           f"got {config['odometry_source']!r}")
    for name in ('set_initial_pose', 'level_pool_rim', 'randomise_end_effectors'):
        if not isinstance(config[name], bool):
            raise RuntimeError(f'{name} must be true or false, got {config[name]!r}')
    if (not isinstance(config['aruco_tag_height'], (int, float))
            or isinstance(config['aruco_tag_height'], bool)
            or config['aruco_tag_height'] < TAG_HALF_HEIGHT):
        raise RuntimeError(f'aruco_tag_height must be a number >= {TAG_HALF_HEIGHT} (half a tag), '
                           f"got {config['aruco_tag_height']!r}")
    pose = config['robot_start_pose']
    if (not isinstance(pose, dict) or set(pose) != {'x', 'y', 'yaw'}
            or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                       for v in pose.values())):
        raise RuntimeError(f'robot_start_pose must set numbers x, y and yaw, got {pose!r}')
    limits = config['crane_effort_limits']
    if (not isinstance(limits, dict) or set(limits) != set(CRANE_JOINTS)
            or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and v > 0
                       for v in limits.values())):
        raise RuntimeError(f'crane_effort_limits must set a number > 0 for each of '
                           f'{list(CRANE_JOINTS)}, got {limits!r}')
    # Which names there are depends on the world: the launch file checks it against that
    if not isinstance(config['target_end_effector'], str) or not config['target_end_effector']:
        raise RuntimeError('target_end_effector must be the name or type of an end effector, '
                           f"got {config['target_end_effector']!r}")
    return config


def running_gazebo(proc='/proc'):
    """
    Return (pid, command line) of every Gazebo (gz sim) server or GUI running on this machine.

    Found by their command lines: the gz-sim-* executables, or the gz tool running 'gz sim'.
    """
    found = []
    for entry in os.listdir(proc):
        if not entry.isdigit():
            continue
        try:
            with open(os.path.join(proc, entry, 'cmdline'), 'rb') as f:
                argv = [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
        except OSError:
            continue  # gone, or not ours to read
        names = [os.path.basename(a) for a in argv]
        if (any(name.startswith('gz-sim-') for name in names)
                or any(a == 'gz' and b == 'sim' for a, b in zip(names, names[1:]))):
            found.append((int(entry), ' '.join(argv)))
    return sorted(found)


def stop_gazebo_server(world_file, timeout=10.0, proc='/proc'):
    """
    Stop any Gazebo server still running on world_file; return the pids it stopped.

    ros2 launch stops the Gazebo server through the gz tool's wrapper process: it interrupts the
    wrapper, waits 5 s, then terminates it, and a server that takes longer than that to shut down
    is left behind, with no launch, until something kills it. The launch file calls this when it
    exits: world_file is the world it generated, a unique temporary file, so only its own server
    matches. Each is interrupted, then killed if still running after timeout seconds.
    """
    def running(pid):
        try:
            with open(f'{proc}/{pid}/stat') as f:
                return f.read().rsplit(')', 1)[1].split()[0] != 'Z'  # a zombie has stopped
        except (OSError, IndexError):
            return False

    pids = [pid for pid, command in running_gazebo(proc) if world_file in command.split()]
    for signal_number in (signal.SIGINT, signal.SIGKILL):
        for pid in [pid for pid in pids if running(pid)]:  # not one that has gone, and been reused
            try:
                os.kill(pid, signal_number)
            except ProcessLookupError:
                pass
        end = time.monotonic() + timeout
        while time.monotonic() < end and any(running(pid) for pid in pids):
            time.sleep(0.1)
    return pids


def configure_robot_model(tree, lidar_range, max_speed, odometry_source='encoder',
                          start_pose=(0.0, 0.0, 0.0), carry_reach=None):
    """
    Change a TurtleBot3's model.sdf in place, for task1.

    The speed limit is in the drive system, so it holds for any command the robot gets.
    start_pose is (x, y, yaw) in the world: with odometry from the world, the odom frame goes
    there, so that odom starts at the robot, as it does with encoders. The robot also gets the
    system that carries end effectors for assessment/end_effector_manager.py, which can only
    pick up or put down models within carry_reach metres of the robot's origin, if it is given.
    """
    root = tree.getroot()
    carrier = ET.SubElement(root.find('model'), 'plugin',
                            {'filename': 'gz_model_carrier', 'name': MODEL_CARRIER})
    if carry_reach is not None:
        ET.SubElement(carrier, 'max_distance').text = f'{carry_reach:g}'
    for lidar_max in root.findall(".//sensor[@type='gpu_lidar']/ray/range/max"):
        lidar_max.text = f'{lidar_range:g}'
    drive = root.find(".//plugin[@name='gz::sim::systems::DiffDrive']")
    ET.SubElement(drive, 'max_linear_velocity').text = f'{max_speed:g}'
    ET.SubElement(drive, 'min_linear_velocity').text = f'{-max_speed:g}'
    if odometry_source == 'world':
        # Nothing bridges these, which takes the drive system's odometry out of the way
        drive.find('odom_topic').text = 'encoder_odom'
        drive.find('tf_topic').text = 'encoder_tf'
        x, y, yaw = start_pose
        root.find('model').append(ET.fromstring(f"""
            <plugin filename="gz-sim-odometry-publisher-system"
                    name="gz::sim::systems::OdometryPublisher">
              <odom_frame>odom</odom_frame>
              <robot_base_frame>base_footprint</robot_base_frame>
              <odom_topic>odom</odom_topic>
              <tf_topic>/tf</tf_topic>
              <odom_publish_frequency>50</odom_publish_frequency>
              <dimensions>2</dimensions>
              <xyz_global_offset>{x:g} {y:g} 0</xyz_global_offset>
              <rpy_global_offset>0 0 {yaw:g}</rpy_global_offset>
            </plugin>"""))


def nav2_simulation_params(config):
    """Return the Nav2 parameters that follow from the settings of the simulation."""
    speed = config['max_speed']
    lidar_range = config['lidar_range']

    def costmap():
        # A new dict each time: yaml writes a shared one as an alias, which ROS cannot parse
        return {'ros__parameters': {'obstacle_layer': {'scan': {
            'raytrace_max_range': lidar_range,
            'obstacle_max_range': lidar_range - OBSTACLE_RANGE_MARGIN,
        }}}}

    return {
        'slam_toolbox': {'ros__parameters': {'max_laser_range': lidar_range}},
        'amcl': {'ros__parameters': {'laser_max_range': lidar_range,
                                     'set_initial_pose': config['set_initial_pose']}},
        'controller_server': {'ros__parameters': {'FollowPath': {'max_linear_vel': speed}}},
        'velocity_smoother': {'ros__parameters': {
            'max_velocity': [speed, 0.0, MAX_ANGULAR_SPEED],
            'min_velocity': [-speed, 0.0, -MAX_ANGULAR_SPEED],
        }},
        'local_costmap': {'local_costmap': costmap()},
        'global_costmap': {'global_costmap': costmap()},
    }


def nav2_lifecycle_nodes(mode, keepout=False):
    """Return the nodes the lifecycle manager starts, in order, for Nav2 mode 'slam' or 'map'."""
    return (NAV2_MAPPING_NODES[mode] + (NAV2_KEEPOUT_NODES if keepout else [])
            + NAV2_NAVIGATION_NODES)


def resolve_path(value):
    """
    Return the absolute path of a launch argument's file.

    package://<package>/<path> is relative to a package's share directory, so unlike an absolute
    path it still works when a workspace is moved.
    """
    if not value.startswith('package://'):
        return os.path.abspath(os.path.expanduser(value))
    package, _, relative = value[len('package://'):].partition('/')
    try:
        return os.path.join(get_package_share_directory(package), relative)
    except PackageNotFoundError:
        raise RuntimeError(f'{value}: there is no package {package!r}. Is it built and sourced?')


def existing_file(argument, value):
    """Return the absolute path of a launch argument's file, which has to exist."""
    path = resolve_path(value)
    if not os.path.isfile(path):
        raise RuntimeError(f'{argument}: no such file: {value}'
                           + ('' if value == path else f' ({path})'))
    return path


def existing_map(argument, value):
    """Return the absolute path of a map's YAML file, which has to exist, as does its image."""
    path = existing_file(argument, value)
    try:
        with open(path) as f:
            image = yaml.safe_load(f)['image']
    except (yaml.YAMLError, KeyError, TypeError):
        raise RuntimeError(f"{argument}: {path} is not a map's YAML file: it has no image")
    if not os.path.isfile(os.path.join(os.path.dirname(path), image)):
        raise RuntimeError(f'{argument}: {path} refers to an image that does not exist: {image}')
    return path


def parse_nav2(task, value):
    """
    Return (mode, map file) for the value of the nav2 launch argument.

    mode is None (no Nav2), 'slam', or 'map', which comes with the map's absolute path.
    """
    if value == 'false':
        return None, None
    if task != 'task1':
        raise RuntimeError(f'nav2 is only available in task1, not in {task}')
    if value == 'slam':
        return 'slam', None
    return 'map', existing_map('nav2', value)


def parse_nav2_start(nav2_mode, value):
    """
    Return whether the launch starts Nav2 itself, from the nav2_start launch argument.

    Only if there is Nav2: auto, the default, makes no difference without it.
    """
    return bool(nav2_mode) and value == 'auto'


def parse_keepout_mask(nav2_mode, value):
    """Return the absolute path of the keepout_mask launch argument, or '' if it is not set."""
    if not value:
        return ''
    if not nav2_mode:
        raise RuntimeError('keepout_mask needs Nav2: launch with nav2:=slam or nav2:=<map.yaml>')
    return existing_map('keepout_mask', value)


# Names that replacement_level also accepts, for these levels (%)
REPLACEMENT_LEVELS = {'all_full': 100.0, 'half_full': 50.0}


def parse_replacement_level(value):
    """Return the replacement_level launch argument as a level: a percentage, or a name of one."""
    if value in REPLACEMENT_LEVELS:
        return REPLACEMENT_LEVELS[value]
    try:
        return float(value)
    except ValueError:
        raise RuntimeError('replacement_level must be a percentage or one of '
                           f'{list(REPLACEMENT_LEVELS)}, got {value!r}')


def parse_time_limit(value):
    """Return the time_limit launch argument, in seconds of simulation time; 0: no limit."""
    try:
        limit = float(value)
    except ValueError:
        limit = -1.0
    if not limit >= 0:  # also refuses nan
        raise RuntimeError(f'time_limit must be a number of seconds >= 0 (0: no limit), got '
                           f'{value!r}')
    return limit


def parse_nav2_params(value):
    """Return the absolute path of the nav2_params launch argument, or '' if it is not set."""
    return existing_file('nav2_params', value) if value else ''


def configure_task1_world(tree, level_rim=False):
    """
    Change the generated world in place for task1.

    Everything that controls or observes the crane goes. The crane becomes static, since the winch
    would otherwise fall into the pool without its joint controllers. A static crane cannot turn
    its beacons, so they go too. Nothing moves the rods
    either, so they become static as well, which saves physics time.
    """
    world = tree.getroot().find('world')
    crane = world.find("model[@name='crane']")
    crane.find('static').text = 'true'
    for plugin in crane.findall('plugin'):
        crane.remove(plugin)
    remove_beacons(crane)
    for link in crane.findall('link'):
        for sensor in link.findall('sensor'):
            link.remove(sensor)

    gui = world.find('gui')
    gui.remove(gui.find("plugin[@filename='WinchCable']"))

    for model in world.findall('model'):
        if model.get('name').startswith(('fuel_rod_', 'control_rod_')):
            static = model.find('static')
            if static is None:
                static = ET.SubElement(model, 'static')
            static.text = 'true'

    if level_rim:
        level_pool_rim(world)


def set_crane_effort_limits(world, limits):
    """
    Set the effort limits of the crane's joints, in a <world> element, from {joint: newtons}.

    Gazebo's physics holds each joint's force to its limit, and crane_control_description() gives
    ros2_control the same limits, which it holds the efforts commanded to.
    """
    crane = world.find("model[@name='crane']")
    for name, limit in limits.items():
        crane.find(f"joint[@name='{name}']/axis/limit/effort").text = f'{limit:g}'


def crane_control_description(world):
    """
    Return the URDF that the crane's ros2_control needs, from a <world> element.

    It has the joints of CRANE_JOINTS, with their limits, the links and joints between them and
    the world (which becomes the link crane_base), and a <ros2_control> system that commands them
    by effort, through gz_ros2_control. The links have no geometry: RViz draws the crane from its
    SDF, published by robot_state_publisher. ros2_control parses this as a URDF, which needs a
    tree of links, but reads only the names and limits of the joints.
    """
    crane = world.find("model[@name='crane']")
    by_child = {joint.find('child').text.strip(): joint for joint in crane.findall('joint')}
    joints, pending = [], [crane.find(f"joint[@name='{name}']") for name in CRANE_JOINTS]
    while pending:
        joint = pending.pop()
        if joint in joints:
            continue
        joints.append(joint)
        parent = joint.find('parent').text.strip()
        if parent != 'world':
            pending.append(by_child[parent])

    def link(name):
        return 'crane_base' if name == 'world' else name

    robot = ET.Element('robot', name='crane')
    links = {'crane_base'} | {link(j.find(end).text.strip())
                              for j in joints for end in ('parent', 'child')}
    for name in sorted(links):
        ET.SubElement(robot, 'link', name=name)
    for joint in joints:
        element = ET.SubElement(robot, 'joint', name=joint.get('name'), type=joint.get('type'))
        ET.SubElement(element, 'parent', link=link(joint.find('parent').text.strip()))
        ET.SubElement(element, 'child', link=link(joint.find('child').text.strip()))
        axis = joint.find('axis')
        if axis is not None:
            ET.SubElement(element, 'axis', xyz=axis.find('xyz').text.strip())
            limit = axis.find('limit')
            ET.SubElement(element, 'limit', **{
                key: limit.find(key).text.strip()
                for key in ('lower', 'upper', 'effort', 'velocity')})
    control = ET.SubElement(robot, 'ros2_control', name='crane', type='system')
    ET.SubElement(ET.SubElement(control, 'hardware'), 'plugin').text = \
        'gz_ros2_control/GazeboSimSystem'
    for name in CRANE_JOINTS:
        effort = crane.find(f"joint[@name='{name}']/axis/limit/effort").text.strip()
        element = ET.SubElement(control, 'joint', name=name)
        command = ET.SubElement(element, 'command_interface', name='effort')
        ET.SubElement(command, 'param', name='min').text = f'-{effort}'
        ET.SubElement(command, 'param', name='max').text = effort
        for interface in ('position', 'velocity', 'effort'):
            ET.SubElement(element, 'state_interface', name=interface)
    return ET.tostring(robot, encoding='unicode')


def remove_beacons(crane):
    """Take the crane's rotating beacons off, from a <model> element, in place."""
    for element in crane.findall('link') + crane.findall('joint'):
        if element.get('name').startswith(BEACON_PREFIX):
            crane.remove(element)


def level_pool_rim(world):
    """Lower the top of the pool's collisions to the floor. What the rim looks like stays."""
    step = RIM_Z - FLOOR_Z
    for collision in world.find("model[@name='pool']").iter('collision'):
        pose = [float(value) for value in collision.find('pose').text.split()]
        size_element = collision.find('geometry/box/size')
        size = [float(value) for value in size_element.text.split()]
        if abs(pose[2] + size[2] / 2 - RIM_Z) < 1e-6:
            pose[2] -= step / 2
            size[2] -= step
            collision.find('pose').text = ' '.join(f'{value:g}' for value in pose)
            size_element.text = ' '.join(f'{value:g}' for value in size)
