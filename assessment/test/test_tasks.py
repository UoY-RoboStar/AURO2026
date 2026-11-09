import os
import xml.etree.ElementTree as ET

from assessment import tasks
from assessment.fuel_layout import generate_world
from assessment.tasks import (BEACON_PREFIX, configure_robot_model, configure_task1_world, FLOOR_Z,
                              load_simulation_config, MODEL_CARRIER, NAV2_KEEPOUT_NODES,
                              nav2_lifecycle_nodes, NAV2_NAVIGATION_NODES, nav2_simulation_params,
                              parse_keepout_mask, parse_nav2, parse_nav2_params, parse_nav2_start,
                              resolve_path, RIM_Z, running_gazebo, shared_memory_cleanup_command)
import pytest
import yaml

PACKAGE = os.path.join(os.path.dirname(__file__), '..')
WORLD = os.path.join(PACKAGE, 'worlds', 'assessment.world')
CONFIG = os.path.join(PACKAGE, 'config', 'simulation.yaml')
ROBOT = """<sdf version="1.8"><model name="robot"><link name="base_scan">
  <sensor name="lidar" type="gpu_lidar"><ray><range><min>0.12</min><max>3.5</max></range></ray>
  </sensor></link>
  <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <odom_topic>odom</odom_topic><tf_topic>/tf</tf_topic>
  </plugin>
</model></sdf>"""


def task1_world(**kw):
    tree, _ = generate_world(WORLD, 1, reactor_fill_percentage=80, replacement_percentage=20,
                             initial_mean_level=50)
    configure_task1_world(tree, **kw)
    return tree.getroot().find('world')


def test_crane_has_no_controls_and_is_static():
    crane = task1_world().find("model[@name='crane']")
    assert crane.find('static').text == 'true'
    assert not crane.findall('plugin')
    assert not list(crane.iter('sensor'))


def test_beacons_only_in_task2():
    def beacons(model):
        return [e for e in model.findall('link') + model.findall('joint')
                if e.get('name').startswith(BEACON_PREFIX)]

    world = ET.parse(WORLD).getroot().find('world')
    assert len(beacons(world.find("model[@name='crane']"))) == 4
    assert not beacons(task1_world().find("model[@name='crane']"))


def test_crane_joint_states_leave_out_the_beacons():
    crane = ET.parse(WORLD).getroot().find("world/model[@name='crane']")
    publisher = crane.find("plugin[@name='gz::sim::systems::JointStatePublisher']")
    assert {j.text for j in publisher.findall('joint_name')} == {
        'bridge_slide', 'winch_block_slide', 'winch_lift',
        'rail_south_mount', 'rail_north_mount', 'rail_middle_mount'}


def test_winch_cable_gui_plugin_removed():
    assert task1_world().find("gui/plugin[@filename='WinchCable']") is None


def test_rods_are_static():
    rods = [m for m in task1_world().findall('model')
            if m.get('name').startswith(('fuel_rod_', 'control_rod_'))]
    assert rods
    assert all(m.find('static').text == 'true' for m in rods)


def test_lidar_system_stays():
    plugins = [p.get('filename') for p in task1_world().findall('plugin')]
    assert 'gz-sim-sensors-system' in plugins


def make_map(folder, name='map'):
    (folder / f'{name}.pgm').write_bytes(b'P5')
    path = folder / f'{name}.yaml'
    path.write_text(f'image: {name}.pgm')
    return path


def test_parse_nav2(tmp_path):
    map_file = make_map(tmp_path)
    assert parse_nav2('task1', 'false') == (None, None)
    assert parse_nav2('task2', 'false') == (None, None)
    assert parse_nav2('task1', 'slam') == ('slam', None)
    assert parse_nav2('task1', str(map_file)) == ('map', str(map_file))


@pytest.mark.parametrize('task, value', [('task2', 'slam'), ('task1', '/no/such/map.yaml')])
def test_parse_nav2_rejects(task, value):
    with pytest.raises(RuntimeError):
        parse_nav2(task, value)


def test_mapping_nodes_start_first():
    assert nav2_lifecycle_nodes('slam') == ['slam_toolbox'] + NAV2_NAVIGATION_NODES
    assert nav2_lifecycle_nodes('map') == ['map_server', 'amcl'] + NAV2_NAVIGATION_NODES


def test_keepout_nodes_start_before_navigation():
    assert nav2_lifecycle_nodes('slam', keepout=True) == (
        ['slam_toolbox'] + NAV2_KEEPOUT_NODES + NAV2_NAVIGATION_NODES)


def test_parse_keepout_mask(tmp_path):
    mask = make_map(tmp_path, 'mask')
    assert parse_keepout_mask(None, '') == ''
    assert parse_keepout_mask('slam', '') == ''
    assert parse_keepout_mask('slam', str(mask)) == str(mask)


@pytest.mark.parametrize('mode, value', [(None, 'mask.yaml'), ('slam', '/no/such/mask.yaml')])
def test_parse_keepout_mask_rejects(mode, value):
    with pytest.raises(RuntimeError):
        parse_keepout_mask(mode, value)


@pytest.mark.parametrize('rmw, tool, expected', [
    ('rmw_fastrtps_cpp', '/usr/bin/fastdds', ['fastdds', 'shm', 'clean']),
    ('rmw_fastrtps_cpp', None, None),
    ('rmw_cyclonedds_cpp', '/usr/bin/fastdds', None),
])
def test_shared_memory_cleanup_is_only_for_fast_dds(monkeypatch, rmw, tool, expected):
    monkeypatch.setattr(tasks.rclpy, 'get_rmw_implementation_identifier', lambda: rmw)
    monkeypatch.setattr(tasks.shutil, 'which', lambda _: tool)
    assert shared_memory_cleanup_command() == expected


def collision_tops(world):
    tops = []
    for collision in world.find("model[@name='pool']").iter('collision'):
        z = float(collision.find('pose').text.split()[2])
        tops.append(z + float(collision.find('geometry/box/size').text.split()[2]) / 2)
    return tops


def test_rim_only_levelled_on_request():
    assert max(collision_tops(task1_world())) == pytest.approx(RIM_Z)
    tops = collision_tops(task1_world(level_rim=True))
    assert max(tops) == pytest.approx(FLOOR_Z)
    assert min(tops) < -2  # the floors of the pools stay where they are


def test_level_rim_keeps_the_bottom_of_the_collisions():
    def bottoms(world):
        return sorted(float(c.find('pose').text.split()[2])
                      - float(c.find('geometry/box/size').text.split()[2]) / 2
                      for c in world.find("model[@name='pool']").iter('collision'))
    assert bottoms(task1_world(level_rim=True)) == pytest.approx(bottoms(task1_world()))


def test_robot_model():
    tree = ET.ElementTree(ET.fromstring(ROBOT))
    configure_robot_model(tree, 12.0, 0.8)
    assert tree.getroot().find('.//ray/range/max').text == '12'
    assert tree.getroot().find('.//ray/range/min').text == '0.12'
    assert tree.getroot().find('.//plugin/max_linear_velocity').text == '0.8'
    assert tree.getroot().find('.//plugin/min_linear_velocity').text == '-0.8'
    assert tree.getroot().find(f"./model/plugin[@name='{MODEL_CARRIER}']") is not None
    assert tree.getroot().find(f"./model/plugin[@name='{MODEL_CARRIER}']/max_distance") is None


def test_carrier_reach():
    tree = ET.ElementTree(ET.fromstring(ROBOT))
    configure_robot_model(tree, 12.0, 0.8, carry_reach=1.25)
    carrier = tree.getroot().find(f"./model/plugin[@name='{MODEL_CARRIER}']")
    assert carrier.find('max_distance').text == '1.25'


def test_encoder_odometry_stays_with_the_drive_system():
    tree = ET.ElementTree(ET.fromstring(ROBOT))
    configure_robot_model(tree, 12.0, 0.8, 'encoder')
    assert tree.getroot().find('.//plugin/odom_topic').text == 'odom'
    assert tree.getroot().find(".//plugin[@name='gz::sim::systems::OdometryPublisher']") is None


def test_world_odometry_replaces_the_drive_systems():
    tree = ET.ElementTree(ET.fromstring(ROBOT))
    configure_robot_model(tree, 12.0, 0.8, 'world', (1.0, -3.5, 0.5))
    root = tree.getroot()
    drive = root.find(".//plugin[@name='gz::sim::systems::DiffDrive']")
    odometry = root.find(".//plugin[@name='gz::sim::systems::OdometryPublisher']")
    assert drive.find('odom_topic').text != 'odom'
    assert drive.find('tf_topic').text != '/tf'
    assert odometry.find('odom_topic').text == 'odom'
    assert odometry.find('tf_topic').text == '/tf'
    assert odometry.find('xyz_global_offset').text == '1 -3.5 0'
    assert odometry.find('rpy_global_offset').text == '0 0 0.5'


def test_shipped_config_matches_shipped_nav2_params():
    with open(os.path.join(PACKAGE, 'config', 'nav2_params.yaml')) as f:
        params = yaml.safe_load(f)

    def check(expected, actual):
        for key, value in expected.items():
            if isinstance(value, dict):
                check(value, actual[key])
            else:
                assert actual[key] == value, key
    check(nav2_simulation_params(load_simulation_config(CONFIG)), params)


VALID_CONFIG = {'lidar_range': 12, 'max_speed': 0.8, 'odometry_source': 'world',
                'set_initial_pose': False, 'level_pool_rim': True, 'collect_distance': 0.5,
                'target_end_effector': 'claw', 'randomise_end_effectors': True,
                'aruco_tag_height': 0.7, 'robot_start_pose': {'x': 0.0, 'y': -3.5, 'yaw': 0.0},
                'crane_effort_limits': {'bridge_slide': 1000, 'winch_block_slide': 500,
                                        'winch_lift': 2000.5}}


@pytest.mark.parametrize('change', [
    {'level_pool_rim': None},
    {'other': 1},
    {'lidar_range': 0},
    {'max_speed': 'fast'},
    {'odometry_source': 'gps'},
    {'set_initial_pose': 1},
    {'level_pool_rim': 1},
    {'collect_distance': 0},
    {'collect_distance': None},
    {'target_end_effector': ''},
    {'crane_effort_limits': None},
    {'crane_effort_limits': 1000},
    {'crane_effort_limits': {'bridge_slide': 1000, 'winch_block_slide': 500}},
    {'crane_effort_limits': {'bridge_slide': 1000, 'winch_block_slide': 500, 'winch_lift': 0}},
    {'crane_effort_limits': {'bridge_slide': 1000, 'winch_block_slide': 500, 'winch_lift': True}},
    {'crane_effort_limits': {'bridge_slide': 1000, 'winch_block_slide': 500, 'winch_lift': 'x'}},
    {'target_end_effector': 3},
    {'randomise_end_effectors': 'yes'},
    {'aruco_tag_height': 0.1},
    {'aruco_tag_height': 'high'},
    {'aruco_tag_height': True},
    {'aruco_tag_height': None},
    {'robot_start_pose': None},
    {'robot_start_pose': {'x': 0.0, 'y': -3.5}},
    {'robot_start_pose': {'x': 0.0, 'y': '-3.5', 'yaw': 0.0}},
    {'robot_start_pose': [0.0, -3.5, 0.0]},
])
def test_invalid_config(tmp_path, change):
    config = {**VALID_CONFIG, **change}
    config = {key: value for key, value in config.items() if value is not None}
    path = tmp_path / 'simulation.yaml'
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(RuntimeError):
        load_simulation_config(str(path))


def test_nav2_simulation_params_have_no_yaml_aliases():
    """ROS aborts on parameter files with aliases, which Python's yaml reads without a problem."""
    text = yaml.safe_dump(nav2_simulation_params(load_simulation_config(CONFIG)))
    assert '&id' not in text
    assert '*id' not in text


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_costmaps_mark_what_the_lidar_sees(costmap):
    """An obstacle layer drops everything above max_obstacle_height, which defaults to 0."""
    with open(os.path.join(PACKAGE, 'config', 'nav2_params.yaml')) as f:
        params = yaml.safe_load(f)[costmap][costmap]['ros__parameters']
    assert params['obstacle_layer']['scan']['max_obstacle_height'] > 0.3  # the lidar is at 0.24 m


def test_valid_config(tmp_path):
    path = tmp_path / 'simulation.yaml'
    path.write_text(yaml.safe_dump(VALID_CONFIG))
    assert load_simulation_config(str(path)) == VALID_CONFIG


def test_nav2_start():
    assert parse_nav2_start('slam', 'auto')
    assert not parse_nav2_start('slam', 'manual')
    assert not parse_nav2_start(None, 'manual')
    # auto is the default, so it means nothing without Nav2
    assert not parse_nav2_start(None, 'auto')


def test_package_paths(monkeypatch, tmp_path):
    monkeypatch.setattr(tasks, 'get_package_share_directory', lambda name: f'{tmp_path}/{name}')
    assert resolve_path('package://solution/maps/a.yaml') == f'{tmp_path}/solution/maps/a.yaml'
    (tmp_path / 'solution').mkdir()
    map_file = make_map(tmp_path / 'solution')
    assert parse_nav2('task1', 'package://solution/map.yaml') == ('map', str(map_file))


def test_unknown_package():
    with pytest.raises(RuntimeError, match='no package'):
        resolve_path('package://no_such_package_here/map.yaml')


def test_map_without_its_image(tmp_path):
    map_file = make_map(tmp_path)
    (tmp_path / 'map.pgm').unlink()
    with pytest.raises(RuntimeError, match='image'):
        parse_nav2('task1', str(map_file))


def test_map_file_that_is_not_a_map(tmp_path):
    path = tmp_path / 'params.yaml'
    path.write_text('amcl: {}')
    with pytest.raises(RuntimeError, match='image'):
        parse_nav2('task1', str(path))


def test_nav2_params(tmp_path):
    path = tmp_path / 'params.yaml'
    path.write_text('amcl: {}')
    assert parse_nav2_params('') == ''
    assert parse_nav2_params(str(path)) == str(path)
    with pytest.raises(RuntimeError, match='nav2_params'):
        parse_nav2_params(str(tmp_path / 'missing.yaml'))


def test_running_gazebo(tmp_path):
    def process(pid, *argv):
        (tmp_path / str(pid)).mkdir()
        (tmp_path / str(pid) / 'cmdline').write_bytes(b'\0'.join(a.encode() for a in argv) + b'\0')
    process(10, '/opt/ros/x/libexec/gz/sim10/gz-sim-main', '-r', '-s', 'world.sdf')
    process(11, 'ruby', '/opt/ros/x/bin/gz', 'sim', '-g')
    process(12, '/usr/bin/python3', '/opt/ros/x/bin/ros2', 'launch', 'assessment')
    process(13, 'vim', 'gz_sim_notes.txt')
    (tmp_path / 'self').mkdir()
    assert [pid for pid, _ in running_gazebo(str(tmp_path))] == [10, 11]


def test_crane_is_under_ros2_control():
    crane = ET.parse(WORLD).getroot().find("world/model[@name='crane']")
    assert crane.find(f"plugin[@name='{tasks.CRANE_CONTROL_PLUGIN}']") is not None
    assert not crane.findall("plugin[@name='gz::sim::systems::JointPositionController']")
    controllers = yaml.safe_load(open(os.path.join(PACKAGE, 'config', 'crane_controllers.yaml')))
    effort = controllers['/crane/effort_controller']['ros__parameters']
    assert tuple(effort['joints']) == tasks.CRANE_JOINTS
    assert effort['interface_name'] == 'effort'


def test_crane_control_description():
    world = ET.parse(WORLD).getroot().find('world')
    robot = ET.fromstring(tasks.crane_control_description(world))
    assert robot.tag == 'robot'
    links = {link.get('name') for link in robot.findall('link')}
    joints = {joint.get('name'): joint for joint in robot.findall('joint')}
    # A tree: every link but the root is the child of exactly one joint
    children = [joint.find('child').get('link') for joint in joints.values()]
    assert sorted(children) == sorted(links - {'crane_base'})
    assert {joint.find('parent').get('link') for joint in joints.values()} <= links
    for name in tasks.CRANE_JOINTS:
        assert joints[name].get('type') == 'prismatic'
        assert joints[name].find('limit').get('effort')
    control = robot.find('ros2_control')
    assert control.find('hardware/plugin').text == 'gz_ros2_control/GazeboSimSystem'
    assert [j.get('name') for j in control.findall('joint')] == list(tasks.CRANE_JOINTS)
    for joint in control.findall('joint'):
        assert [c.get('name') for c in joint.findall('command_interface')] == ['effort']


def test_winch_brake_on_at_start():
    crane = ET.parse(WORLD).getroot().find("world/model[@name='crane']")
    brake = crane.find("plugin[@name='auro::JointBrake']")
    assert brake.find('joint_name').text == 'winch_lift'
    assert brake.find('engaged').text == 'true'
    bridged = {entry.get('service_name') for entry in yaml.safe_load(
        open(os.path.join(PACKAGE, 'config', 'bridge.yaml')))}
    assert {brake.find('engage_service').text, brake.find('release_service').text} <= bridged


def test_crane_effort_limits():
    world = ET.parse(WORLD).getroot().find('world')
    limits = {'bridge_slide': 1000, 'winch_block_slide': 500, 'winch_lift': 2000.5}
    tasks.set_crane_effort_limits(world, limits)
    crane = world.find("model[@name='crane']")
    robot = ET.fromstring(tasks.crane_control_description(world))
    for name, limit in limits.items():
        assert float(crane.find(f"joint[@name='{name}']/axis/limit/effort").text) == limit
        assert float(robot.find(f"joint[@name='{name}']/limit").get('effort')) == limit
        command = robot.find(f"ros2_control/joint[@name='{name}']/command_interface")
        assert float(command.find("param[@name='max']").text) == limit
        assert float(command.find("param[@name='min']").text) == -limit


def test_replacement_level():
    assert tasks.parse_replacement_level('all_full') == 100.0
    assert tasks.parse_replacement_level('half_full') == 50.0
    assert tasks.parse_replacement_level('75') == 75.0
    with pytest.raises(RuntimeError):
        tasks.parse_replacement_level('full')


def test_time_limit():
    assert tasks.parse_time_limit('900') == 900.0
    assert tasks.parse_time_limit('0') == 0.0
    for value in ('-1', 'soon', 'nan', ''):
        with pytest.raises(RuntimeError):
            tasks.parse_time_limit(value)


def test_stop_gazebo_server(tmp_path):
    """Stops a server on the given world, and only that one."""
    import subprocess
    import sys
    fake = tmp_path / 'gz-sim-main'  # running_gazebo() knows it by that name
    fake.write_text('import time\ntime.sleep(60)\n')
    ours, other = str(tmp_path / 'assessment_ours.world'), str(tmp_path / 'other.world')
    servers = {world: subprocess.Popen([sys.executable, str(fake), '-r', '-s', world])
               for world in (ours, other)}
    try:
        assert tasks.stop_gazebo_server(ours, timeout=5.0) == [servers[ours].pid]
        assert servers[ours].wait(timeout=5) is not None
        assert servers[other].poll() is None
    finally:
        for server in servers.values():
            server.kill()
            server.wait()
