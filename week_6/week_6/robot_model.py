"""Changes to a TurtleBot3's model.sdf, made at launch."""

import xml.etree.ElementTree as ET

ODOMETRY_RATE = 25  # Hz, for /odom and /odom_world alike: a whole number of 10 ms physics steps
WHEEL_NORMAL_FORCE = 8  # N, an estimate: WheelSlip divides compliance by it, setting their scale


def add_world_odometry(root):
    """
    Publish the robot's true pose in the world on the Gazebo topic odom_world.

    The DiffDrive system keeps /odom and the odom -> base_footprint transform. This one has no TF
    (it goes to a topic nothing bridges): base_footprint can only have one parent.
    """
    drive = root.find(".//plugin[@name='gz::sim::systems::DiffDrive']")
    ET.SubElement(drive, 'odom_publish_frequency').text = str(ODOMETRY_RATE)
    root.find('model').append(ET.fromstring(f"""
        <plugin filename="gz-sim-odometry-publisher-system"
                name="gz::sim::systems::OdometryPublisher">
          <odom_frame>world</odom_frame>
          <robot_base_frame>base_footprint</robot_base_frame>
          <odom_topic>odom_world</odom_topic>
          <tf_topic>odom_world_tf</tf_topic>
          <odom_publish_frequency>{ODOMETRY_RATE}</odom_publish_frequency>
          <dimensions>2</dimensions>
        </plugin>"""))


def raise_pose(pose, dz):
    """Move an SDF <pose> element up by dz."""
    values = pose.text.split()
    values[2] = f'{float(values[2]) + dz:.6g}'
    pose.text = ' '.join(values)


def set_encoder_bias(root, side, bias):
    """
    Make the odometry over-report the travel of the 'left' or 'right' wheel by a factor of bias.

    With a bias of 1.05 the odometry takes the wheel to have travelled 5% further than it did, as
    if it had slipped. Gazebo's DiffDrive system has a single radius for both wheels and no scale
    for either, so the wheel's real radius changes instead, to radius / bias: its effective rolling
    radius. The odometry is then right about the commands and the robot is what goes astray: a
    command to go straight gives a straight /odom and a curved /odom_world.

    The axle moves with the wheel, leaving the bottom of the wheel where it was. The casters are
    1 mm above that, so a wheel that only shrank would lose the floor.
    """
    if bias <= 0:
        raise RuntimeError(f'encoder_bias_{side} must be > 0, got {bias}')
    wheel = root.find(f".//link[@name='wheel_{side}_link']")
    radius = wheel.find('collision/geometry/cylinder/radius')
    nominal = float(radius.text)
    radius.text = f'{nominal / bias:.6g}'
    dz = nominal / bias - nominal
    for pose in wheel.iter('pose'):
        raise_pose(pose, dz)
    raise_pose(root.find(f".//joint[@name='wheel_{side}_joint']/pose"), dz)


def add_wheel_slip(root, compliances):
    """
    Make wheels really slip, through Gazebo's WheelSlip system; compliances: side -> compliance.

    Slip is proportional to the force a wheel transmits, so a compliance is not a percentage, and
    the slip it gives depends on speed and manoeuvre. Wheels with a compliance of 0 are left alone.
    """
    plugin = ET.Element('plugin', filename='gz-sim-wheel-slip-system',
                        name='gz::sim::systems::WheelSlip')
    for side, compliance in compliances.items():
        if compliance < 0:
            raise RuntimeError(f'wheel_slip_{side} must be >= 0, got {compliance}')
        if compliance == 0:
            continue
        link = f'wheel_{side}_link'
        collision = root.find(f".//link[@name='{link}']/collision")
        # WheelSlip takes the first friction direction to be along the axle
        collision.find('surface/friction/ode/fdir1').text = '0 0 1'
        plugin.append(ET.fromstring(f"""
            <wheel link_name="{link}">
              <slip_compliance_lateral>0</slip_compliance_lateral>
              <slip_compliance_longitudinal>{compliance:g}</slip_compliance_longitudinal>
              <wheel_normal_force>{WHEEL_NORMAL_FORCE}</wheel_normal_force>
              <wheel_radius>{collision.find('geometry/cylinder/radius').text}</wheel_radius>
            </wheel>"""))
    if len(plugin):
        root.find('model').append(plugin)


def configure_robot_model(tree, encoder_bias_left=1.0, encoder_bias_right=1.0,
                          wheel_slip_left=0.0, wheel_slip_right=0.0):
    """Change a TurtleBot3's model.sdf in place, for week 6."""
    root = tree.getroot()
    add_world_odometry(root)
    set_encoder_bias(root, 'left', encoder_bias_left)
    set_encoder_bias(root, 'right', encoder_bias_right)
    add_wheel_slip(root, {'left': wheel_slip_left, 'right': wheel_slip_right})
