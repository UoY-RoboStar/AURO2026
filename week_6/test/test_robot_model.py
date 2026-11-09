import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
from week_6.robot_model import configure_robot_model


def load_model():
    return ET.parse(os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models',
                                 'turtlebot3_waffle_pi', 'model.sdf'))


def wheel(root, side):
    """Return a wheel's collision radius and the height of its axle and of its lowest point."""
    link = root.find(f".//link[@name='wheel_{side}_link']")
    radius = float(link.find('collision/geometry/cylinder/radius').text)
    axle = float(root.find(f".//joint[@name='wheel_{side}_joint']/pose").text.split()[2])
    return radius, axle, axle - radius


def test_no_bias_leaves_the_wheels_alone():
    tree = load_model()
    configure_robot_model(tree)
    for side in ('left', 'right'):
        assert wheel(tree.getroot(), side) == pytest.approx(wheel(load_model().getroot(), side))


def test_bias_only_changes_its_wheel_and_keeps_it_on_the_floor():
    tree = load_model()
    configure_robot_model(tree, encoder_bias_left=1.1)
    nominal, _, bottom = wheel(load_model().getroot(), 'left')
    radius, _, new_bottom = wheel(tree.getroot(), 'left')
    assert radius == pytest.approx(nominal / 1.1)
    assert new_bottom == pytest.approx(bottom)
    assert wheel(tree.getroot(), 'right') == pytest.approx(wheel(load_model().getroot(), 'right'))


def test_both_sources_of_odometry_are_published():
    tree = load_model()
    configure_robot_model(tree)
    root = tree.getroot()
    assert root.find(".//plugin[@name='gz::sim::systems::DiffDrive']/odom_topic").text == 'odom'
    world = root.find(".//plugin[@name='gz::sim::systems::OdometryPublisher']")
    assert world.find('odom_topic').text == 'odom_world'
    assert world.find('odom_frame').text == 'world'


@pytest.mark.parametrize('bias', [0.0, -1.0])
def test_bias_has_to_be_positive(bias):
    with pytest.raises(RuntimeError):
        configure_robot_model(load_model(), encoder_bias_right=bias)


def slip_wheels(root):
    plugin = root.find(".//plugin[@name='gz::sim::systems::WheelSlip']")
    return {} if plugin is None else {wheel.get('link_name'): wheel for wheel in plugin}


def test_no_slip_adds_no_plugin():
    tree = load_model()
    configure_robot_model(tree)
    assert slip_wheels(tree.getroot()) == {}


def test_slip_only_applies_to_its_wheel_and_uses_the_biased_radius():
    tree = load_model()
    configure_robot_model(tree, encoder_bias_left=1.1, wheel_slip_left=10.0)
    root = tree.getroot()
    wheels = slip_wheels(root)
    assert list(wheels) == ['wheel_left_link']
    left = wheels['wheel_left_link']
    assert left.find('slip_compliance_longitudinal').text == '10'
    assert float(left.find('wheel_radius').text) == pytest.approx(wheel(root, 'left')[0])
    fdir = ".//link[@name='wheel_{}_link']/collision/surface/friction/ode/fdir1"
    assert root.find(fdir.format('left')).text == '0 0 1'
    assert root.find(fdir.format('right')).text == '0 0 0'


def test_slip_cannot_be_negative():
    with pytest.raises(RuntimeError):
        configure_robot_model(load_model(), wheel_slip_right=-1.0)
