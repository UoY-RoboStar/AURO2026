import math
import os
import xml.etree.ElementTree as ET

from assessment.end_effector_layout import (check_target, end_effector_models, matches_target,
                                            pose_values, remove_end_effectors, set_tag_height,
                                            shuffle_end_effectors, TAG_PREFIX)
import pytest

WORLD = os.path.join(os.path.dirname(__file__), '..', 'worlds', 'assessment.world')


def world(seed=None):
    element = ET.parse(WORLD).getroot().find('world')
    if seed is not None:
        shuffle_end_effectors(element, seed)
    return element


def poses(element):
    """Pose of every end effector and of every tag, by name."""
    names = list(end_effector_models(element))
    names += [TAG_PREFIX + name for name in names]
    return {name: pose_values(element.find(f"model[@name='{name}']")) for name in names}


def relative(element, name):
    """Return an end effector's (x, y, yaw) in its tag's frame."""
    x, y, _, _, _, yaw = pose_values(element.find(f"model[@name='{name}']"))
    tx, ty, _, _, _, tyaw = pose_values(element.find(f"model[@name='{TAG_PREFIX}{name}']"))
    c, s = math.cos(tyaw), math.sin(tyaw)
    return (c * (x - tx) + s * (y - ty), -s * (x - tx) + c * (y - ty),
            math.atan2(math.sin(yaw - tyaw), math.cos(yaw - tyaw)))


def rounded(values, digits=4):
    return tuple(round(v, digits) + 0.0 for v in values)


def test_same_seed_same_layout():
    assert poses(world(3)) == poses(world(3))


def test_different_seeds_different_layouts():
    assert poses(world(3)) != poses(world(4))


def test_tags_swap_places():
    shipped, shuffled = world(), world(5)
    names = list(end_effector_models(shipped))

    def tag_places(element):
        return sorted(rounded(pose_values(element.find(f"model[@name='{TAG_PREFIX}{n}']")))
                      for n in names)
    assert tag_places(shuffled) == tag_places(shipped)
    assert sum(rounded(pose_values(shipped.find(f"model[@name='{TAG_PREFIX}{n}']")))
               != rounded(pose_values(shuffled.find(f"model[@name='{TAG_PREFIX}{n}']")))
               for n in names) > len(names) // 2


def test_end_effectors_stay_with_their_tags():
    shipped, shuffled = world(), world(5)
    for name, model in end_effector_models(shipped).items():
        assert rounded(relative(shuffled, name)) == rounded(relative(shipped, name)), name
        # Height and the upside-down orientation are kept
        assert rounded(pose_values(shuffled.find(f"model[@name='{name}']"))[2:5]) == \
            rounded(pose_values(model)[2:5])


def test_target_is_a_type_or_a_name():
    element = world()
    for target in ('claw', 'claw_1', 'suction_cup'):
        check_target(element, target)
    for target in ('claws', 'claw_3', 'aruco_claw_1', ''):
        with pytest.raises(ValueError):
            check_target(element, target)
    assert matches_target('claw_2', 'claw')
    assert matches_target('claw_2', 'claw_2')
    assert not matches_target('claw_2', 'claw_1')
    assert not matches_target('drill_long_1', 'drill_short')


def test_tag_height():
    element = world(5)
    before = poses(element)
    set_tag_height(element, 1.2, 0.1)
    after = poses(element)
    for name, pose in after.items():
        if name.startswith(TAG_PREFIX):
            assert round(pose[2], 6) == 1.3, name
            assert pose[:2] + pose[3:] == before[name][:2] + before[name][3:], name
        else:
            assert pose == before[name], name
    with pytest.raises(ValueError):
        set_tag_height(element, 0.1, 0.1)


def test_remove_end_effectors():
    element = world(5)
    names = list(end_effector_models(element))
    others = [m.get('name') for m in element.findall('model')
              if m.get('name') not in names and m.get('name') not in
              [TAG_PREFIX + n for n in names]]
    remove_end_effectors(element)
    assert end_effector_models(element) == {}
    assert not [m for m in element.findall('model') if m.get('name').startswith(TAG_PREFIX)]
    assert [m.get('name') for m in element.findall('model')] == others
