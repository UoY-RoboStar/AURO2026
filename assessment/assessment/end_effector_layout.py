"""
The end effectors in the assessment world, and their ArUco tags.

Each end effector is a model named <type>_<n>, with a type from END_EFFECTOR_TYPES, and has a tag
model named aruco_<type>_<n> that shows which type it is: both of a type have the same tag. The
tags hang at 16 places along the north and south walls, each end effector on the floor in front
of its tag, at an offset from the tag that depends only on its type (the suction cup and the jack
hammer sit a little further out than the others).

shuffle_end_effectors() moves the pairs between those places at random, keeping each end effector
at its offset from its own tag, and set_tag_height() sets how high the tags hang. The launch file
calls both on the generated world, as config/simulation.yaml says (see assessment.launch.py);
end_effector_manager reads the result. remove_end_effectors() takes the pairs out altogether, for
the launch's end_effectors:=false.

Nothing here depends on ROS or Gazebo, so the launch file and the tests can use it.
"""

import math
import random
import re
import xml.etree.ElementTree as ET

END_EFFECTOR_TYPES = ('claw', 'drill_long', 'drill_short', 'hook', 'jack_hammer', 'pallet_fork',
                      'spade', 'suction_cup')
END_EFFECTOR_NAME = re.compile(rf"^({'|'.join(END_EFFECTOR_TYPES)})_\d+$")
TAG_PREFIX = 'aruco_'
TAG_HALF_HEIGHT = 0.18  # m: the tags' meshes are 0.36 m tall, centred on their origins
PLATFORM = 'platform'

ROBOT_RADIUS = 0.25      # m, from the robot's origin: covers its footprint as it turns in place
OFFLOAD_CLEARANCE = 0.05  # m, between that circle and an end effector put down behind it
CARRY_REACH_MARGIN = 0.05  # m, added to the carrier's reach, for rounding


def pose_values(element):
    """Return the six numbers of an SDF <pose> child of element, zeros if there is none."""
    pose = element.find('pose')
    return [float(v) for v in pose.text.split()] if pose is not None else [0.0] * 6


def set_pose(element, values):
    element.find('pose').text = ' '.join(f'{round(v, 6):g}' for v in values)


def rotation(roll, pitch, yaw):
    """Return the rotation matrix of SDF's fixed-axis roll, pitch and yaw, as rows."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def rotate(matrix, vector):
    return tuple(sum(m * v for m, v in zip(row, vector)) for row in matrix)


def quaternion(roll, pitch, yaw):
    """Return (x, y, z, w) of SDF's fixed-axis roll, pitch and yaw."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def end_effector_type(name):
    """Return the type of an end effector, from its name: 'claw' for claw_1."""
    return name.rsplit('_', 1)[0]


def matches_target(name, target):
    """Whether end effector name is the target: a type (either of that type) or a name."""
    return name == target or end_effector_type(name) == target


class EndEffector:
    """
    An end effector's pose, and the footprint of its collision box.

    Its box is upright (rotated about z only) once the model's pose is applied, as all of the
    world's are: their models are upside down (roll pi) and their boxes are not rotated.
    """

    def __init__(self, model):
        self.name = model.get('name')
        self.x, self.y, self.z, self.roll, self.pitch, self.yaw = pose_values(model)
        self.floor_z = self.z  # it starts on the floor, and is put down at the same height
        collision = model.find('link/collision')
        self.centre = tuple(pose_values(collision)[:3])  # in the model's frame
        self.half_size = [float(v) / 2 for v in collision.find('geometry/box/size').text.split()]
        hx, hy = self.half_size[:2]
        cx, cy = self.centre[:2]
        # Rotating the model about z, or flipping it over x, keeps this
        self.radius = max(math.hypot(cx + sx * hx, cy + sy * hy)
                          for sx in (-1, 1) for sy in (-1, 1))

    def rotation(self):
        return rotation(self.roll, self.pitch, self.yaw)

    def centre_offset(self):
        """Return where the box's centre is from the model's origin, in the world frame."""
        return rotate(self.rotation(), self.centre)

    def distance(self, x, y):
        """Return the horizontal distance from (x, y) to the box's footprint (0 if inside it)."""
        matrix = self.rotation()
        ox, oy, _ = rotate(matrix, self.centre)
        dx, dy = x - (self.x + ox), y - (self.y + oy)
        axes = (rotate(matrix, (1.0, 0.0, 0.0)), rotate(matrix, (0.0, 1.0, 0.0)))
        outside = [max(abs(dx * axis[0] + dy * axis[1]) - half, 0.0)
                   for axis, half in zip(axes, self.half_size[:2])]
        return math.hypot(*outside)


def offload_distance(effector):
    """Return how far behind the robot's origin an end effector is put down, to its origin."""
    return ROBOT_RADIUS + OFFLOAD_CLEARANCE + effector.radius


def carry_reach(collect_distance, end_effectors):
    """
    Return the reach the robot's carrier needs.

    That is the farthest from the robot's origin an end effector's origin can be when
    end_effector_manager picks it up or puts it down. It picks up those within collect_distance
    of their footprint, whose farthest point from their origin is effector.radius away; and puts
    them down offload_distance() away.
    """
    return max(max(collect_distance + e.radius, offload_distance(e))
               for e in end_effectors) + CARRY_REACH_MARGIN


def end_effector_models(world):
    """Return the end effectors' <model> elements in a <world> element, by name."""
    return {model.get('name'): model for model in world.findall('model')
            if END_EFFECTOR_NAME.match(model.get('name'))}


def find_end_effectors(world_file):
    """Return the end effectors in a world file, by name."""
    world = ET.parse(world_file).getroot().find('world')
    return {name: EndEffector(model) for name, model in end_effector_models(world).items()}


def find_platform(world_file):
    """Return (x, y, yaw, half length, half width) of the platform's top, from a world file."""
    world = ET.parse(world_file).getroot().find('world')
    model = world.find(f"model[@name='{PLATFORM}']")
    x, y, _, _, _, yaw = pose_values(model)
    size = [float(v) for v in model.find('link/visual/geometry/box/size').text.split()]
    return x, y, yaw, size[0] / 2, size[1] / 2


def check_target(world, target):
    """Raise ValueError unless target names a type or an end effector in a <world> element."""
    names = sorted(end_effector_models(world))
    valid = sorted({end_effector_type(name) for name in names}) + names
    if target not in valid:
        raise ValueError(f'target_end_effector must be one of {valid}, got {target!r}')


def set_tag_height(world, height, floor_z):
    """Put the centre of every end effector's tag height metres above the floor, in a <world>."""
    if height < TAG_HALF_HEIGHT:
        raise ValueError(f'tag height must be at least {TAG_HALF_HEIGHT} m, half a tag, '
                         f'got {height}')
    for name in end_effector_models(world):
        tag = world.find(f"model[@name='{TAG_PREFIX}{name}']")
        pose = pose_values(tag)
        pose[2] = floor_z + height
        set_pose(tag, pose)


def remove_end_effectors(world):
    """Remove every end effector, and its tag, from a <world> element."""
    for name, model in end_effector_models(world).items():
        world.remove(model)
        tag = world.find(f"model[@name='{TAG_PREFIX}{name}']")
        if tag is not None:
            world.remove(tag)


def shuffle_end_effectors(world, seed):
    """
    Move the end effectors, with their tags, between their places at random, in a <world>.

    The tags take each other's poses; each end effector keeps its pose relative to its own tag.
    The same seed gives the same layout. The random numbers are drawn from their own generator,
    so this changes nothing else that seed decides, like the fuel rods.
    """
    pairs = []
    for name, model in sorted(end_effector_models(world).items()):
        tag = world.find(f"model[@name='{TAG_PREFIX}{name}']")
        if tag is None:
            raise ValueError(f'end effector {name} has no tag model {TAG_PREFIX}{name}')
        tx, ty, _, _, _, tyaw = pose_values(tag)
        x, y, z, roll, pitch, yaw = pose_values(model)
        c, s = math.cos(tyaw), math.sin(tyaw)
        # In the tag's frame, which only ever turns about z
        offset = (c * (x - tx) + s * (y - ty), -s * (x - tx) + c * (y - ty), z, roll, pitch,
                  wrap(yaw - tyaw))
        pairs.append((model, tag, offset))

    places = [pose_values(tag) for _, tag, _ in pairs]
    random.Random(f'end_effectors:{seed}').shuffle(places)
    for (model, tag, (dx, dy, z, roll, pitch, dyaw)), place in zip(pairs, places):
        set_pose(tag, place)
        tx, ty, _, _, _, tyaw = place
        c, s = math.cos(tyaw), math.sin(tyaw)
        set_pose(model, (tx + c * dx - s * dy, ty + s * dx + c * dy, z, roll, pitch,
                         wrap(tyaw + dyaw)))
