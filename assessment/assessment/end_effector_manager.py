"""
Lets the robot collect the end effectors in the assessment world and offload them anywhere.

Serves
  /pickup (std_srvs/Trigger): pick up the nearest end effector, if the robot is within
      collect_distance of it (from the robot's origin to the edge of the end effector's
      footprint) and is not already holding one.
  /offload (std_srvs/Trigger): put down the end effector the robot holds, on the floor right
      behind it, anywhere in the arena.
The responses do not say which end effector it is: telling them apart is up to the robot.

Publishes
  /collected (std_msgs/Bool, 1 Hz, latched): false until the target end effector is offloaded
      while the robot is inside the area marked by the X (the platform model), then true for good.
      The target (parameter target_end_effector) is a type, e.g. claw, which either end effector
      of that type meets, as their tags are the same; or the name of one, e.g. claw_1.

Tamper detection: Gazebo's services can be called by any process on the machine, so the robot can
be moved by something other than its wheels, e.g. with set_pose. If the robot ever moves faster
than its speed limit (parameter max_speed) allows, an error is logged and /collected can no
longer become true in that run. Resetting the world removes the robot, which ends the run. The
carrier's own services only reach as far as this node does (see assessment/tasks.py), and this
node keeps its own record of what is held, so calling them directly does not count as a delivery
either.

The end effectors are the models in the world named <type>_<n> (see end_effector_layout.py).
Their poses are read from the world file Gazebo loaded, which may have them shuffled. They are
carried by the robot's ModelCarrier system (gz_model_carrier, in package auro_gz_plugins, added to
the robot by assessment/tasks.py), which keeps the held one rigidly above the robot on every
simulation step, clear of its collisions, its camera and its lidar's plane, and puts it down where
this node says. They are static, and Gazebo moves a static model's collision with it, so one that
has been put down is an obstacle where it lies, as before. The robot's pose comes straight from
Gazebo, from the carrier's pose service, asked every POSE_PERIOD and whenever it is needed. It
is not from a subscription to Gazebo's pose topics: in Python, gz-transport requests, which this
node makes to the carrier, do not get their replies while the process has a subscription.
"""

import copy
import math
import signal

from assessment.end_effector_layout import (find_end_effectors, find_platform, matches_target,
                                            offload_distance, quaternion, wrap)
from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.empty_pb2 import Empty
from gz.msgs.pose_pb2 import Pose
from gz.transport import Node as GzNode
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

CARRY_HEIGHT = 0.3       # m, of a held end effector's origin above the robot's
REQUEST_TIMEOUT = 3000   # ms, for the carrier's services, which give up themselves after 2 s
# The robot is taken to have been teleported if it moves more than this, horizontally, between
# two of Gazebo's pose updates: a factor of its speed limit times the time between them, plus a
# margin for physics jitter
TELEPORT_SPEED_FACTOR = 1.5
TELEPORT_MARGIN = 0.05   # m
POSE_PERIOD = 0.1        # s of simulation time, between checks of the robot's pose

COLLECTED_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)


def pose_msg(effector):
    """Return a gz Pose message of an EndEffector's pose, named after it."""
    msg = Pose(name=effector.name)
    msg.position.x, msg.position.y, msg.position.z = effector.x, effector.y, effector.z
    q = quaternion(effector.roll, effector.pitch, effector.yaw)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
    return msg


class EndEffectorManager(Node):

    def __init__(self):
        super().__init__('end_effector_manager')
        p = self.declare_parameter
        world_file = p('world_file', '').value
        self.robot_name = p('robot_name', 'waffle_pi').value
        self.target = p('target_end_effector', '').value
        self.collect_distance = float(p('collect_distance', 0.5).value)
        self.max_speed = float(p('max_speed', 0.26).value)

        self.end_effectors = find_end_effectors(world_file)
        if not any(matches_target(name, self.target) for name in self.end_effectors):
            raise RuntimeError(f'no end effector is target_end_effector {self.target!r}')
        self.platform = find_platform(world_file)

        self.robot = None   # (x, y, z, yaw) of the robot's origin
        self.robot_stamp = None  # sim time of that pose, in seconds
        self.teleported = False
        self.held = None    # the EndEffector the robot holds
        self.held_yaw = 0.0  # its yaw relative to the robot's
        self.collected = False

        self.gz = GzNode()
        carrier = f'/model/{self.robot_name}/model_carrier'
        self.carry_service, self.release_service = f'{carrier}/carry', f'{carrier}/release'
        self.pose_service = f'{carrier}/pose'

        self.collected_pub = self.create_publisher(Bool, '/collected', COLLECTED_QOS)
        self.publish_collected()
        self.create_timer(1.0, self.publish_collected)
        self.create_timer(POSE_PERIOD, self.sample_robot)
        self.create_service(Trigger, '/pickup', self.on_pickup)
        self.create_service(Trigger, '/offload', self.on_offload)
        self.get_logger().info(
            f'{len(self.end_effectors)} end effectors; target {self.target}; '
            f'collect distance {self.collect_distance:g} m')

    def sample_robot(self):
        """
        Return the robot's pose (x, y, z, yaw) now, from its carrier, or None if it is not known.

        Every pose fetched is checked against the one before, whatever it is fetched for, so the
        robot cannot be moved away and back between two of them unnoticed.
        """
        ok, msg = self.gz.request(self.pose_service, Empty(), Empty, Pose, REQUEST_TIMEOUT)
        if not ok:
            if self.robot is None:
                self.get_logger().info("waiting for the robot's pose from Gazebo",
                                       throttle_duration_sec=5.0)
            else:
                self.get_logger().error(
                    "the robot's pose is no longer available: the robot has gone from the world "
                    '(e.g. the world was reset, which removes it, as it was spawned after the '
                    'world loaded)', throttle_duration_sec=5.0)
            return None
        q = msg.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        robot = (msg.position.x, msg.position.y, msg.position.z, yaw)
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        if self.robot is not None:
            self.check_motion(self.robot, self.robot_stamp, robot, stamp)
        self.robot, self.robot_stamp = robot, stamp
        return robot

    def check_motion(self, previous, previous_stamp, robot, stamp):
        """Flag the robot as teleported if it moved faster than its speed limit allows."""
        if previous_stamp == 0.0:
            # From before the first simulation step, while the robot is being spawned: its pose
            # is not set yet, and nothing can have moved it
            return
        distance = math.hypot(robot[0] - previous[0], robot[1] - previous[1])
        # If time went backwards, any movement counts
        dt = max(stamp - previous_stamp, 0.0)
        if distance <= self.max_speed * dt * TELEPORT_SPEED_FACTOR + TELEPORT_MARGIN:
            return
        self.teleported = True
        self.get_logger().error(
            f'Robot moved {distance:.2f} m in {dt:.3f} s of simulation time, faster than its '
            f'{self.max_speed:g} m/s limit allows: something other than its wheels moved it '
            "(e.g. Gazebo's set_pose service). /collected can no longer become "
            'true in this run.')

    def request(self, service, msg):
        """Call one of the robot's carrier services; return whether it succeeded."""
        ok, reply = self.gz.request(service, msg, Pose, Boolean, REQUEST_TIMEOUT)
        if not (ok and reply.data):
            self.get_logger().warning(f'{service} failed (see the Gazebo log)')
            return False
        return True

    def on_pickup(self, request, response):
        robot = self.sample_robot()
        response.success = False
        if self.held is not None:
            response.message = 'Already holding an end effector'
            return response
        if robot is None:
            response.message = 'Robot pose not known yet'
            return response
        distance, effector = min((e.distance(robot[0], robot[1]), e)
                                 for e in self.end_effectors.values())
        if distance > self.collect_distance:
            response.message = 'No end effector within reach'
            return response
        # Its pose in the robot's frame: yaw kept as it is relative to the robot, and the centre of
        # its box over the robot's origin
        held_yaw = wrap(effector.yaw - robot[3])
        relative = copy.copy(effector)
        relative.yaw = held_yaw
        ox, oy, _ = relative.centre_offset()
        relative.x, relative.y, relative.z = -ox, -oy, CARRY_HEIGHT
        if not self.request(self.carry_service, pose_msg(relative)):
            response.message = 'Could not pick the end effector up, try again'
            return response
        self.held = effector
        self.held_yaw = held_yaw
        self.get_logger().info(f'Robot collected {effector.name}')
        response.success = True
        response.message = 'Collected an end effector'
        return response

    def in_platform(self, x, y):
        px, py, yaw, half_length, half_width = self.platform
        dx, dy = x - px, y - py
        c, s = math.cos(yaw), math.sin(yaw)
        return abs(dx * c + dy * s) <= half_length and abs(-dx * s + dy * c) <= half_width

    def on_offload(self, request, response):
        robot = self.sample_robot()
        response.success = False
        if self.held is None:
            response.message = 'Not holding an end effector'
            return response
        if robot is None:
            response.message = 'Robot pose not known yet'
            return response
        x, y, _, yaw = robot
        effector = self.held
        # On the floor (the height it started at) behind the robot, clear of it as it turns
        back = offload_distance(effector)
        effector.x, effector.y = x - back * math.cos(yaw), y - back * math.sin(yaw)
        effector.z = effector.floor_z
        effector.yaw = wrap(yaw + self.held_yaw)
        if not self.request(self.release_service, pose_msg(effector)):
            response.message = 'Could not put the end effector down, try again'
            return response
        self.held = None
        on_x = self.in_platform(x, y)
        self.get_logger().info(f'Robot offloaded {effector.name} '
                               f"{'on' if on_x else 'off'} the X")
        if on_x and matches_target(effector.name, self.target) and not self.collected:
            if self.teleported:
                self.get_logger().error('Target end effector delivered, but /collected stays '
                                        'false: the robot was teleported earlier in this run')
            else:
                self.collected = True
                self.get_logger().info('Target end effector delivered')
                self.publish_collected()
        response.success = True
        response.message = 'Offloaded the end effector'
        return response

    def publish_collected(self):
        self.collected_pub.publish(Bool(data=self.collected))


def main():
    rclpy.init()
    node = EndEffectorManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C reaches the node twice (from the terminal and forwarded by ros2 launch); don't let
        # the second one interrupt the cleanup.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()
