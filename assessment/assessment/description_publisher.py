"""
Publishes a robot description, latched, on robot_description (remap it as needed).

Parameter description (string): the description, e.g. URDF.

assessment.launch.py runs it for the crane's ros2_control in task2: the controller manager of the
crane's gz_ros2_control plugin needs a URDF with <ros2_control> tags, on a topic, while the crane's
robot_state_publisher has its SDF, which RViz draws the crane from.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


class DescriptionPublisher(Node):

    def __init__(self):
        super().__init__('description_publisher')
        description = self.declare_parameter('description', '').value
        self.publisher = self.create_publisher(String, 'robot_description', LATCHED)
        self.publisher.publish(String(data=description))


def main():
    rclpy.init()
    node = DescriptionPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
