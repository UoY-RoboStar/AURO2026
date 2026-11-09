"""
Sketch of a solution to task 1: the mobile robot.

wait_for_simulation() waits until the simulation monitor says, on /simulation_ready, that the
simulation is running.
"""

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class Task1(Node):

    def __init__(self):
        super().__init__('task1')
        # slam_toolbox or amcl, set by solution.launch.py from its nav2 argument
        self.localizer = self.declare_parameter('localizer', 'slam_toolbox').value

        # If using nav2
        self.navigator = BasicNavigator('task1_navigator')

        self.simulation_ready = False
        # Latched: the last value is received whenever the subscription is made
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/simulation_ready', self.on_ready, latched)

    def on_ready(self, msg):
        self.simulation_ready = msg.data

    def wait_for_simulation(self):
        """Spin until the simulation is ready."""
        self.get_logger().info('Waiting for the simulation')
        while rclpy.ok() and not self.simulation_ready:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('The simulation is ready')

    def start_nav2(self):
        """Wait until Nav2 is active, and localising the robot."""

        if self.localizer == 'amcl':
            # If using a map.
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            self.navigator.setInitialPose(pose)

        # With AMCL this also republishes the initial pose until AMCL takes it
        self.navigator.waitUntilNav2Active(localizer=self.localizer)
        self.get_logger().info('Nav2 is ready')


def main():
    rclpy.init()
    node = Task1()
    try:
        node.wait_for_simulation()
        # If using nav2.
        # node.start_nav2()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.navigator.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()
