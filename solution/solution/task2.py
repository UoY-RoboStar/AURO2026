"""
Sketch of a solution to task 2: the crane and the reactor.

wait_for_simulation() waits until the simulation monitor says, on /simulation_ready, that the
simulation is running.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class Task2(Node):

    def __init__(self):
        super().__init__('task2')
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


def main():
    rclpy.init()
    node = Task2()
    try:
        node.wait_for_simulation()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
