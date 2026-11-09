"""
Example of starting Nav2 from a node, once the simulation is ready.

Nav2 is launched idle (nav2:=slam or nav2:=<map.yaml>), with its nodes unconfigured. This node
waits for /simulation_ready and then asks the lifecycle manager to start them. With a map, that
finishes when something gives AMCL an initial pose. nav2_start:=auto runs this from the launch.
"""

from assessment.tasks import LIFECYCLE_MANAGER
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

LOG_PERIOD = 10.0  # how often to report that it is still waiting, in seconds of wall time


class Nav2Startup(Node):

    def __init__(self):
        super().__init__('nav2_startup')
        self.simulation_ready = False
        # The message is latched: this durability is what gets it to late subscribers
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/simulation_ready', self.on_ready, qos)
        self.client = self.create_client(
            ManageLifecycleNodes, f'/{LIFECYCLE_MANAGER}/manage_nodes')

    def on_ready(self, msg):
        self.simulation_ready = msg.data

    def run(self):
        """Return whether Nav2 was started."""
        self.get_logger().info('Waiting for the simulation')
        while rclpy.ok() and not self.simulation_ready:
            rclpy.spin_once(self, timeout_sec=0.1)

        while rclpy.ok() and not self.client.wait_for_service(timeout_sec=LOG_PERIOD):
            self.get_logger().info('Waiting for the lifecycle manager: was the simulation '
                                   'launched with nav2:=slam or nav2:=<map.yaml>?')

        self.get_logger().info('Starting Nav2')
        request = ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP)
        future = self.client.call_async(request)
        # The reply comes when all of the nodes are active. If one dies it never does, and the
        # launch logs an error saying which.
        while rclpy.ok() and not future.done():
            rclpy.spin_until_future_complete(self, future, timeout_sec=LOG_PERIOD)
            if not future.done():
                self.get_logger().info('Nav2 is still starting. With a map, it needs an initial '
                                       'pose on /initialpose')
        if not future.done() or not future.result().success:
            self.get_logger().error('Nav2 failed to start: see the output of the launch')
            return False
        self.get_logger().info('Nav2 is active')
        return True


def main():
    rclpy.init()
    node = Nav2Startup()
    try:
        started = node.run()
    except KeyboardInterrupt:
        started = False
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if started else 1
