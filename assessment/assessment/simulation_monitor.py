"""
Tell other nodes when the simulation is ready to use.

/simulation_ready (std_msgs/Bool) is false until everything the task needs is up, and then true.
It is latched: subscribe with transient local durability to get the current value at any time.
Ready means that the simulation clock is advancing, the robot's sensors are publishing and its TF
tree is connected. In task2 the crane and reactor_monitor are publishing too, and the crane's
effort controller is taking commands (on CRANE_COMMANDS). If Nav2 is enabled,
all its nodes can be reached by the lifecycle manager, and are waiting to be started.

This node runs on wall time, as it has to work while /clock is not being published: the launch
file sets use_sim_time to false, whatever is including it.
"""

from assessment.tasks import LIFECYCLE_MANAGER
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, Float64
from tf2_ros import Buffer, TransformListener

CHECK_PERIOD = 0.5
LOG_PERIOD = 5.0  # how often to log what is still missing, in seconds

CRANE_COMMANDS = '/crane/effort_controller/commands'

READY_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)


class SimulationMonitor(Node):

    def __init__(self):
        super().__init__('simulation_monitor')
        task = self.declare_parameter('task', 'task2').value
        # rclpy can't infer a type from an empty list, so no Nav2 is [''] here
        nav2_nodes = [name for name in self.declare_parameter('nav2_nodes', ['']).value if name]

        self.ready_pub = self.create_publisher(Bool, '/simulation_ready', READY_QOS)
        self.ready_pub.publish(Bool(data=False))

        self.first_clock = None
        self.clock_advancing = False
        self.received = set()
        topics = [('/scan', LaserScan), ('/odom', Odometry)]
        if task == 'task2':
            topics += [('/crane/joint_states', JointState), ('/current_power_level', Float64)]
        self.expected = [topic for topic, _ in topics]
        self.crane_control = task == 'task2'
        self.subscriptions_held = [
            self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)]
        for topic, msg_type in topics:
            self.subscriptions_held.append(self.create_subscription(
                msg_type, topic, lambda _, topic=topic: self.received.add(topic),
                qos_profile_sensor_data))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav2_clients = {}
        if nav2_nodes:
            self.nav2_clients[f'/{LIFECYCLE_MANAGER}/manage_nodes'] = self.create_client(
                ManageLifecycleNodes, f'/{LIFECYCLE_MANAGER}/manage_nodes')
        for name in nav2_nodes:
            self.nav2_clients[f'/{name}/get_state'] = self.create_client(
                GetState, f'/{name}/get_state')

        self.checks = 0
        self.timer = self.create_timer(CHECK_PERIOD, self.check)

    def on_clock(self, msg):
        now = Time.from_msg(msg.clock)
        if self.first_clock is None:
            self.first_clock = now
        self.clock_advancing = now > self.first_clock

    def missing(self):
        """Return what the simulation is still waiting for."""
        missing = [] if self.clock_advancing else ['/clock advancing']
        missing += [topic for topic in self.expected if topic not in self.received]
        if self.crane_control and not self.count_subscribers(CRANE_COMMANDS):
            missing.append(f'the crane controller (subscriber on {CRANE_COMMANDS})')
        if not self.tf_buffer.can_transform('odom', 'base_scan', Time()):
            missing.append('TF odom -> base_scan')
        missing += [name for name, client in self.nav2_clients.items()
                    if not client.service_is_ready()]
        return missing

    def check(self):
        missing = self.missing()
        if not missing:
            self.timer.cancel()
            self.ready_pub.publish(Bool(data=True))
            self.get_logger().info('Simulation ready')
            self.release()
            return
        self.checks += 1
        if self.checks % round(LOG_PERIOD / CHECK_PERIOD) == 0:
            self.get_logger().info(f'Simulation not ready, waiting for: {", ".join(missing)}')

    def release(self):
        """Stop receiving sensor data: from here on the node only holds the latched message."""
        for subscription in self.subscriptions_held:
            self.destroy_subscription(subscription)
        self.tf_listener.unregister()
        for client in self.nav2_clients.values():
            self.destroy_client(client)


def main():
    rclpy.init()
    node = SimulationMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
