"""
Ends the simulation once it has run for a given time, counted from when it is ready.

Parameter time_limit (float): the time, in seconds of simulation time (from /clock), > 0.

Counting starts when /simulation_ready turns true, so that however long the simulation takes to
start, a solution always gets the whole time. When it is up, this node logs that it is and exits;
assessment.launch.py, which runs it with the launch argument time_limit, then shuts the whole
launch down. Every 60 s of simulation time, it logs how much time is left.

(launch_ros's RosTimer, after SetUseSimTime, also counts simulation time, but from when the launch
starts it: it cannot wait for /simulation_ready.)

This node runs on wall time, as it reads /clock itself: the launch file sets use_sim_time to false
for it.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool

REPORT_PERIOD = 60.0  # s of simulation time between reports of the time left
READY_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)


class TimeLimit(Node):

    def __init__(self):
        super().__init__('time_limit')
        self.limit = float(self.declare_parameter('time_limit', 900.0).value)
        if self.limit <= 0:
            raise ValueError(f'time_limit must be > 0, got {self.limit}')
        self.ready = False
        self.start = None  # simulation time when the simulation was ready
        self.reached = False
        self.next_report = REPORT_PERIOD
        self.create_subscription(Bool, '/simulation_ready', self.on_ready, READY_QOS)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.get_logger().info(f'The simulation ends {self.limit:g} s of simulation time after '
                               'it is ready')

    def on_ready(self, msg):
        self.ready = self.ready or msg.data

    def on_clock(self, msg):
        now = msg.clock.sec + msg.clock.nanosec * 1e-9
        if not self.ready or self.reached:
            return
        if self.start is None:
            self.start = now
            self.get_logger().info(f'Simulation ready at {now:.2f} s: it ends at '
                                   f'{now + self.limit:.2f} s of simulation time')
        elapsed = now - self.start
        if elapsed >= self.limit:
            self.reached = True
            self.get_logger().warning(
                f'Time limit reached: {self.limit:g} s of simulation time since the simulation '
                'was ready. Ending the simulation.')
        elif elapsed >= self.next_report:
            self.get_logger().info(f'{self.limit - elapsed:.0f} s of simulation time left')
            self.next_report += REPORT_PERIOD


def main():
    rclpy.init()
    node = TimeLimit()
    try:
        while rclpy.ok() and not node.reached:
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
