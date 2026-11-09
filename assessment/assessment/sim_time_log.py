"""
Prints the log messages of every node, from /rosout, stamped with simulation time.

ROS stamps log messages with wall time, whatever use_sim_time is: the logging library takes the
time when a message is logged, from the system clock, and knows nothing of a node's clock. This
node stamps each message with the latest time on /clock when it receives it, a few milliseconds
after it was logged (and the clock moves in steps of one physics step, 0.01 s, anyway). Each line
also shows the message's own wall time:

  [sim    37.870] [wall 23:59:38.722] [INFO] [node]: demo.

The simulation time is --- for messages logged before /clock was published, and for messages that
arrive more than STALE seconds after they were logged: /rosout keeps messages for subscribers that
join late, which would otherwise get the simulation time of when they arrived.

Parameter file (string): a file to write the lines to, instead of the terminal; empty: the
terminal. assessment.launch.py runs this node with sim_time_log:=true, and sets file from
sim_time_log_file.

This node runs on wall time, as it reads /clock itself and judges how old messages are by the
wall clock: the launch file sets use_sim_time to false for it.
"""

from datetime import datetime
import os
import sys

from rcl_interfaces.msg import Log
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy)
from rosgraph_msgs.msg import Clock

LEVELS = {Log.DEBUG: 'DEBUG', Log.INFO: 'INFO', Log.WARN: 'WARN', Log.ERROR: 'ERROR',
          Log.FATAL: 'FATAL'}
STALE = 1.0  # s of wall time from logging to arrival, past which the simulation time is unknown

# As /rosout is published: reliable, transient local, with a deep history
ROSOUT_QOS = QoSProfile(depth=1000, history=HistoryPolicy.KEEP_LAST,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SimTimeLog(Node):

    def __init__(self):
        super().__init__('sim_time_log')
        path = self.declare_parameter('file', '').value
        if path:
            path = os.path.abspath(os.path.expanduser(path))
            self.out = open(path, 'a', buffering=1)  # line buffered
            self.get_logger().info(f'writing log messages with simulation time to {path}')
        else:
            self.out = sys.stdout
        self.sim = None  # latest simulation time, in seconds
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.create_subscription(Log, '/rosout', self.on_log, ROSOUT_QOS)

    def on_clock(self, msg):
        self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_log(self, msg):
        if msg.name == self.get_name():
            return
        logged = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        arrived = self.get_clock().now().nanoseconds * 1e-9
        if self.sim is None or arrived - logged > STALE:
            sim = '   ---   '
        else:
            sim = f'{self.sim:9.3f}'
        wall = datetime.fromtimestamp(logged).strftime('%H:%M:%S.%f')[:-3]
        level = LEVELS.get(msg.level, str(msg.level))
        print(f'[sim {sim}] [wall {wall}] [{level}] [{msg.name}]: {msg.msg}', file=self.out,
              flush=True)

    def destroy_node(self):
        if self.out is not sys.stdout:
            self.out.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = SimTimeLog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
