"""
Show the robot's true pose in RViz, beside the one it estimates.

The lidar's frame hangs off the robot's estimate of its pose (odom -> base_footprint), so RViz
draws /scan from there, and the walls seem to move when the wheels slip. This node gives RViz a
second tree of frames, true/*, which hangs off the true pose instead:

  world -> true/base_footprint   here, from /odom_world
  true/base_footprint -> ...     from a robot_state_publisher with the frame prefix true/
  /scan_true                     /scan, in the frame true/base_scan

These are for display only: a real robot has no access to them, so nothing should navigate by them.
"""

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

PREFIX = 'true/'


class TrueFrames(Node):

    def __init__(self):
        super().__init__('true_frames')
        self.broadcaster = TransformBroadcaster(self)
        # Reliable, unlike the subscription: subscribers of either kind can then have it
        self.scan_publisher = self.create_publisher(LaserScan, 'scan_true', 10)
        self.create_subscription(Odometry, 'odom_world', self.pose_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

    def pose_callback(self, msg):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = PREFIX + msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)

    def scan_callback(self, msg):
        msg.header.frame_id = PREFIX + msg.header.frame_id
        self.scan_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = TrueFrames()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
