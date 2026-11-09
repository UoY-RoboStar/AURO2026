#
# Copyright (c) 2025 University of York and others
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   * Alan Millard - initial contributor
#   * Pedro Ribeiro - revised implementation
#

from collections import deque

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
import tf_transformations

BUFFER_SIZE = 1000


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')

        self.declare_parameter('x_pose', 0.0)
        self.declare_parameter('y_pose', 0.0)
        self.declare_parameter('yaw', 0.0)

        self.odom_pose_history = deque(maxlen=BUFFER_SIZE)
        self.true_pose_history = deque(maxlen=BUFFER_SIZE)

        # Wheel odometry starts from zero wherever the robot is spawned, so the odom frame is at
        # the start pose in the world. Gazebo does not publish this transform, and RViz needs it
        # to show /odom and /odom_world together.
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Convert yaw to quaternion (roll=pitch=0)
        quat = tf_transformations.quaternion_from_euler(
            0, 0, self.get_parameter('yaw').value)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'odom'
        t.transform.translation.x = self.get_parameter('x_pose').value
        t.transform.translation.y = self.get_parameter('y_pose').value
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Send transform once: static transforms do not change
        self.static_broadcaster.sendTransform(t)
        self.get_logger().info('Published static transform world to odom')

        self.odom_path_publisher = self.create_publisher(Path, 'odom_path', 10)
        self.true_path_publisher = self.create_publisher(Path, 'true_path', 10)

        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.true_pose_subscriber = self.create_subscription(
            Odometry, 'odom_world', self.true_pose_callback, 10)

    def odom_callback(self, msg):
        self.publish_path(msg, 'odom', self.odom_pose_history, self.odom_path_publisher)

    def true_pose_callback(self, msg):
        self.publish_path(msg, 'world', self.true_pose_history, self.true_path_publisher)

    def publish_path(self, msg, frame_id, history, publisher):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = msg.header.stamp
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose = msg.pose.pose
        history.append(pose_stamped)

        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = msg.header.stamp
        path.poses = list(history)
        publisher.publish(path)


def main(args=None):
    rclpy.init(args=args)

    node = PathPublisher()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
