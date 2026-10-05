#!/usr/bin/env python3
#
# Sample solutions to Task 5 of Practical 2, but with graceful handling of
# SIGINT kill signal, stoping the robot before terminating.
#

import rclpy
import signal

from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TwistStamped

class Task5GracefulPublisher(Node):

    def __init__(self):

        # Initialise teh node with the name 'task5_publisher'
        super().__init__('task5_graceful_publisher')

        # Create a publisher for the 'cmd_vel' topic with message type Twist
        # Notice how we use 'cmd_vel' rather than '/cmd_vel' as the topic name.
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)

        # Period for the timer in seconds
        timer_period = 1  # seconds

        # Create a timer to call the timer_callback function every 1 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create a guard condition to be used to trigger the executor
        self.guard_condition = self.create_guard_condition(self.shutdown_hook)

        # Save old signal handler for SIGINT
        self.old_handler = signal.signal(signal.SIGINT, self.sigint_handler)

    def timer_callback(self):

        # Create the Twist message
        msg = TwistStamped()

        # Angular is a component of type Twist
        msg.twist = Twist()
        msg.twist.angular.z = 1.0

        # Alternatively, we could have written just:
        # msg = TwistStamped()
        # msg.twist.angular.z = 1.0

        # Above we have ignored the 'stamp' part of the message. You
        # will find that in ROS several message types include a header
        # with a time stamp and frame_id. So we could have included 
        # the time at which the message is sent as additional information:
        #
        # msg.header.stamp = self.get_clock().now().to_msg()
        # 
        # You can try and uncomment the code below and examine the content
        # on the topic 'cmd_vel' from another terminal to see what values
        # are actually published.
        
        # Publish the message
        self.publisher_.publish(msg)

        # Log the message to the console
        self.get_logger().info(f"Publishing: '{msg}'")

    def shutdown_hook(self):

        # Log the shutdown message to console
        self.get_logger().info('Shutting down node...')

        # Create a Twist message to stop the robot
        msg = TwistStamped()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0

        # Publish the Twist message with 0 velocity for linear/angular components
        self.publisher_.publish(msg)

        # Log the message to the console
        self.get_logger().info(f"Stopping robot by publishing: '{msg}'")

        # Shutdown node
        rclpy.shutdown()

    # This is a custom handler for the SIGINT signal that catches Ctrl+C.
    def sigint_handler(self, signum, frame):

        # Trigger the guard condition to wake up the executor
        self.guard_condition.trigger()

        # Restore the old signal handler for SIGINT
        signal.signal(signal.SIGINT, self.old_handler)

def main(args=None):

    # Pass command line arguments to the rclpy library
    rclpy.init(args=args)

    # Create the node
    node = Task5GracefulPublisher()

    try:
        # Spin the node so the callbacks are processed.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()