#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

class MoveAndMonitor(Node):
    def __init__(self):
        super().__init__('debug_vel')

        # Publisher
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # Subscriber
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.duration = 5.0  # seconds
        self.start_time = self.get_clock().now()

        # Timer at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.moving = True

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        self.get_logger().info(
            f"Position -> x: {pos.x:.3f}, y: {pos.y:.3f}, z: {pos.z:.3f}"
        )

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        cmd = TwistStamped()

        if elapsed < self.duration:
            cmd.twist.linear.x = 0.25
            cmd.twist.angular.z = 0.0
            self.cmd_pub.publish(cmd)
        else:
            if self.moving:
                self.get_logger().info("Stopped robot after 5 seconds.")
                self.moving = False

            # keep publishing zero to ensure stop
            self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = MoveAndMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()