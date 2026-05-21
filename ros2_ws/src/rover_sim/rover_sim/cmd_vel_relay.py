#!/usr/bin/env python3
"""Relay /cmd_vel (TwistStamped, from autonomy) and /cmd_vel_teleop (Twist, from
keyboard teleop) onto a single plain Twist topic consumed by the Gazebo drive
system via ros_gz_bridge. Resolves the pre-existing cmd_vel type split."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.declare_parameter('output_topic', '/model/rover/cmd_vel')
        out = self.get_parameter('output_topic').get_parameter_value().string_value
        self.pub = self.create_publisher(Twist, out, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self._stamped_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_teleop', self._twist_cb, 10)
        self.get_logger().info(
            f'cmd_vel_relay: /cmd_vel (TwistStamped) + /cmd_vel_teleop (Twist) -> {out} (Twist)')

    def _stamped_cb(self, msg: TwistStamped) -> None:
        self.pub.publish(msg.twist)

    def _twist_cb(self, msg: Twist) -> None:
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
