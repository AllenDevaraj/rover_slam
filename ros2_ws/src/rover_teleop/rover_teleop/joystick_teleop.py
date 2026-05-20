#!/usr/bin/env python3
"""Xbox controller teleop via /dev/input/js0 (Linux raw joystick API).

Axis 0: left/right  — left = -32767, right = +32767
Axis 1: fwd/back    — forward = -32767, backward = +32767
Deadband: 7500 raw counts in every direction.
Output: Twist on cmd_vel with axes scaled to [-1.0, 1.0].
"""

import struct
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Linux joystick event: uint32 time_ms, int16 value, uint8 type, uint8 number
JS_EVENT_FMT = "IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)
JS_EVENT_AXIS = 0x02

DEVICE = "/dev/input/js0"
DEADBAND = 7500
RAW_MAX = 32767
PUBLISH_HZ = 30.0


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__("joystick_teleop")
        control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel", control_qos)

        self.axis = [0.0, 0.0]  # [axis0, axis1]
        self.lock = threading.Lock()

        self.js_thread = threading.Thread(target=self.read_js, daemon=True)
        self.js_thread.start()

        self.create_timer(1.0 / PUBLISH_HZ, self.publish)
        self.get_logger().info(f"Joystick teleop running on {DEVICE}")

    def read_js(self):
        try:
            with open(DEVICE, "rb") as js:
                while rclpy.ok():
                    data = js.read(JS_EVENT_SIZE)
                    if len(data) < JS_EVENT_SIZE:
                        break
                    _, value, evt_type, number = struct.unpack(JS_EVENT_FMT, data)
                    if (evt_type & JS_EVENT_AXIS) and number in (0, 1):
                        scaled = self.apply_deadband(value, number)
                        with self.lock:
                            self.axis[number] = scaled
        except OSError as e:
            self.get_logger().error(f"Cannot read {DEVICE}: {e}")

    def apply_deadband(self, raw: int, number: int) -> float:
        """Return 0.0 inside deadband, else scale the live range to [-1.0, 1.0]."""
        if abs(raw) < DEADBAND:
            return 0.0
        sign = 1 if raw > 0 else -1
        live = abs(raw) - DEADBAND
        if number == 1:
            command = 0.3 * sign * live / (RAW_MAX - DEADBAND)
            self.get_logger().info(f"Forawred Command = {command}")
        else:
            command = -2.0*(sign * live / (RAW_MAX - DEADBAND))
            self.get_logger().info(f"Lateral Command = {command}")
        return command

    def publish(self):
        with self.lock:
            axis0, axis1 = self.axis

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Axis 1 forward = -32767 raw → negate so forward stick = positive linear.x
        msg.twist.linear.x = -axis1
        # Axis 0 left = -32767 raw → negate so left stick = positive angular.z
        msg.twist.angular.z = -axis0
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = JoystickTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(TwistStamped())
        rclpy.shutdown()


if __name__ == "__main__":
    main()
