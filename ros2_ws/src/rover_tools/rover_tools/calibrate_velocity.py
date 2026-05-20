#!/usr/bin/env python3
"""
One-shot calibration for fake_odom's velocity_scale.

Procedure:
  1. Sample IMU accel.x while stationary to measure bias.
  2. Publish a known cmd_vel.linear.x for N seconds.
  3. Integrate (accel.x - bias) to estimate actual velocity.
  4. Print velocity_scale = measured_velocity / commanded_velocity.

Run with rover_node already up (so cmd_vel drives the motors) but nothing
else publishing cmd_vel. Make sure the robot has clear space ahead — it
will drive forward ~0.5m.

Parameters:
  cmd_velocity (float, default 0.3):  cmd_vel.linear.x to send
  calibration_duration (float, 2.0):  seconds to drive
  bias_duration (float, 1.0):         seconds to measure IMU bias

Usage:
  ros2 run mal_planner calibrate_velocity
  # NEED TO CHANGE MSG TYPE TO TWIsT STAMPED IF RUNNING THIS

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


class VelCalibrator(Node):
    def __init__(self):
        super().__init__('velocity_calibrator')

        self.declare_parameter('cmd_velocity', 0.3)
        self.declare_parameter('calibration_duration', 2.0)
        self.declare_parameter('bias_duration', 1.0)
        self.cmd_v = self.get_parameter('cmd_velocity').value
        self.cal_duration = self.get_parameter('calibration_duration').value
        self.bias_duration = self.get_parameter('bias_duration').value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, 50)

        self.phase = 'wait_for_imu'
        self.bias_samples = []
        self.bias_x = 0.0
        self.v_integrated = 0.0
        self.latest_accel_x = 0.0
        self.got_imu = False
        self.last_time = None
        self.phase_start = None

        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f'Calibration: cmd={self.cmd_v}, drive={self.cal_duration}s')

    def imu_cb(self, msg: Imu):
        self.latest_accel_x = msg.linear_acceleration.x
        self.got_imu = True

    def tick(self):
        now = self.get_clock().now()

        if self.phase == 'wait_for_imu':
            if self.got_imu:
                self.phase = 'bias'
                self.phase_start = now
                self.get_logger().info('Sampling IMU bias (hold still)...')
            return

        if self.phase == 'bias':
            self.bias_samples.append(self.latest_accel_x)
            if (now - self.phase_start).nanoseconds * 1e-9 >= self.bias_duration:
                self.bias_x = sum(self.bias_samples) / len(self.bias_samples)
                self.phase = 'drive'
                self.phase_start = now
                self.last_time = now
                self.get_logger().info(
                    f'Bias: {self.bias_x:.3f} m/s². Driving forward...')
            return

        if self.phase == 'drive':
            twist = Twist()
            twist.linear.x = self.cmd_v
            self.cmd_pub.publish(twist)

            dt = (now - self.last_time).nanoseconds * 1e-9
            self.last_time = now
            corrected = self.latest_accel_x - self.bias_x
            self.v_integrated += corrected * dt

            if (now - self.phase_start).nanoseconds * 1e-9 >= self.cal_duration:
                self.cmd_pub.publish(Twist())
                scale = abs(self.v_integrated) / self.cmd_v if self.cmd_v else 0.0
                self.get_logger().info('=== CALIBRATION RESULT ===')
                self.get_logger().info(f'  commanded:      {self.cmd_v}')
                self.get_logger().info(f'  measured (m/s): {self.v_integrated:.3f}')
                self.get_logger().info(f'  velocity_scale: {scale:.3f}')
                self.get_logger().info(
                    'Set this in slam_nav.launch.py fake_odom parameters.')
                self.phase = 'done'
            return

        if self.phase == 'done':
            self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = VelCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
