#!/usr/bin/env python3
"""
Fake odometry from cmd_vel + IMU gyro.

Integrates commanded linear velocity and calibrated IMU angular velocity into a
pose, publishing /odom and the odom->base_link TF. Gives slam_toolbox a motion
prior on robots without wheel encoders.

Parameters:
  velocity_scale (float, default 1.0): multiplier applied to cmd_vel.linear.x.
    cmd_vel on this rover is normalized [-1, 1] rather than m/s, so tune this
    to match the actual speed the line follower drives at.
"""

import math
import os

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory


class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')

        self.declare_parameter('velocity_scale', 1.12)
        self.velocity_scale = self.get_parameter('velocity_scale').value

        self.declare_parameter('publish_tf', False)
        self.publish_tf = self.get_parameter('publish_tf').value

        self.gyro_bias_z = 0.0
        self.load_calibration()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.omega = 0.0
        self.last_time = self.get_clock().now()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, sensor_qos)

        # Relative name so launch-file remap can redirect to /fake_odom/raw,
        # which feeds the EKF without colliding with the EKF's /odom output.
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.publish_transform(self.get_clock().now())
        self.create_timer(0.05, self.update)

        self.get_logger().info(
            f'fake_odom up (velocity_scale={self.velocity_scale}, '
            f'gyro_bias_z={self.gyro_bias_z:.6f})')

    def load_calibration(self):
        try:
            calib_path = os.path.join(
                get_package_share_directory('robo_rover'),
                'calibration_results',
                'imu_calibration.yaml',
            )
            with open(calib_path) as f:
                data = yaml.safe_load(f)
            self.gyro_bias_z = data['imu_calibration']['gyro_bias']['z']
            self.get_logger().info(f'Loaded IMU calibration from {calib_path}')
        except Exception as e:
            self.get_logger().warn(
                f'Could not load IMU calibration ({e}); using zero gyro bias')

    def cmd_cb(self, msg: TwistStamped):
        self.v = msg.twist.linear.x * self.velocity_scale

    def imu_cb(self, msg: Imu):
        self.omega = -msg.angular_velocity.z + self.gyro_bias_z

    def publish_transform(self, now):
        q = Quaternion()
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)

        # Only broadcast TF if not using EKF downstream
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = q
            self.tf_broadcaster.sendTransform(tf)

        # Always publish /odom topic — EKF needs this regardless
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.omega
        # Pose covariance — how much we trust the integrated position
        # Diagonal of 6x6 matrix: x, y, z, roll, pitch, yaw
        odom.pose.covariance[0]  = 0.5    # x
        odom.pose.covariance[7]  = 0.5    # y
        odom.pose.covariance[35] = 0.3    # yaw

        # Twist covariance
        odom.twist.covariance[0]  = 0.1    # vx
        odom.twist.covariance[35] = 0.05   # vyaw
        self.odom_pub.publish(odom)

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        self.theta += self.omega * dt
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

        self.publish_transform(now)


def main():
    rclpy.init()
    node = FakeOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
