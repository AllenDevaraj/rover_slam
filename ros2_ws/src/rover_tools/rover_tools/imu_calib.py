#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


GRAVITY = 9.80665

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
DEFAULT_OUTPUT = os.path.join(PACKAGE_DIR, 'calibration_results', 'imu_calibration.yaml')


class ImuCalibNode(Node):
    def __init__(self):
        super().__init__('imu_calib_node')

        self.declare_parameter('num_samples', 500)
        self.declare_parameter('output_file', DEFAULT_OUTPUT)

        self.num_samples = self.get_parameter('num_samples').value
        self.output_file = self.get_parameter('output_file').value

        self.samples = []

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(Imu, '/imu', self.imu_cb, sensor_qos)

        self.get_logger().info(
            f'IMU calibration node started. Keep the robot STATIONARY. '
            f'Collecting {self.num_samples} samples...'
        )

    def imu_cb(self, msg: Imu):
        if len(self.samples) >= self.num_samples:
            return

        self.samples.append({
            'gx': msg.angular_velocity.x,
            'gy': msg.angular_velocity.y,
            'gz': msg.angular_velocity.z,
            'ax': msg.linear_acceleration.x,
            'ay': msg.linear_acceleration.y,
            'az': msg.linear_acceleration.z,
        })

        collected = len(self.samples)
        if collected % 50 == 0:
            self.get_logger().info(f'  {collected}/{self.num_samples} samples collected...')

        if collected >= self.num_samples:
            self.compute_and_save()
            rclpy.shutdown()

    def compute_and_save(self):
        n = len(self.samples)

        def mean(key):
            return sum(s[key] for s in self.samples) / n

        gyro_bias = {
            'x': mean('gx'),
            'y': mean('gy'),
            'z': mean('gz'),
        }

        accel_bias = {
            'x': mean('ax'),
            'y': mean('ay'),
            'z': mean('az') + GRAVITY,
        }

        calib = {
            'imu_calibration': {
                'num_samples': n,
                'gyro_bias': gyro_bias,
                'accel_bias': accel_bias,
            }
        }

        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        with open(self.output_file, 'w') as f:
            yaml.dump(calib, f, default_flow_style=False)

        self.get_logger().info(f'Calibration complete. Results written to: {self.output_file}')
        self.get_logger().info(
            f'  Gyro bias  (rad/s): x={gyro_bias["x"]:.6f}  y={gyro_bias["y"]:.6f}  z={gyro_bias["z"]:.6f}'
        )
        self.get_logger().info(
            f'  Accel bias (m/s²):  x={accel_bias["x"]:.6f}  y={accel_bias["y"]:.6f}  z={accel_bias["z"]:.6f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ImuCalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
