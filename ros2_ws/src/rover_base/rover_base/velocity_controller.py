#!/usr/bin/evn python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np

class MovingAverage:
    def __init__(self, size):
        self.window_size = size
        self.values = []

    def filter(self, new_value):
        self.values.append(new_value)
        if len(self.values) > self.window_size:
            self.values.pop(0)
        return sum(self.values) / len(self.values)

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        
        # --- QoS Definitions ---
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # --- Parameters ---
        self.accel_filter = MovingAverage(size=10)
        self.kp, self.ki, self.kd = 1.5, 0.5, 0.1
        self.accel_noise_threshold = 0.05
        self.estop_threshold = 15.0  # m/s^2 - Trigger for crash/impact
        
        # --- State ---
        self.is_calibrated = False
        self.calibration_samples = []
        self.accel_bias = 0.0
        self.estop_active = False
        
        self.target_vel = 0.0
        self.estimated_vel = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = self.get_clock().now()

        # --- Subs/Pubs ---
        self.cmd_sub = self.create_subscription(TwistStamped, 'cmd_vel', self.cmd_callback, control_qos)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, sensor_qos)
        self.motor_pub = self.create_publisher(TwistStamped, 'motor_cmd', 1)

        self.get_logger().info("Controller Initialized. Calibrating IMU...")

    def imu_callback(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0: return
        self.last_time = now

        # 1. Calibration
        if not self.is_calibrated:
            self.calibration_samples.append(msg.linear_acceleration.x)
            if len(self.calibration_samples) >= 100:
                self.accel_bias = sum(self.calibration_samples) / 100
                self.is_calibrated = True
                self.get_logger().info(f"Calibration Complete. Bias: {self.accel_bias:.4f}")
            return

        # 2. Filter & Impact Detection
        raw_accel = msg.linear_acceleration.x - self.accel_bias
        filtered_accel = self.accel_filter.filter(raw_accel)

        if abs(filtered_accel) > self.estop_threshold:
            self.estop_active = True
            self.get_logger().error("EMERGENCY STOP: Impact Detected!")

        if self.estop_active:
            self.publish_zero()
            return

        # 3. Drift Comp (ZUPT)
        if abs(self.target_vel) < 0.01 and abs(filtered_accel) < self.accel_noise_threshold:
            self.estimated_vel = 0.0
            self.integral = 0.0
        else:
            self.estimated_vel += filtered_accel * dt

        # 4. PID Logic
        error = self.target_vel - self.estimated_vel
        self.integral = float(np.clip(self.integral + error * dt, -1.0, 1.0))
        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error

        # 5. Output
        out_msg = TwistStamped()
        out_msg.twist.linear.x = float(np.clip(output, -1.0, 1.0))
        self.motor_pub.publish(out_msg)

    def cmd_callback(self, msg):
        self.target_vel = msg.linear.x
        # Reset E-Stop if user sends a stop command manually
        if self.estop_active and abs(msg.linear.x) < 0.01:
            self.estop_active = False
            self.get_logger().info("E-Stop Cleared.")

    def publish_zero(self):
        msg = TwistStamped()
        self.motor_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
