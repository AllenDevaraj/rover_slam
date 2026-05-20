#!/usr/bin/env python3
"""
ROS2 ArduPilot Rover Node
Combines steering/throttle control and IMU data publishing
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped, Vector3, Vector3Stamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32
import time
import threading
from pymavlink import mavutil
import numpy as np


class ArduPilotRoverNode(Node):
    def __init__(self):
        super().__init__('rover_node')

        # Parameters
        self.declare_parameter('connection_string', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('imu_frequency', 20.0)

        # IMU velocity estimation — shared between threads via lock
        self.imu_lock           = threading.Lock()
        self.estimated_velocity = 0.0
        self.last_imu_time      = None
        self.pitch_angle        = 0.0
        self.accel_noise_floor  = 0.25  # m/s² — tune while robot is stationary

        # Velocity PID — tune via /estimated_velocity topic
        # Order: get velocity_decay right, then vel_kp, then vel_ki
        self.declare_parameter('velocity_decay', 0.90)
        self.declare_parameter('vel_kp', 30.0)
        self.declare_parameter('vel_ki', 2.0)
        self.declare_parameter('vel_integral_max', 10.0)
        # Change to 'y' or 'z' if /imu/accel shows a different axis moving
        # when driving forward
        self.declare_parameter('forward_accel_axis', 'x')

        self.connection_string  = self.get_parameter('connection_string').value
        self.baud_rate          = self.get_parameter('baud_rate').value
        self.control_freq       = self.get_parameter('control_frequency').value
        self.imu_freq           = self.get_parameter('imu_frequency').value
        self.velocity_decay     = self.get_parameter('velocity_decay').value
        self.vel_kp             = self.get_parameter('vel_kp').value
        self.vel_ki             = self.get_parameter('vel_ki').value
        self.vel_integral_max   = self.get_parameter('vel_integral_max').value
        self.forward_accel_axis = self.get_parameter('forward_accel_axis').value

        # MAVLink state — written only by MAVLink thread
        self.master    = None
        self.connected = False
        self.armed     = False

        # Control — written by cmd_vel callback, read by MAVLink thread
        self.current_throttle = 0
        self.current_steering = 0
        self.last_cmd_time    = time.time()
        self.cmd_timeout      = 3.0

        # IMU velocity estimation — shared between threads via lock
        self.imu_lock           = threading.Lock()
        self.estimated_velocity = 0.0
        self.last_imu_time      = None
        self.pitch_angle        = 0.0   # integrated from gyro, used for gravity correction

        # Velocity PID state — written only by cmd_vel callback
        self.desired_velocity  = 0.0
        self.vel_integral      = 0.0
        self.last_vel_pid_time = None

        # QoS
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

        # Publishers
        self.imu_pub     = self.create_publisher(Imu,     '/imu',                sensor_qos)
        self.gyro_pub    = self.create_publisher(Vector3Stamped, 'imu/gyro',            sensor_qos)
        self.accel_pub   = self.create_publisher(Vector3Stamped, 'imu/accel',           sensor_qos)
        self.armed_pub   = self.create_publisher(Bool,    'rover/armed',         control_qos)
        self.vel_est_pub = self.create_publisher(Float32, '/estimated_velocity', sensor_qos)

        # Subscribers
        self.cmd_sub = self.create_subscription(
            TwistStamped, 'cmd_vel', self.cmd_vel_callback, control_qos
        )

        # Single background thread owns ALL MAVLink I/O
        self.mavlink_thread = threading.Thread(
            target=self.mavlink_run, daemon=True
        )
        self.mavlink_thread.start()

        self.get_logger().info(
            f'rover_node ready — vel_kp={self.vel_kp} '
            f'vel_ki={self.vel_ki} decay={self.velocity_decay} '
            f'axis={self.forward_accel_axis}'
        )

    # ------------------------------------------------------------------
    # MAVLink thread — only place self.master is touched
    # ------------------------------------------------------------------

    def mavlink_run(self):
        try:
            self.get_logger().info(f'Connecting on {self.connection_string}...')
            self.master = mavutil.mavlink_connection(
                self.connection_string, baud=self.baud_rate, timeout=10
            )
            heartbeat = self.master.wait_heartbeat(timeout=10)
            if heartbeat is None:
                self.get_logger().error('No heartbeat — check USB and baud rate')
                return

            self.get_logger().info(
                f'Connected: sys={self.master.target_system} '
                f'comp={self.master.target_component}'
            )
            self.connected = True
            self._request_streams()
            self._set_mode('ACRO')
            self._arm()

        except Exception as e:
            self.get_logger().error(f'Connection failed: {e}')
            return

        last_control_time = time.time()
        control_interval  = 1.0 / self.control_freq

        while rclpy.ok():
            now = time.time()

            try:
                msg = self.master.recv_match(blocking=False)
            except Exception as e:
                self.get_logger().warn(
                    f'recv_match error: {e}', throttle_duration_sec=3.0
                )
                msg = None

            if msg is not None:
                t = msg.get_type()
                if t == 'SCALED_IMU':
                    self._publish_imu(msg)
                elif t == 'HEARTBEAT':
                    self.armed = bool(
                        msg.base_mode &
                        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    self._publish_armed()

            if now - last_control_time >= control_interval:
                last_control_time = now
                self._send_control()

            time.sleep(0.002)

    # ------------------------------------------------------------------
    # MAVLink helpers
    # ------------------------------------------------------------------

    def _request_streams(self):
        interval_us = int(1000000 / self.imu_freq)
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, 26, interval_us, 0, 0, 0, 0, 0       # SCALED_IMU
        )
        self.get_logger().info(f'Streams requested: IMU={self.imu_freq}Hz')

    def _set_mode(self, mode_name):
        mode_mapping = self.master.mode_mapping()
        if mode_name not in mode_mapping:
            self.get_logger().error(f'Mode {mode_name} not available')
            return
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_mapping[mode_name]
        )
        start = time.time()
        while time.time() - start < 5:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if msg and mavutil.mode_string_v10(msg) == mode_name:
                self.get_logger().info(f'Mode set to {mode_name}')
                return
            time.sleep(0.1)
        self.get_logger().error(f'Failed to set mode {mode_name}')

    def _arm(self):
        self.get_logger().info('Arming...')
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        start = time.time()
        while time.time() - start < 10:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
            if msg and (msg.base_mode &
                        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                self.get_logger().info('Armed successfully')
                self.armed = True
                return
            time.sleep(0.1)
        self.get_logger().error('Failed to arm')

    def _send_control(self):
        if not self.armed:
            return
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            throttle, steering = 0, 0
            # Reset PID so stale windup doesn't carry into next command
            self.vel_integral      = 0.0
            self.last_vel_pid_time = None
        else:
            throttle = self.current_throttle
            steering = self.current_steering
        try:
            self.master.mav.manual_control_send(
                self.master.target_system,
                0, steering, throttle, 0, 0
            )
        except Exception as e:
            self.get_logger().error(f'Control send failed: {e}')

    # ------------------------------------------------------------------
    # IMU publish + velocity integration
    # ------------------------------------------------------------------

    def _publish_imu(self, msg):
        now = time.time()

        accel_x = (msg.xacc / 1000.0) * 9.80665
        accel_y = (msg.yacc / 1000.0) * 9.80665
        accel_z = (msg.zacc / 1000.0) * 9.80665

        gyro_x = msg.xgyro / 1000.0
        gyro_y = msg.ygyro / 1000.0
        gyro_z = msg.zgyro / 1000.0

        forward_accel = {'x': accel_x, 'y': accel_y, 'z': accel_z}[
            self.forward_accel_axis
        ]
        # Pitch rate axis is perpendicular to forward axis
        pitch_rate = {'x': gyro_y, 'y': gyro_x, 'z': gyro_y}[
            self.forward_accel_axis
        ]

        with self.imu_lock:
            if self.last_imu_time is not None:
                dt = float(np.clip(now - self.last_imu_time, 0.001, 0.1))

                self.pitch_angle += pitch_rate * dt

                gravity    = 9.80665
                accel_corr = forward_accel - gravity * np.sin(self.pitch_angle)

                if abs(accel_corr) < self.accel_noise_floor:
                    # Stationary — decay hard toward zero
                    self.estimated_velocity *= 0.5
                else:
                    self.estimated_velocity = float(np.clip(
                        (self.estimated_velocity + accel_corr * dt) * self.velocity_decay,
                        0.0,
                        1.0
                    ))

                # Safety decay — always runs, prevents infinite drift
                # from persistent bias getting past the noise floor
                self.estimated_velocity = float(np.clip(
                    self.estimated_velocity * 0.98,
                    0.0,
                    1.0
                ))

            self.last_imu_time = now
            vel_snapshot = self.estimated_velocity

        vel_msg      = Float32()
        vel_msg.data = vel_snapshot
        self.vel_est_pub.publish(vel_msg)

        gyro  = Vector3Stamped()
        gyro.header.stamp    = self.get_clock().now().to_msg()
        gyro.header.frame_id = 'imu_link'
        gyro.vector.x = gyro_x
        gyro.vector.y = gyro_y
        gyro.vector.z = gyro_z

        accel = Vector3Stamped()
        accel.header.stamp    = self.get_clock().now().to_msg()
        accel.header.frame_id = 'imu_link'
        accel.vector.x = accel_x
        accel.vector.y = accel_y
        accel.vector.z = accel_z

        self.gyro_pub.publish(gyro)
        self.accel_pub.publish(accel)

        imu_msg                           = Imu()
        imu_msg.header.stamp              = self.get_clock().now().to_msg()
        imu_msg.header.frame_id           = 'imu_link'
        imu_msg.angular_velocity          = gyro.vector
        imu_msg.linear_acceleration       = accel.vector
        imu_msg.orientation_covariance[0] = -1.0
        # Gyro covariance — how noisy is your gyro
        # Tune this: higher = trust IMU less, lower = trust IMU more
        imu_msg.angular_velocity_covariance[0] = 0.01  # roll
        imu_msg.angular_velocity_covariance[4] = 0.01  # pitch
        imu_msg.angular_velocity_covariance[8] = 0.01  # yaw — most important for 2D

        # Accel covariance — IMU accel is noisy, set relatively high
        imu_msg.linear_acceleration_covariance[0] = 0.1   # x — forward
        imu_msg.linear_acceleration_covariance[4] = 0.1   # y
        imu_msg.linear_acceleration_covariance[8] = 0.1   # z
        self.imu_pub.publish(imu_msg)

    # ------------------------------------------------------------------
    # Armed publisher
    # ------------------------------------------------------------------

    def _publish_armed(self):
        armed_msg      = Bool()
        armed_msg.data = self.armed
        self.armed_pub.publish(armed_msg)

    # ------------------------------------------------------------------
    # cmd_vel callback — velocity PID
    # ------------------------------------------------------------------

    def cmd_vel_callback(self, msg):
        throttle_raw = msg.twist.linear.x * -400

        # Only apply offset when actually commanding motion
        if abs(throttle_raw) > 10:
            offset = 100
            throttle_base = (
                throttle_raw + offset if throttle_raw >= 0
                else throttle_raw - offset
            )
        else:
            throttle_base = 0

        self.current_throttle = int(np.clip(throttle_base, -250, 250))
        self.current_steering = int(np.clip(msg.twist.angular.z * 500, -1000, 1000))
        self.last_cmd_time    = time.time()

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('Shutting down...')
        if self.master and self.armed:
            try:
                self.master.mav.manual_control_send(
                    self.master.target_system, 0, 0, 0, 0, 0
                )
                time.sleep(0.1)
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 0, 0, 0, 0, 0, 0, 0
                )
            except Exception:
                pass
        if self.master:
            self.master.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArduPilotRoverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()