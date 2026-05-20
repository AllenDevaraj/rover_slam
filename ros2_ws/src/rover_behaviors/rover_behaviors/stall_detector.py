import rclpy
import numpy as np
import yaml
import os
from collections import deque
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node, QoSProfile
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType, SetParametersResult
from geometry_msgs.msg import TwistStamped, Vector3, Vector3Stamped
from std_msgs.msg import Float64
import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class StallDetector(Node):
    NORMAL   = 'NORMAL'
    STALLING = 'STALLING'

    def __init__(self):
        super().__init__('stall_detector')
        self.declare_parameter('imu_stall_threshold', 0.4)
        self.declare_parameter('accel_window_sec', 1.5)
        self.declare_parameter('duration_till_stall', 1.0)
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('boost_step', 0.01)
        self.declare_parameter('boost_interval', 1.0)
        self.declare_parameter('forward_velocity_offset', 0.025)
        self.declare_parameter('min_lock_duration', 5.0)
        self.declare_parameter('max_forward_velocity', 0.275)
        self.declare_parameter('over_max_delay', 2.5)
        self.declare_parameter('over_max_step', 0.1)
        self.declare_parameter('emergency_max_velocity', 0.5)
        self.imu_stall_threshold = self.get_parameter('imu_stall_threshold').get_parameter_value().double_value
        self.accel_window_sec = self.get_parameter('accel_window_sec').get_parameter_value().double_value
        self.duration_till_stall = self.get_parameter('duration_till_stall').get_parameter_value().double_value
        self.gravity = self.get_parameter('gravity').get_parameter_value().double_value
        self.boost_step = self.get_parameter('boost_step').get_parameter_value().double_value
        self.forward_velocity_offset = self.get_parameter('forward_velocity_offset').get_parameter_value().double_value
        self.min_lock_duration = self.get_parameter('min_lock_duration').get_parameter_value().double_value
        self.max_forward_velocity = self.get_parameter('max_forward_velocity').get_parameter_value().double_value
        self.over_max_delay = self.get_parameter('over_max_delay').get_parameter_value().double_value
        self.over_max_step = self.get_parameter('over_max_step').get_parameter_value().double_value
        self.emergency_max_velocity = self.get_parameter('emergency_max_velocity').get_parameter_value().double_value
        self.get_logger().info(f'IMU Stall threshold set to: {self.imu_stall_threshold}')
        self.get_logger().info(f'Accel window set to: {self.accel_window_sec}s')
        self.get_logger().info(f'Duration till stall set to: {self.duration_till_stall}')
        self.get_logger().info(f'Gravity set to: {self.gravity}')
        self.get_logger().info(f'Boost step set to: {self.boost_step}')
        self.get_logger().info(f'Forward velocity offset set to: {self.forward_velocity_offset}')
        self.get_logger().info(f'Min lock duration set to: {self.min_lock_duration}')
        self.get_logger().info(f'Max forward velocity set to: {self.max_forward_velocity}')
        self.get_logger().info(f'Over-max delay set to: {self.over_max_delay}s')
        self.get_logger().info(f'Over-max step set to: {self.over_max_step}')
        self.get_logger().info(f'Emergency max velocity set to: {self.emergency_max_velocity}')
        self.add_on_set_parameters_callback(self.on_param_change)

        calib_path = os.path.join(
            get_package_share_directory('robo_rover'),
            'calibration_results', 'imu_calibration.yaml'
        )
        with open(calib_path, 'r') as f:
            calib = yaml.safe_load(f)
        bias = calib['imu_calibration']['accel_bias']
        self.accel_bias = np.array([bias['x'], bias['y'], bias['z']])
        self.get_logger().info(f'Loaded IMU accel bias: {self.accel_bias}')

        self.declare_parameter('initial_min_linear', 0.22)
        self.declare_parameter('initial_forward_velocity', 0.22)
        initial_min = self.get_parameter('initial_min_linear').get_parameter_value().double_value
        initial_fwd = self.get_parameter('initial_forward_velocity').get_parameter_value().double_value

        # Service client for setting line_follower parameters
        self.param_client = self.create_client(SetParameters, '/line_follower/set_parameters')

        # Anti-stall state machine (NORMAL → STALLING → HOLDING → RAMPING → NORMAL)
        self.stall_state = self.NORMAL

        # min_linear: raised during startup stall until rover moves, then locked permanently
        self.min_linear_base = initial_min
        self.min_linear_current = initial_min
        self.min_linear_locked = False

        # forward_velocity: raised alongside min_linear until rover moves; only fwd_vel adjusted after that
        self.forward_velocity_base = initial_fwd
        self.forward_velocity_current = initial_fwd

        # Push initial low values to line_follower before any callbacks fire
        if self.param_client.wait_for_service(timeout_sec=2.0):
            self.set_params(initial_min, initial_fwd)
            self.get_logger().info(f'Initialized line_follower: min_linear={initial_min:.3f}, forward_velocity={initial_fwd:.3f}')
        else:
            self.get_logger().warn('line_follower/set_parameters not ready — initial values not pushed')

        # Timing for stall detection and boosting
        self.last_step_time   = 0.0
        self.boost_interval   = self.get_parameter('boost_interval').get_parameter_value().double_value
        self.stall_start_time = None  # set when stall conditions first met; cleared on recovery
        self.stall_clear_time = None  # set when stall first clears; min_linear locked after min_lock_duration
        self.stall_enter_time = None  # set when STALLING state is entered
        self.accel_history    = deque()  # (timestamp, accel_deviation) rolling window

        # Message filter to sync cmd_vel and imu messages for stall detection
        # Need IMU reliable QoS type
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

        self.accel_mag_pub = self.create_publisher(Float64, '/stall_detector/accel_mag', 10)

        sub = message_filters.Subscriber(self, TwistStamped, '/cmd_vel', qos_profile=control_qos)
        imu_sub = message_filters.Subscriber(self, Vector3Stamped, '/imu/accel', qos_profile=sensor_qos)
        self.ts = message_filters.ApproximateTimeSynchronizer([sub, imu_sub], queue_size=10, slop=0.5)
        self.ts.registerCallback(self.stall_detector_callback)

    def on_param_change(self, params):
        for param in params:
            if param.name == 'imu_stall_threshold':
                self.imu_stall_threshold = param.value
                self.get_logger().info(f'IMU Stall threshold updated to: {self.imu_stall_threshold}')
            elif param.name == 'accel_window_sec':
                self.accel_window_sec = param.value
                self.get_logger().info(f'Accel window updated to: {self.accel_window_sec}s')
            elif param.name == 'duration_till_stall':
                self.duration_till_stall = param.value
                self.get_logger().info(f'Duration till stall updated to: {self.duration_till_stall}')
            elif param.name == 'gravity':
                self.gravity = param.value
                self.get_logger().info(f'Gravity updated to: {self.gravity}')
            elif param.name == 'boost_step':
                self.boost_step = param.value
                self.get_logger().info(f'Boost step updated to: {self.boost_step}')
            elif param.name == 'boost_interval':
                self.boost_interval = param.value
                self.get_logger().info(f'Boost interval updated to: {self.boost_interval}')
            elif param.name == 'forward_velocity_offset':
                self.forward_velocity_offset = param.value
                self.get_logger().info(f'Forward velocity offset updated to: {self.forward_velocity_offset}')
            elif param.name == 'min_lock_duration':
                self.min_lock_duration = param.value
                self.get_logger().info(f'Min lock duration updated to: {self.min_lock_duration}')
            elif param.name == 'max_forward_velocity':
                self.max_forward_velocity = param.value
                self.get_logger().info(f'Max forward velocity updated to: {self.max_forward_velocity}')
            elif param.name == 'over_max_delay':
                self.over_max_delay = param.value
                self.get_logger().info(f'Over-max delay updated to: {self.over_max_delay}s')
            elif param.name == 'over_max_step':
                self.over_max_step = param.value
                self.get_logger().info(f'Over-max step updated to: {self.over_max_step}')
            elif param.name == 'emergency_max_velocity':
                self.emergency_max_velocity = param.value
                self.get_logger().info(f'Emergency max velocity updated to: {self.emergency_max_velocity}')
        return SetParametersResult(successful=True)

    def detect_stall(self, cmd_vel_msg, imu_msg):
        accel_raw = np.array([imu_msg.vector.x, imu_msg.vector.y, imu_msg.vector.z])
        accel_mag = float(np.linalg.norm(accel_raw - self.accel_bias))
        self.accel_mag_pub.publish(Float64(data=accel_mag))

        cmd_vel_mag = np.sqrt(
            cmd_vel_msg.twist.linear.x**2 +
            cmd_vel_msg.twist.linear.y**2 +
            cmd_vel_msg.twist.linear.z**2
        )

        now = self.get_clock().now().nanoseconds / 1e9
        accel_deviation = abs(accel_mag - self.gravity)

        # Rolling window: keep only samples within accel_window_sec
        while self.accel_history and self.accel_history[0][0] < now - self.accel_window_sec:
            self.accel_history.popleft()
        self.accel_history.append((now, accel_deviation))
        rolling_max = max(dev for _, dev in self.accel_history)

        # Stall requires commanded motion AND no high-accel spike in the recent window
        conditions_met = cmd_vel_mag >= self.min_linear_current and rolling_max < self.imu_stall_threshold

        if conditions_met:
            if self.stall_start_time is None:
                self.stall_start_time = now
            elapsed = now - self.stall_start_time
            self.get_logger().info(f'cmd={cmd_vel_mag:.3f} (thr={self.min_linear_current:.3f}), accel_dev={accel_deviation:.3f}, win_max={rolling_max:.3f}, stall duration: {elapsed:.2f}s')
            return elapsed >= self.duration_till_stall
        else:
            self.stall_start_time = None
            self.get_logger().info(f'cmd={cmd_vel_mag:.3f} (thr={self.min_linear_current:.3f}), accel_dev={accel_deviation:.3f}, win_max={rolling_max:.3f}')
            return False

    def set_params(self, min_linear: float, forward_velocity: float):
        """Fire-and-forget set of min_linear and forward_velocity on line_follower."""
        if not self.param_client.service_is_ready():
            self.get_logger().warn('[anti-stall] line_follower/set_parameters not ready yet')
            return
        def make_param(name, value):
            p = Parameter()
            p.name = name
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
            return p
        req = SetParameters.Request()
        req.parameters = [make_param('min_linear', min_linear), make_param('forward_velocity', forward_velocity)]
        self.param_client.call_async(req)
        self.get_logger().info(f'[anti-stall] min_linear → {min_linear:.3f}  forward_velocity → {forward_velocity:.3f}')

    def stall_detector_callback(self, cmd_vel_msg, imu_msg):
        # Record current time and check stall for state machine
        now = self.get_clock().now().nanoseconds / 1e9
        stalling = self.detect_stall(cmd_vel_msg, imu_msg)

        # State NORMAL: watch for stall; lock min_linear after continuous movement
        if self.stall_state == self.NORMAL:
            if not self.min_linear_locked and self.stall_clear_time is not None:
                elapsed = now - self.stall_clear_time
                if elapsed >= self.min_lock_duration:
                    self.min_linear_locked = True
                    self.min_linear_base = self.min_linear_current
                    self.forward_velocity_current = min(
                        self.min_linear_current + self.forward_velocity_offset,
                        self.max_forward_velocity
                    )
                    self.forward_velocity_base = self.forward_velocity_current
                    self.set_params(self.min_linear_current, self.forward_velocity_current)
                    self.get_logger().info(
                        f'min_linear locked at {self.min_linear_current:.3f} after {self.min_lock_duration:.1f}s of movement, '
                        f'forward_velocity set to {self.forward_velocity_current:.3f}'
                    )
                else:
                    self.get_logger().info(f'Waiting to lock min_linear — {elapsed:.1f}/{self.min_lock_duration:.1f}s')

            if stalling:
                self.get_logger().warn('Stall detected — entering STALLING')
                self.stall_clear_time = None  # reset continuous-movement timer
                self.stall_state = self.STALLING
                self.stall_enter_time = now
                self.last_step_time = now - self.boost_interval  # trigger immediate first boost

        # State STALLING: raise velocities until rover moves
        elif self.stall_state == self.STALLING:
            if stalling:
                if now - self.last_step_time >= self.boost_interval:
                    time_in_stall = now - self.stall_enter_time
                    if time_in_stall >= self.over_max_delay:
                        # Emergency boost: exceed max_forward_velocity in larger increments
                        self.forward_velocity_current = min(
                            self.forward_velocity_current + self.over_max_step,
                            self.emergency_max_velocity
                        )
                        self.get_logger().warn(
                            f'[emergency boost] stalled {time_in_stall:.1f}s — '
                            f'forward_velocity → {self.forward_velocity_current:.3f} '
                            f'(emergency cap: {self.emergency_max_velocity:.3f})'
                        )
                    else:
                        if not self.min_linear_locked:
                            self.min_linear_current += self.boost_step
                        self.forward_velocity_current = min(
                            self.forward_velocity_current + self.boost_step,
                            self.max_forward_velocity
                        )
                    self.last_step_time = now
                    self.set_params(self.min_linear_current, self.forward_velocity_current)
            else:
                # Stall cleared
                if self.forward_velocity_current > self.max_forward_velocity:
                    self.forward_velocity_current = self.max_forward_velocity
                if self.forward_velocity_current >= 0.3:
                    self.forward_velocity_current = 0.3
                    self.get_logger().info(
                        f'Stall cleared — clamped forward_velocity to 0.300'
                    )
                elif self.forward_velocity_current < 0.27:
                    self.forward_velocity_current = 0.27
                    self.get_logger().info(
                        f'Stall cleared — raised forward_velocity to 0.270'
                    )
                else:
                    self.get_logger().info(
                        f'Stall cleared — forward_velocity holding at {self.forward_velocity_current:.3f}'
                    )
                self.set_params(self.min_linear_current, self.forward_velocity_current)
                if not self.min_linear_locked and self.stall_clear_time is None:
                    self.stall_clear_time = now
                    self.get_logger().info(
                        f'Starting {self.min_lock_duration:.1f}s timer before locking min_linear '
                        f'(current: {self.min_linear_current:.3f})'
                    )
                self.forward_velocity_base = self.forward_velocity_current
                self.stall_enter_time = None
                self.stall_state = self.NORMAL


def main(args=None):
    rclpy.init(args=args)
    node = StallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
