#!/usr/bin/env python3
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, TwistStamped
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
import time


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower')

        # Parameters
        self.declare_parameter('forward_velocity', 0.325)
        self.declare_parameter('min_linear', 0.22)
        self.declare_parameter('angular_velocity', 1.9)
        self.declare_parameter('grid_n', 15)

        self.forward_velocity = self.get_parameter('forward_velocity').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        self.grid_n           = self.get_parameter('grid_n').value
        self.add_on_set_parameters_callback(self.on_param_change)

        # PID gains
        self.kp = 0.8
        self.ki = 0.0
        self.kd = 0.2

        # Velocity limits
        self.max_linear  = 0.375
        self.min_linear  = 0.225
        self.max_angular = 12.0

        # PID state
        self.error      = 0.0
        self.prev_error = 0.0
        self.integral   = 0.0
        self.derivative = 0.0
        self.last_time  = None

        # Lost-line behavior
        self.lost_count      = 0
        self.last_seen_error = 0.0
        self.search_bias     = 1.2
        self.search_gain     = 0.8
        # FIX 5: cap how far lost_count can drive the search ramp
        self.max_lost_ramp   = 10000

        # Right-corner detector state
        self.corner_streak          = 0
        self.corner_debounce_frames = 2
        self.turn_cooldown_sec      = 25.0
        self.turn_time              = 2.25
        self.last_turn_time         = self.get_clock().now()
        self.num_turns              = 0

        # FIX 1: non-blocking turn state machine
        self.turning            = False
        self.turn_start         = None
        self.current_turn_time  = self.turn_time  # velocity-scaled at turn start

        self.frame_w = 640
        self.frame_h = 480

        self.bridge  = CvBridge()
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 1)
        self.closure_sub = self.create_subscription(
            Bool,
            '/course_complete',
            self.stop_callback,
            1
        )

        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            1
        )

        self.get_logger().info('LineFollowerNode ready')

    def stop_callback(self, msg):
        if(msg.data):
            self.destroy_node()
            rclpy.shutdown()


    def on_param_change(self, params):
        for param in params:
            if param.name == 'forward_velocity':
                self.forward_velocity = param.value
                self.get_logger().info(f'forward_velocity updated to {self.forward_velocity:.3f}')
            elif param.name == 'min_linear':
                self.min_linear = param.value
                self.get_logger().info(f'min_linear updated to {self.min_linear:.3f}')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Grid helper
    # ------------------------------------------------------------------

    def get_blue_grid(self, binary_mask, n):
        h, w   = binary_mask.shape
        cell_h = h // n
        cell_w = w // n
        grid   = np.zeros((n, n), dtype=bool)
        for row in range(n):
            for col in range(n):
                cell = binary_mask[
                    row * cell_h:(row + 1) * cell_h,
                    col * cell_w:(col + 1) * cell_w
                ]
                grid[row, col] = cell.mean() > 0.025
        return grid

    # ------------------------------------------------------------------
    # Corner detection  (FIX 4)
    # ------------------------------------------------------------------

    def detect_right_corner(self, binary_mask):
        """Detect a right 90-degree turn.

        Added vertical-distribution check: a true right corner has pixels
        concentrated in the BOTTOM rows (near robot) AND the RIGHT columns.
        A wide straight line has pixels spread across all rows uniformly,
        so it fails the bottom-heavy test and won't false-trigger.
        """
        h, w       = binary_mask.shape
        total_mass = int(binary_mask.sum())
        if total_mass < 100:
            return False

        # (A) Right mass > left mass
        right_mass = int(binary_mask[:, w // 2:].sum())
        left_mass  = int(binary_mask[:, :w // 2].sum())
        right_bias = right_mass > 1.5 * left_mass

        # (B) Right edge strip
        edge_strip_width = max(3, w // 20)
        edge_pixels      = int(binary_mask[:, -edge_strip_width:].sum())
        touches_right    = edge_pixels > 20

        # (C) Horizontal extent
        col_has_pixel     = binary_mask.any(axis=0)
        horizontal_extent = int(col_has_pixel.sum()) / w
        wide_extent       = horizontal_extent > 0.40

        # (D) NEW — bottom-heavy vertical distribution
        # Pixels should be denser in the bottom half than the top half
        # for a corner approaching the robot, not a distant straight line
        bottom_mass  = int(binary_mask[h // 2:, :].sum())
        top_mass     = int(binary_mask[:h // 2, :].sum())
        bottom_heavy = bottom_mass > 1.2 * top_mass

        # print(
        #     f'(A) right_bias={right_bias} '
        #     f'(A) touches_right={touches_right} '
        #     f'(A) wide_extent={wide_extent} '
        #     f'(A) bottom_heavy={bottom_heavy} '  
        # )

        return right_bias and touches_right and wide_extent and bottom_heavy

    # ------------------------------------------------------------------
    # Right turn  (FIX 1: non-blocking state machine)
    # ------------------------------------------------------------------

    def start_right_turn(self):
        extra = 0.25 * (0.275 - self.forward_velocity) / 0.035
        self.current_turn_time = self.turn_time + extra
        self.get_logger().info(
            f'RIGHT CORNER — starting turn (turn #{self.num_turns + 1}), '
            f'turn_time={self.current_turn_time:.2f}s (base={self.turn_time:.2f}s, extra={extra:.2f}s)'
        )
        # if self.num_turns > 0:
        #     self.forward_velocity += 0.10
        self.turning    = True
        self.turn_start = self.get_clock().now()
        self.integral        = 0.0
        self.prev_error      = 0.0
        self.derivative      = 0.0
        self.last_seen_error = 0.0
        self.lost_count      = 0
        self.last_time       = None
        self.corner_streak   = 0

    def continue_turn(self):
        """Called each frame while self.turning is True.
        Returns True while still turning, False when done.
        """
        elapsed = (self.get_clock().now() - self.turn_start).nanoseconds / 1e9
        if elapsed < self.turn_time:
            twist           = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.twist.linear.x  = self.forward_velocity * 0.9
            twist.twist.angular.z = float(self.max_angular)
            self.cmd_pub.publish(twist)
            self.get_logger().info(f'TURNING {elapsed:.2f}/{self.current_turn_time:.2f}s')
            return True
        else:
            # Turn complete
            self.turning = False
            self.num_turns     += 1
            self.last_turn_time = self.get_clock().now()
            twist           = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.twist.linear.x  = self.forward_velocity  # start slow, PID ramps up naturally
            twist.twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info(f'Turn complete (total turns: {self.num_turns})')
            return False

    # ------------------------------------------------------------------
    # Image callback
    # ------------------------------------------------------------------

    def image_callback(self, msg):
        # Convert image
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1
        )
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Crop to bottom half
        frame = frame[frame.shape[0] // 2:, :]

        # Detect blue line
        rgb         = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        blue        = np.array([0, 153, 255], dtype=np.float32)
        diff        = rgb.astype(np.float32) - blue
        binary_mask = (np.linalg.norm(diff, axis=2) < 150).astype(np.uint8)

        # --- FIX 1: service the turn state machine first ---
        if self.turning:
            self.continue_turn()
            return

        # --- Corner detection ---
        corner_detected = self.detect_right_corner(binary_mask)

        if self.last_turn_time is not None:
            cooldown_elapsed = (
                self.get_clock().now() - self.last_turn_time
            ).nanoseconds / 1e9
            in_cooldown = cooldown_elapsed < self.turn_cooldown_sec
        else:
            cooldown_elapsed = 0.0
            in_cooldown      = False

        if in_cooldown:
            self.corner_streak = 0
            remaining = self.turn_cooldown_sec - cooldown_elapsed
            # self.get_logger().info(
            #     f'[CORNER] cooldown active, {remaining:.1f}s remaining'
            # )
        elif corner_detected:
            self.corner_streak += 1
            if self.corner_streak == 1:
                self.turn_cooldown_sec = 37.5
            elif self.corner_streak == 2:
                self.turn_cooldown_sec = 25.0
        else:
            self.corner_streak = 0

        if self.corner_streak >= self.corner_debounce_frames:
            if self.num_turns < 3:
                self.start_right_turn()
            return

        # --- Grid ---
        blue_grid = self.get_blue_grid(binary_mask, self.grid_n)

        xs, ys = [], []
        for row in range(self.grid_n):
            for col in range(self.grid_n):
                if blue_grid[row, col]:
                    cell_h = frame.shape[0] // self.grid_n
                    cell_w = frame.shape[1] // self.grid_n
                    cx     = col * cell_w + cell_w // 2
                    cy     = row * cell_h + cell_h // 2
                    xs.append(frame.shape[0] // 2 - cy)
                    ys.append(cx - frame.shape[1] // 2)

        # --- Error computation ---
        if len(ys) > 0:
            avg_y      = float(np.mean(ys))
            self.error = float(np.clip(
                avg_y / (frame.shape[1] / 2.0), -1.0, 1.0
            ))
            # Reset integral and derivative spike on reacquisition
            if self.lost_count > 0:
                self.integral   = 0.0
                self.prev_error = self.error
            self.lost_count      = 0
            self.last_seen_error = self.error

        else:
            self.lost_count += 1
            ramp      = min(self.lost_count, self.max_lost_ramp)
            direction = np.sign(self.last_seen_error) if self.last_seen_error != 0 else 1.0

            if abs(self.last_seen_error) <= 0.425:
                # Phase 1 — known gap: drive straight and wait for line to reappear
                twist           = TwistStamped()
                twist.header.stamp = self.get_clock().now().to_msg()
                twist.twist.linear.x  = self.forward_velocity
                twist.twist.angular.z = 0.1
                self.cmd_pub.publish(twist)
                return
            else:
                # Lost — bypass PID and command angular directly
                # Spin fast in last known direction, slow linear to maintain control
                twist           = TwistStamped()
                twist.header.stamp = self.get_clock().now().to_msg()
                twist.twist.linear.x  = self.min_linear
                twist.twist.angular.z = float(direction * self.max_angular * 0.55)
                self.cmd_pub.publish(twist)
                return

        # --- Timing ---
        now            = self.get_clock().now().nanoseconds / 1e9
        dt             = (now - self.last_time) if self.last_time else 0.1
        dt             = max(dt, 1e-3)
        self.last_time = now

        # --- PID ---
        if self.lost_count == 0:
            self.integral = float(np.clip(
                self.integral + self.error * dt, -1.0, 1.0
            ))

        raw_derivative = float(np.clip(
            (self.error - self.prev_error) / dt, -5.0, 5.0
        ))
        alpha           = 0.7
        self.derivative = alpha * self.derivative + (1 - alpha) * raw_derivative

        control = (
            self.kp * self.error +
            self.ki * self.integral +
            self.kd * self.derivative
        )
        self.prev_error = self.error

        # --- Velocity commands ---
        angular_z = float(np.clip(
            self.angular_velocity * control,
            -self.max_angular,
            self.max_angular
        ))

        # FIX 3: stronger speed reduction on large errors (curves/turns)
        # Use a squared falloff so speed drops aggressively on sharp turns
        speed_scale = 1.0 - 0.75 * abs(self.error)
        linear_x    = float(np.clip(
            self.forward_velocity * speed_scale,
            self.min_linear,
            self.max_linear
        ))

        twist           = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x  = linear_x
        twist.twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f'err={self.error:.3f} streak={self.corner_streak} '
            f'cmd=(lin={linear_x:.2f}, ang={angular_z:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
