#!/usr/bin/env python3
"""
Pure Pursuit path-following controller.

Subscribes to:
  /plan          (nav_msgs/Path)           — from astar_planner

Publishes:
  /cmd_vel       (geometry_msgs/TwistStamped) — to robo_rover (MAVLink)

Uses TF (map -> base_link) for current pose.
Stops when within GOAL_TOLERANCE_M of the final waypoint.

Sim mode (--ros-args -p use_sim:=true):
  Integrates cmd_vel via bicycle model and publishes a fake map->base_link TF.
  Initialises the simulated pose at the first waypoint, facing the second.
"""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import TwistStamped, TransformStamped

import tf2_ros
import tf2_geometry_msgs  # noqa: F401


# --- Tuning knobs ---
LOOKAHEAD_DISTANCE = 1.0   # metres — larger = smoother turns, less reactive
LINEAR_SPEED = 0.8         # m/s
# LINEAR_SPEED = 0.23         # m/s
ANGULAR_SPEED = 2.2        # m/s
GOAL_TOLERANCE_M = 0.25    # stop when this close to the final waypoint
CONTROL_HZ = 20.0
WHEELBASE = 0.152          # metres — measure and update
MAX_STEER_RAD = math.atan2(2.0 * WHEELBASE, LOOKAHEAD_DISTANCE)

TURN_SPEED_REDUCTION = 0.5  # fraction of LINEAR_SPEED shed at a 90-degree turn
# TURN_SPEED_REDUCTION = 0.45  # fraction of LINEAR_SPEED shed at a 90-degree turn
MIN_SPEED_SCALE = 0.34       # floor: never drive slower than this fraction of LINEAR_SPEED
# MIN_SPEED_SCALE = 0.12       # floor: never drive slower than this fraction of LINEAR_SPEED

# Pose must not drift more than STABLE_THRESHOLD_M over STABLE_DURATION_S to activate
STABLE_DURATION_S = 5.0
STABLE_THRESHOLD_M = 0.15

# If TF drops out, keep republishing the last command for this long before stopping
TF_LATCH_S = 0.3


def yaw_from_tf(tf_msg) -> float:
    q = tf_msg.transform.rotation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        self.declare_parameter('use_sim', False)
        self.use_sim: bool = self.get_parameter('use_sim').get_parameter_value().bool_value

        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_cb, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path: list[tuple[float, float]] = []

        # Controller state: stabilizing → active
        self.active = False
        self.stabilizing = False
        self.stable_ref_x: float | None = None
        self.stable_ref_y: float | None = None
        self.stable_start_time: float | None = None

        # Last published command — republished while TF is briefly unavailable
        self.last_cmd: TwistStamped | None = None
        self.last_tf_time: float = 0.0

        self.debug_tick = 0

        # sim state
        self.sim_x = 0.0
        self.sim_y = 0.0
        self.sim_yaw = 0.0
        self.sim_active = False
        self.last_linear = 0.0
        self.last_angular = 0.0

        if self.use_sim:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
            self.create_subscription(TwistStamped, '/cmd_vel', self.sim_cmd_cb, 10)
            self.create_timer(1.0 / CONTROL_HZ, self.sim_loop)
            self.get_logger().info('SIM MODE enabled — publishing fake map->base_link TF')

        self.create_timer(1.0 / CONTROL_HZ, self.control_loop)
        self.get_logger().info('Pure Pursuit controller ready')

    # ------------------------------------------------------------------
    # Path callback
    # ------------------------------------------------------------------

    def plan_cb(self, msg: Path) -> None:
        self.path = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.get_logger().info(f'New path received: {len(self.path)} waypoints')
        for i, (x, y) in enumerate(self.path):
            self.get_logger().info(f'Waypoint {i+1}: x={x:.3f}, y={y:.3f}')

        if len(self.path) == 0:
            return

        self.active = False
        self.stabilizing = True
        self.stable_ref_x = None
        self.stable_ref_y = None
        self.stable_start_time = None
        self.last_cmd = None
        self.get_logger().info(
            f'Waiting for pose to be stable for {STABLE_DURATION_S:.0f}s '
            f'(threshold {STABLE_THRESHOLD_M * 100:.0f} cm)...'
        )

        if self.use_sim and len(self.path) >= 1:
            self.sim_x, self.sim_y = self.path[0]
            if len(self.path) >= 2:
                dx = self.path[1][0] - self.path[0][0]
                dy = self.path[1][1] - self.path[0][1]
                self.sim_yaw = math.atan2(dy, dx)
            else:
                self.sim_yaw = 0.0
            self.last_linear = 0.0
            self.last_angular = 0.0
            self.sim_active = True

    # ------------------------------------------------------------------
    # Pose stability check
    # ------------------------------------------------------------------

    def check_stability(self, rx: float, ry: float) -> None:
        now = self.get_clock().now().nanoseconds / 1e9

        if self.stable_ref_x is None:
            self.stable_ref_x = rx
            self.stable_ref_y = ry
            self.stable_start_time = now
            return

        drift = math.hypot(rx - self.stable_ref_x, ry - self.stable_ref_y)

        if drift > STABLE_THRESHOLD_M:
            self.get_logger().info(
                f'Pose moved {drift * 100:.1f} cm — resetting stability timer'
            )
            self.stable_ref_x = rx
            self.stable_ref_y = ry
            self.stable_start_time = now
            return

        elapsed = now - self.stable_start_time
        self.get_logger().info(
            f'Pose stable: {elapsed:.1f} / {STABLE_DURATION_S:.0f}s',
            throttle_duration_sec=1.0,
        )

        if elapsed >= STABLE_DURATION_S:
            self.stabilizing = False
            self.active = True
            lookahead = self.find_lookahead(rx, ry)
            if lookahead is None:
                lookahead = self.path[-1]
            self.get_logger().info(
                f'Pose stable — starting. '
                f'First target: ({lookahead[0]:.3f}, {lookahead[1]:.3f})'
            )

    # ------------------------------------------------------------------
    # Sim helpers
    # ------------------------------------------------------------------

    def sim_cmd_cb(self, msg: TwistStamped) -> None:
        self.last_linear = msg.twist.linear.x
        self.last_angular = msg.twist.angular.z

    def sim_loop(self) -> None:
        if not self.sim_active:
            return

        dt = 1.0 / CONTROL_HZ
        v = self.last_linear

        if abs(v) > 1e-4:
            delta = -self.last_angular * MAX_STEER_RAD / 2.0
            omega = v * math.tan(delta) / WHEELBASE
            self.sim_x += v * math.cos(self.sim_yaw) * dt
            self.sim_y += v * math.sin(self.sim_yaw) * dt
            self.sim_yaw += omega * dt
            self.sim_yaw = math.atan2(math.sin(self.sim_yaw), math.cos(self.sim_yaw))

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.sim_x
        t.transform.translation.y = self.sim_y
        t.transform.translation.z = 0.0
        cy = math.cos(self.sim_yaw * 0.5)
        sy = math.sin(self.sim_yaw * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------

    def control_loop(self) -> None:
        if not self.stabilizing and not self.active:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        try:
            if not self.tf_buffer.can_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0)
            ):
                # TF temporarily unavailable — keep the last command alive
                if self.active and self.last_cmd is not None:
                    if (now - self.last_tf_time) < TF_LATCH_S:
                        self.cmd_pub.publish(self.last_cmd)
                return
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return

        self.last_tf_time = now
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        ryaw = yaw_from_tf(tf)

        if self.stabilizing:
            self.check_stability(rx, ry)
            return

        self.get_logger().info(f'Pose: ({rx:.3f}, {ry:.3f}) yaw={math.degrees(ryaw):.1f}°')

        # Stop if close enough to the final goal
        gx, gy = self.path[-1]
        if math.hypot(rx - gx, ry - gy) < GOAL_TOLERANCE_M:
            self.get_logger().info('Goal reached — stopping')
            self.active = False
            self.sim_active = False
            self.path = []
            self.last_cmd = None
            stop = TwistStamped()
            stop.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(stop)
            return

        lookahead = self.find_lookahead(rx, ry)
        if lookahead is None:
            lookahead = self.path[-1]

        lx, ly = lookahead
        self.get_logger().info(f'Target: ({lx:.3f}, {ly:.3f})')

        dx = lx - rx
        dy = ly - ry
        local_x = math.cos(-ryaw) * dx - math.sin(-ryaw) * dy
        local_y = math.sin(-ryaw) * dx + math.cos(-ryaw) * dy

        dist = math.hypot(local_x, local_y)
        if dist < 0.01:
            return

        alpha = math.atan2(local_y, local_x)

        self.get_logger().info(
            f'local=({local_x:.3f}, {local_y:.3f}) '
            f'alpha={math.degrees(alpha):.1f}° dist={dist:.3f}',
            throttle_duration_sec=0.5,
        )

        # Lookahead is behind the robot — yaw estimate is probably stale or wrong
        if abs(alpha) > math.pi / 2:
            self.get_logger().warn(
                f'ALPHA GUARD: lookahead is {math.degrees(alpha):.1f}° off — driving straight',
                throttle_duration_sec=1.0,
            )
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.twist.linear.x = LINEAR_SPEED
            cmd.twist.angular.z = 0.0
            self.last_cmd = cmd
            self.cmd_pub.publish(cmd)
            return

        delta = math.atan2(2.0 * WHEELBASE * math.sin(alpha), dist)
        delta = max(-MAX_STEER_RAD, min(MAX_STEER_RAD, delta))

        # rover_node expects angular.z in [-2.0, 2.0] → scales by 500 → MAVLink ±1000
        steering_cmd = -(delta / MAX_STEER_RAD) * ANGULAR_SPEED

        speed_scale = max(MIN_SPEED_SCALE, 1.0 - TURN_SPEED_REDUCTION * (abs(alpha) / (math.pi / 2)))
        linear_speed = LINEAR_SPEED * speed_scale

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = linear_speed
        cmd.twist.angular.z = steering_cmd
        self.debug_tick += 1
        if self.debug_tick % int(CONTROL_HZ) == 0:
            self.get_logger().info(
                f'v={linear_speed:.3f} (scale={speed_scale:.2f})  steer={steering_cmd:.3f}'
            )
        self.last_cmd = cmd
        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------
    # Lookahead search
    # ------------------------------------------------------------------

    def find_lookahead(self, rx: float, ry: float) -> tuple[float, float] | None:
        # Anchor to closest waypoint, then walk forward to find lookahead.
        # Prevents targeting waypoints already behind the robot.
        min_dist = float('inf')
        closest_idx = 0
        for i, (px, py) in enumerate(self.path):
            d = math.hypot(px - rx, py - ry)
            if d < min_dist:
                min_dist = d
                closest_idx = i

        for px, py in self.path[closest_idx:]:
            if math.hypot(px - rx, py - ry) >= LOOKAHEAD_DISTANCE:
                return px, py
        return None


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
