#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AutoLidarNode(Node):
    def __init__(self):
        # This is the name that shows up in 'ros2 node list'
        super().__init__('auto_lidar_node')
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.get_logger().info("Auto Lidar Node has started")

        def lidar_callback(self, msg):
        # 360-degree mapping (assuming 180 is front)
        front_zone = msg.ranges[150:210]
        right_zone = msg.ranges[30:150]
        left_zone  = msg.ranges[210:330]

        def get_min_dist(zone):
            valid_ranges = [r for r in zone if r > 0.1]
            return min(valid_ranges) if valid_ranges else float('inf')

        dist_front = get_min_dist(front_zone)
        dist_right = get_min_dist(right_zone)
        dist_left  = get_min_dist(left_zone)

        cmd = Twist()
        
        # 1. IMMEDIATE OBSTACLE: If blocked in front, prioritize turning right
        if dist_front < 0.7:
            cmd.linear.x = 0.0
            # Strong bias: Only turn left if right is critically blocked (< 0.4m)
            if dist_right > 0.4:
                cmd.angular.z = -0.8  # Sharp Right Turn
            else:
                cmd.angular.z = 0.8   # Emergency Left Turn
        
        # 2. HALLWAY SEEKING: If moving forward and a right opening appears
        elif dist_right > 1.5:  # Threshold for "I see a hallway"
            cmd.linear.x = 0.1   # Slow down to make a tighter turn
            cmd.angular.z = -0.7  # Strong right turn into the hallway
            self.get_logger().info("Open hallway detected! Turning right.")
            
        # 3. NORMAL PATH: Move forward
        else:
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
            
        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AutoLidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
