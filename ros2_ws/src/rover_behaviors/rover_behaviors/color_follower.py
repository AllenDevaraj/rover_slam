#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorFollowerNode(Node):
    def __init__(self):
        super().__init__('color_follower')

        # HSV range for the neon-green paper    
        self.hsv_lower = np.array([65, 100, 60])
        self.hsv_upper = np.array([95, 255, 255])

        self.frame_w = 640
        self.frame_h = 480
        self.frame_center_x = self.frame_w // 2

        self.min_contour_area = 800

        # Proportional gains
        self.kp_angular = 2.0
        self.target_area = 12000.0   
        self.kp_linear = 0.15        

        self.max_linear = 0.25       
        self.min_linear = 0.0        
        self.max_angular = 3.0       
        self.lost_count = 0
        self.lost_threshold = 10     

        self.bridge = CvBridge()

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info(
            'Color follower started — tracking green paper '
            f'HSV [{self.hsv_lower}] to [{self.hsv_upper}]'
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area >= self.min_contour_area:
                self.lost_count = 0

                M = cv2.moments(largest)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                error_x = (cx - self.frame_center_x) / (self.frame_w / 2.0)
                angular_z = self.kp_angular * error_x

                area_ratio = area / self.target_area
                linear_x = self.kp_linear * (1.0 - area_ratio)

                linear_x = float(np.clip(linear_x, self.min_linear, self.max_linear))
                angular_z = float(np.clip(angular_z, -self.max_angular, self.max_angular))

                twist = Twist()
                twist.linear.x = linear_x * 2.0
                twist.angular.z = angular_z
                self.cmd_pub.publish(twist)

                self.get_logger().info(
                    f'Tracking: centroid=({cx},{cy}) area={area:.0f} '
                    f'cmd=(lin={linear_x:.3f}, ang={angular_z:.3f})',
                    throttle_duration_sec=0.5
                )
                return

        self.lost_count += 1
        if self.lost_count >= self.lost_threshold:
            self.cmd_pub.publish(Twist())  
            self.get_logger().warn(
                'Target lost - stopping rover',
                throttle_duration_sec=2.0
            )
        else:
            self.get_logger().info(
                f'Target not found (lost {self.lost_count}/{self.lost_threshold})',
                throttle_duration_sec=1.0
            )

    def destroy_node(self):   
        self.get_logger().info('Shutting down color follower - sending stop command')
        self.cmd_pub.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColorFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
