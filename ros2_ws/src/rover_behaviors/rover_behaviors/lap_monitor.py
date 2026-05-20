from nav_msgs.msg import Odometry
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool

class LoopClosureMonitor(Node):
    def __init__(self):
        super().__init__('loop_closure_monitor')
        
        self.loop_closed = False
        self.last_x = None
        self.last_y = None
        self.last_theta = None

        self.jump_threshold = 0.3

        self.min_distance_traveled = 3.0
        self.distance_traveled = 0.0

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.twist_pub = self.create_publisher(TwistStamped, '/cmd_vel', 1)
        self.done_pub = self.create_publisher(Bool, 'course_complete')

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_x is None:
           self.last_x = x
           self.last_y = y
           return
        step = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
        self.distance_traveled += step
        if (self.distance_traveled > self.min_distance_traveled and not self.loop_closed):
            if step > self.jump_threshold:
                self.loop_closed = True
                self.get_logger().info(
                    f'Loop closure detected — pose jumped {step:.3f}m '
                    f'after {self.distance_traveled:.2f}m traveled'
                )
                self.stop_robot()
        self.last_x = x
        self.last_y = y
    
    def stop_robot(self):
        done_msg = Bool()
        done_msg.data = True
        self.done_pub.publish(done_msg)
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = 0.0
        twist.twist.angular.z = 0.0
        self.twist_pub.publish(twist)
        
    def main(args=None):
        rclpy.init(args=args)
        node = LoopClosureMonitor()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()


    if __name__ == '__main__':
        main()