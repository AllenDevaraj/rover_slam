import rclpy
import numpy as np 
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class PIDControlNode(Node):
    
    def __init__(self):
        super().__init__('pid_control_node')
        self.kp = 0.5
        self.ki = 0.0
        self.kd = 0.0
        self.prev_dist = 0.0
        self.integral = 0.0

        self.subscription = self.create_subscription(
            Float32,
            'wall_dist',
            self.distance_to_wall_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

    def distance_to_wall_callback(self, msg):
        dist_to_wall = msg.data
        error = dist_to_wall - 1  # Assuming the desired distance is 0
        self.integral += error
        derivative = error - self.prev_dist
        control_signal = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_dist = error

        cmd_msg = Twist()
        cmd_msg.linear.x = max(0.0, min(0.25, control_signal))  # Limit linear speed
        cmd_msg.angular.z = 0.0  # No angular control in this example
        self.cmd_pub.publish(cmd_msg)
        self.get_logger().info(f'Distance to Wall: {dist_to_wall}, Error: {error}, Control Signal: {control_signal}, Cmd Linear Vel: {cmd_msg.linear.x}')

def main(args=None):
    rclpy.init(args=args)
    pid_control_node = PIDControlNode()
    rclpy.spin(pid_control_node)
    pid_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()