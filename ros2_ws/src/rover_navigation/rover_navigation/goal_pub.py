import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from rclpy.qos import QoSProfile, ReliabilityPolicy




class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_pub')

        # Create publisher
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pub = self.create_publisher(PoseStamped, '/goal_pose', qos)
        # self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Publish at 2 Hz (every 0.5 seconds)
        self.timer = self.create_timer(0.5, self.publish_goal)

    def publish_goal(self):
        msg = PoseStamped()

        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Position
        # msg.pose.position.x = -2.635
        # msg.pose.position.y = 0.086
        # msg.pose.position.z = 0.0

        msg.pose.position.x = -6.096  # 20 ft directly behind home (0,0) in -x direction
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0

        # Orientation
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.pub.publish(msg)
        self.get_logger().info('Publishing goal_pose')


def main():
    rclpy.init()
    node = GoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()