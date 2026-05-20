import rclpy
import numpy as np 
from simple_pid import PID
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 
from std_msgs.msg import Float32


class LidarWallStop(Node):

    def __init__(self):
        super().__init__('LidarWallStop')
        #attributes 
        self.count = 0
        self.front_dist = np.zeros((10, 1)) 
        self.cone_scan = 0 
        self.wall_dist = np.inf
        self.stop_dist = 1.0 
        self.max_linear = 0.25       
        self.min_linear = 0.0              
        self.stop_count = 0
        self.stop_threshold = 10   

        self.cmd_pub = self.create_publisher(
            Twist, 
            'cmd_vel',
              10)
        self.debug_scan_pub = self.create_publisher(
            LaserScan, 
            'cone_scan',
            10)
        
        self.wall_dist_pub = self.create_publisher(
            Float32, 
            'wall_dist',
            10)


        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):


        #get total size of ranges 
        rsize =np.size(msg.ranges)
       
        #Determine front of robot 
        #Angles in radians 

        front_idx = int((0.0 - msg.angle_min) / msg.angle_increment)
        # self.cone_scan = np.array(msg.ranges)[front_idx-5:front_idx+5]
        # cone_cloud = LaserScan()
        # cone_cloud.header.frame_id = "laser_frame"
        # cone_cloud = msg
        # cone_cloud.ranges = []
        # cone_cloud.ranges = self.cone_scan
        # self.debug_scan_pub.publish(cone_cloud)

        #data index in front of robot ~180 
        #5 degree cone both sides of front 
        self.front_dist = np.array(msg.ranges)[front_idx-5:front_idx+5]
        
       

        #save range data 
        if self.count < 5: 
           self.count += 1 
        else:
            # for i in range(dist_size):
            #     self.init_dist[i] = np.inf 
            #     # Clean data: Replace 'inf' with max range
            #     if front_dist[i] == np.inf:
            #         continue 
            #     # Clean data: Replace 'nan' with 0.0 or a safe value
            #     if front_dist[i] == np.nan:
            #         continue 
            #     else : 
            #         self.init_dist[i] = front_dist[i]

            self.front_dist[np.isnan(self.front_dist)] = np.inf 
            self.wall_dist = np.min(self.front_dist)

            #from 5 scans use the minimum distance 
            msg_wall_dist = Float32()
            msg_wall_dist.data = float(self.wall_dist)
            self.wall_dist_pub.publish(msg_wall_dist) 

        self.get_logger().info(f'Front Dist: {self.wall_dist}"')

def dist_data(data):
    #weighted average 
    sumd = data.sum()

    average = sumd/data.size

    return average 
        

def main(args=None):
    rclpy.init(args=args)
    node = LidarWallStop()
    
    rclpy.spin(node)

    #clean up 
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
