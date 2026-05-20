#!/home/danitns/Documents/autonomous-robot/pi-env/bin/python3
import time
from math import atan2, sqrt
from math import pi as PI

from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

orientation_quat = [0,0,0,1] # x, y, z, w (real)
orientation_rad = [0,0,0] # roll, pitch, yaw(heading)  (in rad)
orientation_deg = [0,0,0] # roll, pitch, yaw(heading)  (in deg)

linear_accel = [0,0,0] # x, y, z  (in m/s^2)
gyro = [0,0,0] # x, y, z  (in deg/s)


class BNO085Node(Node):
    def __init__(self):
        super().__init__("bno085_node")
        self.imu_publisher = self.create_publisher(Imu, "imu/data", 30)

        self.imu = None
        self.init_sensor()
        self.read_send_timer = self.create_timer(0.03, self.publish_sensor_data)

    def init_sensor(self):
        i2c = I2C(1)
        try:
            self.imu = BNO08X_I2C(i2c)
        except:
            self.get_logger().error('Failed to connect to BNO085 via I2C...')
            raise Exception('Failed to connect to BNO085 via I2')
            
        self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR) # orientation_quat data
        self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION) # linear acceleration data
        self.imu.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE) # angular velocity data
        time.sleep(1)

    def publish_sensor_data(self):
        try:
            gyro[0], gyro[1], gyro[2] = self.imu.gyro 
            linear_accel[0], linear_accel[1], linear_accel[2] = self.imu.linear_acceleration  
            orientation_quat[0], orientation_quat[1], orientation_quat[2], orientation_quat[3] = self.imu.quaternion

            now = self.get_clock().now()
            imu_data_msg = Imu()
            imu_data_msg.header.stamp = now.to_msg()
            imu_data_msg.header.frame_id = "imu_link"

            imu_data_msg.angular_velocity.x = gyro[1]
            imu_data_msg.angular_velocity.y = -gyro[0]
            imu_data_msg.angular_velocity.z = gyro[2]

            imu_data_msg.linear_acceleration.x = linear_accel[1]
            imu_data_msg.linear_acceleration.y = -linear_accel[0]
            imu_data_msg.linear_acceleration.z = linear_accel[2]
            
            imu_data_msg.orientation.x = orientation_quat[1]
            imu_data_msg.orientation.y = -orientation_quat[0]
            imu_data_msg.orientation.z = orientation_quat[2]
            imu_data_msg.orientation.w = orientation_quat[3]


            imu_data_msg.orientation_covariance[0] = 0.01
            imu_data_msg.orientation_covariance[4] = 0.01
            imu_data_msg.orientation_covariance[8] = 0.01
            imu_data_msg.angular_velocity_covariance[0] = 0.01
            imu_data_msg.angular_velocity_covariance[4] = 0.01
            imu_data_msg.angular_velocity_covariance[8] = 0.01
            imu_data_msg.linear_acceleration_covariance[0] = 0.01
            imu_data_msg.linear_acceleration_covariance[4] = 0.01
            imu_data_msg.linear_acceleration_covariance[8] = 0.01

        
            self.imu_publisher.publish(imu_data_msg)

        except OSError as e:
            self.get_logger().warn(f"I2C communication error: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")



def main(args=None):
    
    rclpy.init(args=args)
    node = BNO085Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()