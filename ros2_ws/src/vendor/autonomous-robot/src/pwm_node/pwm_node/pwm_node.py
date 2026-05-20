# pwm_node.py
import rclpy
from rclpy.node import Node
import pigpio
import time

class PWMNode(Node):
    def __init__(self):
        super().__init__('pwm_node')
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error('Could not connect to pigpio daemon.')
            rclpy.shutdown()
            return

        PWM_PIN = 18
        FREQ = 10000
        DUTY_CYCLE = 72

        self.pi.hardware_PWM(PWM_PIN, FREQ, int(DUTY_CYCLE * 10000))
        self.get_logger().info(f'Started PWM on GPIO {PWM_PIN} with {FREQ}Hz and {DUTY_CYCLE}% duty cycle')

    def destroy_node(self):
        self.pi.set_mode(18, pigpio.INPUT)
        self.pi.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PWMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
