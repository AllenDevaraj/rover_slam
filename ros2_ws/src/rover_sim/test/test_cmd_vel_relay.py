import rclpy
import pytest
from geometry_msgs.msg import Twist, TwistStamped
from rover_sim.cmd_vel_relay import CmdVelRelay


@pytest.fixture(scope='module', autouse=True)
def rclpy_ctx():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_stamped_callback_publishes_inner_twist():
    node = CmdVelRelay()
    captured = []
    node.pub.publish = lambda m: captured.append(m)  # intercept output

    msg = TwistStamped()
    msg.twist.linear.x = 0.5
    msg.twist.angular.z = -0.3
    node._stamped_cb(msg)

    assert len(captured) == 1
    assert isinstance(captured[0], Twist)
    assert captured[0].linear.x == 0.5
    assert captured[0].angular.z == -0.3
    node.destroy_node()


def test_teleop_twist_passthrough():
    node = CmdVelRelay()
    captured = []
    node.pub.publish = lambda m: captured.append(m)

    msg = Twist()
    msg.linear.x = 0.25
    node._twist_cb(msg)

    assert len(captured) == 1
    assert captured[0].linear.x == 0.25
    node.destroy_node()
