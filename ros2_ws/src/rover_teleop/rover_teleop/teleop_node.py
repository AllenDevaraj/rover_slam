#!/usr/bin/env python3
import sys
import select
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


_KB_BUF = ""

def read_available_keys() -> list[str]:
    """
    Read stdin without blocking, buffering bytes across calls so arrow-key
    escape sequences can be parsed even if split across reads/timer ticks.
    Returns: 'UP', 'DOWN', 'LEFT', 'RIGHT', ' ', 'q', etc.
    """
    global _KB_BUF
    events: list[str] = []

    # Pull any available bytes into the buffer
    while True:
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not rlist:
            break
        ch = sys.stdin.read(1)
        if not ch:
            break
        _KB_BUF += ch

    # Parse buffer
    i = 0
    while i < len(_KB_BUF):
        c = _KB_BUF[i]

        # Ctrl+C in raw/cbreak comes as ETX
        if c == "\x03":
            events.append("\x03")
            i += 1
            continue

        # Arrow keys: ESC [ A/B/C/D or ESC O A/B/C/D
        if c == "\x1b":
            if i + 2 >= len(_KB_BUF):
                break  # not enough bytes yet; wait for next call
            c2 = _KB_BUF[i + 1]
            c3 = _KB_BUF[i + 2]
            if c2 in ("[", "O") and c3 in ("A", "B", "C", "D"):
                events.append({"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT"}[c3])
                i += 3
                continue

            # Unknown ESC sequence; skip just the ESC and keep scanning
            i += 1
            continue

        # Normal single-char keys
        events.append(c)
        i += 1

    # Keep any remaining unparsed bytes (partial escape sequence)
    _KB_BUF = _KB_BUF[i:]
    return events



class RoverTeleop(Node):
    def __init__(self):
        super().__init__("rover_teleop")
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Tunables
        self.linear_speed = 0.25    # m/s
        self.angular_speed = 5.0   # rad/s
        self.publish_hz = 30.0
        self.hold_timeout = 0.8

        # Track last time each key was seen
        now = time.time()
        self.last_seen = {k: 0.0 for k in ["UP", "DOWN", "LEFT", "RIGHT", " "]}

        self.timer = self.create_timer(1.0 / self.publish_hz, self.control_loop)

        self.get_logger().info(
            "Combined teleop:\n"
            "  Forward Arrow/Back Arrow = forward/back\n"
            "  Left Arrow/Right Arrow = left/right turn\n"
            "  Q = quit"
        )

    def key_active(self, k: str, now: float) -> bool:
        return (now - self.last_seen.get(k, 0.0)) <= self.hold_timeout

    def control_loop(self):
        now = time.time()

        # Read all keys pressed since last tick
        for key in read_available_keys():
            if key == "q" or (isinstance(key, str) and key.lower() == "q"):
                self.publish_stop()
                rclpy.shutdown()
                return

            if key in self.last_seen:
                self.last_seen[key] = now
            elif len(key) == 1 and key.lower() in self.last_seen:
                self.last_seen[key.lower()] = now

        # SPACE = immediate stop (and clears motion)
        if self.key_active(" ", now):
            self.publish_stop()
            return

        # Build combined command
        forward = self.key_active("UP", now)
        back    = self.key_active("DOWN", now)
        left    = self.key_active("LEFT", now)
        right   = self.key_active("RIGHT", now)


        v = 0.0
        w = 0.0

        # linear: W and S cancel if both "held"
        if forward and not back:
            v = self.linear_speed
            w = 0 
        elif back and not forward:
            v = -self.linear_speed
            w = 0

        # angular: A and D cancel if both "held"
        if left and not right:
            v = self.linear_speed
            w = -self.angular_speed
        elif right and not left:
            v = self.linear_speed
            w = self.angular_speed

        self.publish_cmd(v, w)

    def publish_cmd(self, v: float, w: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def publish_stop(self):
        self.publish_cmd(0.0, 0.0)


def main():
    if not sys.stdin.isatty():
        print("stdin is not a TTY (no interactive keyboard input). Run in a real terminal, not via ros2 launch.")
    # Terminal raw mode for single key reads
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        rclpy.init()
        node = RoverTeleop()
        rclpy.spin(node)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        try:
            node.publish_stop()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
