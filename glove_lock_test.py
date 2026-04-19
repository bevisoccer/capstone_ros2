#!/usr/bin/env python3
"""Press B to lock glove servos at current finger position. Press F to free. Q to quit."""
import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GloveLockTester(Node):
    def __init__(self):
        super().__init__("glove_lock_tester")
        self._pub = self.create_publisher(String, "/glove/command", 10)
        print("Glove lock tester ready.")
        print("  B — lock servos at current position")
        print("  F — free servos")
        print("  Q — quit")

    def send(self, cmd):
        msg = String()
        msg.data = cmd
        self._pub.publish(msg)
        print(f"  >> {cmd}")


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = GloveLockTester()
    try:
        while rclpy.ok():
            key = get_key().lower()
            if key == 'b':
                node.send('lock')
            elif key == 'f':
                node.send('free')
            elif key in ('q', '\x03'):
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
