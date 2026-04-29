#!/usr/bin/env python3
"""
glove_haptic_test.py — Interactive haptic glove test tool.

Publishes to /glove/servo_cmd  (Int32MultiArray, 5 values 0-1000)
  0    = free (open)
  1000 = block (closed / full resistance)

Keys:
  1-5   toggle individual finger  (1=thumb, 2=index, 3=middle, 4=ring, 5=pinky)
  B     block ALL fingers
  F     free  ALL fingers
  Q     quit

Run:
  source ~/ros2_ws/install/setup.bash
  python3 ~/ros2_ws/glove_haptic_test.py
"""

import sys
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
BLOCK = 0
FREE  = 1000


def read_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.buffer.read(1)
        if ch == b'\x1b':
            ch += sys.stdin.buffer.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class HapticTestNode(Node):
    def __init__(self):
        super().__init__('haptic_test_node')
        self._pub = self.create_publisher(Int32MultiArray, '/glove/servo_cmd', 10)
        self._values = [FREE] * 5

    def _send(self):
        msg = Int32MultiArray()
        msg.data = list(self._values)
        self._pub.publish(msg)

    def block_all(self):
        self._values = [BLOCK] * 5
        self._send()
        self._print_state()

    def free_all(self):
        self._values = [FREE] * 5
        self._send()
        self._print_state()

    def toggle_finger(self, idx):
        self._values[idx] = FREE if self._values[idx] == BLOCK else BLOCK
        self._send()
        self._print_state()

    def _print_state(self):
        parts = []
        for i, name in enumerate(FINGER_NAMES):
            state = "BLOCK" if self._values[i] == BLOCK else "free "
            parts.append(f"{name}:{state}")
        print("  " + "  ".join(parts))

    def run(self):
        print("\n=== Haptic Glove Test ===")
        print("  1-5  toggle finger  (1=Thumb 2=Index 3=Middle 4=Ring 5=Pinky)")
        print("  B    block ALL")
        print("  F    free  ALL")
        print("  Q    quit")
        print("========================")
        self._print_state()

        while rclpy.ok():
            key = read_key()
            if key in (b'1',):
                self.toggle_finger(0)
            elif key in (b'2',):
                self.toggle_finger(1)
            elif key in (b'3',):
                self.toggle_finger(2)
            elif key in (b'4',):
                self.toggle_finger(3)
            elif key in (b'5',):
                self.toggle_finger(4)
            elif key in (b'b', b'B'):
                self.block_all()
            elif key in (b'f', b'F'):
                self.free_all()
            elif key in (b'q', b'Q', b'\x03'):
                break

        # Release all on exit
        self.free_all()


def main():
    rclpy.init()
    node = HapticTestNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
