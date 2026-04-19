#!/usr/bin/env python3
"""
Wrist keyboard controller — reads from the terminal it is run in.

  W  or  UP   arrow → wrist up   (M5 pitches up)
  S  or  DOWN arrow → wrist down (M5 pitches down)
  P              → toggle arm pause / resume
  O              → toggle hand pause / resume
  L              → toggle hand ESTOP / RESUME_ALL (servos limp / re-enabled)
  X              → immediately cut arm motor power
  Q              → quit (parks arm)
"""

import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


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


class WristKeyboardNode(Node):
    def __init__(self):
        super().__init__('wrist_keyboard_node')
        self.arm_pub  = self.create_publisher(String, '/arm_control_command', 10)
        self.hand_pub = self.create_publisher(String, '/hand/command', 10)
        self._arm_paused  = False
        self._hand_paused = False
        self._hand_estop  = False
        print('\n=== Wrist / Hand Control ===')
        print('  W / UP   arrow → wrist up')
        print('  S / DOWN arrow → wrist down')
        print('  P              → toggle arm pause / resume')
        print('  O              → toggle hand pause / resume')
        print('  L              → toggle hand ESTOP / RESUME_ALL')
        print('  X              → cut arm motor power immediately')
        print('  Q              → quit (parks arm)')
        print('============================\n')

    def _send_arm(self, cmd):
        msg = String(); msg.data = cmd
        self.arm_pub.publish(msg)
        print(f'  [ARM]  {cmd}')

    def _send_hand(self, cmd):
        msg = String(); msg.data = cmd
        self.hand_pub.publish(msg)
        print(f'  [HAND] {cmd}')

    def run(self):
        while rclpy.ok():
            key = read_key()
            if key in (b'w', b'W', b'\x1b[A'):
                self._send_arm('wrist_up')
            elif key in (b's', b'S', b'\x1b[B'):
                self._send_arm('wrist_down')
            elif key in (b'p', b'P'):
                if self._arm_paused:
                    self._arm_paused = False
                    self._send_arm('resume')
                else:
                    self._arm_paused = True
                    self._send_arm('pause')
            elif key in (b'o', b'O'):
                if self._hand_paused:
                    self._hand_paused = False
                    self._send_hand('resume')
                else:
                    self._hand_paused = True
                    self._send_hand('pause')
            elif key in (b'l', b'L'):
                if self._hand_estop:
                    self._hand_estop = False
                    self._send_hand('resume_all')
                else:
                    self._hand_estop = True
                    self._send_hand('estop')
            elif key in (b'x', b'X'):
                self._send_arm('stop')
            elif key in (b'q', b'Q', b'\x03'):
                print('  Parking arm and shutting down...')
                self._send_arm('quit')
                break


def main(args=None):
    rclpy.init(args=args)
    node = WristKeyboardNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
