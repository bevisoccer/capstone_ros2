#!/usr/bin/env python3
"""
Finger curl visualiser — subscribes to /glove/finger_pct and draws live bars.

Run:
  source ~/ros2_ws/install/setup.bash
  DISPLAY=:1 python3 ~/ros2_ws/glove_viz.py
"""

import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np

FINGER_NAMES = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']

BAR_W      = 80
BAR_H      = 300
PADDING    = 30
TOP        = 60
WIN_W      = (BAR_W + PADDING) * 5 + PADDING
WIN_H      = TOP + BAR_H + 60

COLOR_BG   = (30,  30,  30)
COLOR_BAR  = (60, 180, 75)
COLOR_EMPTY= (70,  70,  70)
COLOR_TEXT = (220, 220, 220)


class VizNode(Node):
    def __init__(self):
        super().__init__('glove_viz')
        self.values = [0.0] * 5
        self.lock    = threading.Lock()
        self.create_subscription(
            Float32MultiArray, '/glove/finger_pct', self._cb, 10)
        self.get_logger().info('Subscribed to /glove/finger_pct')

    def _cb(self, msg):
        with self.lock:
            self.values = list(msg.data[:5]) if len(msg.data) >= 5 else self.values

    def get_values(self):
        with self.lock:
            return list(self.values)


def draw_frame(values):
    img = np.full((WIN_H, WIN_W, 3), COLOR_BG, dtype=np.uint8)

    cv2.putText(img, 'Finger Curl', (PADDING, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, COLOR_TEXT, 2)

    for i, (name, pct) in enumerate(zip(FINGER_NAMES, values)):
        pct   = max(0.0, min(1.0, pct))
        x     = PADDING + i * (BAR_W + PADDING)
        bot   = TOP + BAR_H
        fill  = int(BAR_H * pct)

        # Empty bar
        cv2.rectangle(img, (x, TOP), (x + BAR_W, bot), COLOR_EMPTY, -1)

        # Filled portion (grows upward from bottom)
        if fill > 0:
            cv2.rectangle(img, (x, bot - fill), (x + BAR_W, bot), COLOR_BAR, -1)

        # Percentage text inside bar
        pct_str = f'{int(pct * 100)}%'
        (tw, th), _ = cv2.getTextSize(pct_str, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        tx = x + (BAR_W - tw) // 2
        ty = bot - fill - 6 if fill < BAR_H - 20 else bot - fill + th + 4
        cv2.putText(img, pct_str, (tx, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_TEXT, 1)

        # Finger name below bar
        (nw, _), _ = cv2.getTextSize(name, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        cv2.putText(img, name, (x + (BAR_W - nw) // 2, bot + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_TEXT, 1)

    return img


def main():
    rclpy.init()
    node = VizNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    cv2.namedWindow('Finger Curl', cv2.WINDOW_NORMAL)

    while True:
        values = node.get_values()
        img    = draw_frame(values)
        cv2.imshow('Finger Curl', img)
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
