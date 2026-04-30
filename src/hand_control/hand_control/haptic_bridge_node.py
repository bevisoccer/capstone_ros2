#!/usr/bin/env python3
"""
haptic_bridge_node.py — Bridges the haptic glove and the robot hand.

Haptic feedback path  (when haptics_enabled=true):
  /hand/fsr_pressed   (Int32MultiArray, 0|1 per finger — from Teensy threshold)
      pressed=1  →  publish 1 to /glove/lock_cmd for that finger
      pressed=0  →  publish 0 to /glove/lock_cmd (glove_node frees the servo)

  glove_node computes the servo command via lock_at_current_position_per_finger(),
  which maps the current pot reading to the servo value that holds the string in place.

  Modes:
    per_finger  — only pressed fingers lock
    all_fingers — any press locks all fingers

  4-finger cap (per_finger mode only):
    At most MAX_LOCKED_FINGERS fingers locked simultaneously.

Thresholding is done on the Teensy (FSRx_THRESHOLD in firmware) — tune it there.
Finger open/closed calibration is done in glove_node on BLE connect.
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String

FINGER_NAMES       = ['thumb', 'index', 'middle', 'ring', 'pinky']
MAX_LOCKED_FINGERS = 4
MODE_PER_FINGER    = 'per_finger'
MODE_ALL_FINGERS   = 'all_fingers'


class HapticBridgeNode(Node):
    def __init__(self):
        super().__init__('haptic_bridge_node')

        self.declare_parameter('haptics_enabled', False)
        self.declare_parameter('haptic_rate_hz',  20.0)
        self.declare_parameter('haptic_mode',     MODE_ALL_FINGERS)

        self._haptics_enabled = self.get_parameter('haptics_enabled').value

        self._pub_haptic    = self.create_publisher(Int32MultiArray, '/glove/lock_cmd',  10)
        self._pub_glove_cmd = self.create_publisher(String,          '/glove/command',   10)

        self._sub_pressed = self.create_subscription(
            Int32MultiArray, '/hand/fsr_pressed', self._pressed_cb, 10)

        self._lock           = threading.Lock()
        self._latest_pressed = [0] * 5

        haptic_period = 1.0 / self.get_parameter('haptic_rate_hz').value
        self.create_timer(haptic_period, self._haptic_cb)

        self._startup_timer = None
        if self._haptics_enabled:
            self._startup_timer = self.create_timer(1.0, self._send_haptics_on)

        self.get_logger().info(
            f'HapticBridgeNode started  '
            f'haptics_enabled={self._haptics_enabled}  '
            f'haptic_rate_hz={1.0/haptic_period:.0f}')

    def _send_haptics_on(self):
        msg = String(); msg.data = 'haptics_on'
        self._pub_glove_cmd.publish(msg)
        self.get_logger().info('[BRIDGE] Sent haptics_on to glove_node.')
        if self._startup_timer is not None:
            self._startup_timer.cancel()
            self._startup_timer = None

    # ── Haptic path: Teensy pressed → glove servo ─────────────────────────────

    def _pressed_cb(self, msg: Int32MultiArray):
        if len(msg.data) < 5:
            return
        with self._lock:
            self._latest_pressed = list(msg.data[:5])

    def _haptic_cb(self):
        if not self._haptics_enabled:
            return
        with self._lock:
            pressed = list(self._latest_pressed)

        mode        = self.get_parameter('haptic_mode').value
        any_pressed = any(pressed)

        if mode == MODE_ALL_FINGERS:
            lock_flags = [1 if any_pressed else 0] * 5
        else:
            lock_flags = [1 if pressed[i] else 0 for i in range(5)]
            locked = [i for i, f in enumerate(lock_flags) if f]
            while len(locked) > MAX_LOCKED_FINGERS:
                lock_flags[locked.pop()] = 0

        out = Int32MultiArray()
        out.data = lock_flags
        self._pub_haptic.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = HapticBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
