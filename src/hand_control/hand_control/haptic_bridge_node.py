#!/usr/bin/env python3
"""
haptic_bridge_node.py — Bridges the haptic glove and the robot hand.

Forward path  (always active):
  /glove/finger_pct   (Float32MultiArray, 0.0–1.0)
      × finger_gain   →  /hand/finger_cmd  (Float32MultiArray, 0.0–1.0)

Haptic feedback path  (when haptics_enabled=true):
  /hand/fsr_pressed   (Int32MultiArray, 0|1 per finger — from Teensy threshold)
      pressed=1  →  lock glove servo at current finger position
      pressed=0  →  free glove servo (1000 = no resistance)

  4-finger cap:
    At most MAX_LOCKED_FINGERS fingers locked simultaneously.
    If more would be locked, the extras are freed (arbitrary tie-break).

  Lock mapping:
    glove_pct = 0.0 (open)    → servo_cmd = 1000 (free)
    glove_pct = 1.0 (closed)  → servo_cmd =    0 (taut)

Thresholding is done on the Teensy (FSRx_THRESHOLD in firmware) — tune it there.
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String

FINGER_NAMES = ['thumb', 'index', 'middle', 'ring', 'pinky']

MAX_LOCKED_FINGERS = 4


class HapticBridgeNode(Node):
    def __init__(self):
        super().__init__('haptic_bridge_node')

        self.declare_parameter('finger_gain',     2.5)
        self.declare_parameter('haptics_enabled', False)
        self.declare_parameter('haptic_rate_hz',  20.0)

        self._gain            = self.get_parameter('finger_gain').value
        self._haptics_enabled = self.get_parameter('haptics_enabled').value

        self._pub_hand      = self.create_publisher(Float32MultiArray, '/hand/finger_cmd', 10)
        self._pub_haptic    = self.create_publisher(Int32MultiArray,   '/glove/servo_cmd', 10)
        self._pub_glove_cmd = self.create_publisher(String,            '/glove/command',   10)

        self._sub_glove   = self.create_subscription(
            Float32MultiArray, '/glove/finger_pct',  self._glove_cb,   10)
        self._sub_pressed = self.create_subscription(
            Int32MultiArray,   '/hand/fsr_pressed',  self._pressed_cb, 10)

        self._lock             = threading.Lock()
        self._latest_glove_pct = [0.0] * 5
        self._latest_pressed   = [0]   * 5

        haptic_period = 1.0 / self.get_parameter('haptic_rate_hz').value
        self.create_timer(haptic_period, self._haptic_cb)

        # One-shot timer: tell glove_node to accept servo commands after BLE connects.
        self._startup_timer = None
        if self._haptics_enabled:
            self._startup_timer = self.create_timer(1.0, self._send_haptics_on)

        self.get_logger().info(
            f'HapticBridgeNode started  finger_gain={self._gain}  '
            f'haptics_enabled={self._haptics_enabled}  '
            f'haptic_rate_hz={1.0/haptic_period:.0f}')

    def _send_haptics_on(self):
        msg = String(); msg.data = 'haptics_on'
        self._pub_glove_cmd.publish(msg)
        self.get_logger().info('[BRIDGE] Sent haptics_on to glove_node.')
        if self._startup_timer is not None:
            self._startup_timer.cancel()
            self._startup_timer = None

    # ── Forward path: glove pots → robot hand ─────────────────────────────────

    def _glove_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 5:
            return
        with self._lock:
            self._latest_glove_pct = list(msg.data[:5])
        out = Float32MultiArray()
        out.data = [max(0.0, min(1.0, float(v) * self._gain)) for v in msg.data[:5]]
        self._pub_hand.publish(out)

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
            glove_pct = list(self._latest_glove_pct)
            pressed   = list(self._latest_pressed)

        cmds = []
        for i in range(5):
            if pressed[i]:
                cmds.append(max(0, min(1000, int((1.0 - glove_pct[i]) * 1000))))
            else:
                cmds.append(1000)

        # Enforce cap: free extras if too many fingers locked
        locked = [i for i, c in enumerate(cmds) if c < 1000]
        while len(locked) > MAX_LOCKED_FINGERS:
            cmds[locked.pop()] = 1000

        out = Int32MultiArray()
        out.data = cmds
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
