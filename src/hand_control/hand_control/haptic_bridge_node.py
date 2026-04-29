#!/usr/bin/env python3
"""
haptic_bridge_node.py — Bridges the haptic glove and the robot hand.

Forward path  (always active):
  /glove/finger_pct  (Float32MultiArray, 0.0–1.0)
      × finger_gain  →  /hand/finger_cmd  (Float32MultiArray, 0.0–1.0)

Haptic feedback path  (when haptics_enabled=true):
  /hand/fsr_pct  (Float32MultiArray, 0.0–100.0 %)
      per-finger threshold check  →  /glove/servo_cmd  (Int32MultiArray, 0–1000)

  Per-finger rule:
    FSR[i] > FSR_LOCK_THRESHOLDS[i]  →  lock glove finger i at current position
                                         cmd = (1 - glove_pct[i]) * 1000
    FSR[i] ≤ FSR_LOCK_THRESHOLDS[i]  →  free glove finger i  (cmd = 1000)

  4-finger cap:
    At most MAX_LOCKED_FINGERS fingers may be locked simultaneously.
    If more than MAX_LOCKED_FINGERS would be locked, the finger(s) with the
    lowest FSR reading are freed first until the cap is met.

  Lock mapping (mirrors glove_node.py):
    glove_pct = 0.0 (open)    → servo_cmd = 1000 (free)
    glove_pct = 0.75 (curled) → servo_cmd =  250
    glove_pct = 1.0 (closed)  → servo_cmd =    0 (taut)

The glove_node applies incoming servo_cmd only when its own haptics_on flag is
set (via /glove/command 'haptics_on').
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray

FINGER_NAMES = ['thumb', 'index', 'middle', 'ring', 'pinky']

# ── FSR lock thresholds (0.0 – 100.0 %) ──────────────────────────────────────
# When a finger's FSR reading (on the robot hand) exceeds its threshold, the
# corresponding glove servo locks at the current glove finger position.
# Tune per-finger to match the sensitivity of each FSR sensor.
FSR_LOCK_THRESHOLDS = {
    'thumb':  10.0,   # %
    'index':  10.0,   # %
    'middle': 10.0,   # %
    'ring':   10.0,   # %
    'pinky':  10.0,   # %
}

# ── Maximum simultaneous locked fingers ───────────────────────────────────────
# Never lock all five fingers at once. If more than this many fingers exceed
# their FSR threshold, the finger(s) with the lowest FSR reading are freed
# first until the count is within the cap.
MAX_LOCKED_FINGERS = 4


class HapticBridgeNode(Node):
    def __init__(self):
        super().__init__('haptic_bridge_node')

        self.declare_parameter('finger_gain',          1.5)
        self.declare_parameter('haptics_enabled',      False)
        self.declare_parameter('haptic_rate_hz',       20.0)
        # Per-finger FSR thresholds — tunable at runtime via:
        #   ros2 param set /haptic_bridge_node fsr_threshold_<finger> <value>
        self.declare_parameter('fsr_threshold_thumb',  FSR_LOCK_THRESHOLDS['thumb'])
        self.declare_parameter('fsr_threshold_index',  FSR_LOCK_THRESHOLDS['index'])
        self.declare_parameter('fsr_threshold_middle', FSR_LOCK_THRESHOLDS['middle'])
        self.declare_parameter('fsr_threshold_ring',   FSR_LOCK_THRESHOLDS['ring'])
        self.declare_parameter('fsr_threshold_pinky',  FSR_LOCK_THRESHOLDS['pinky'])

        self._gain            = self.get_parameter('finger_gain').value
        self._haptics_enabled = self.get_parameter('haptics_enabled').value

        self._pub_hand   = self.create_publisher(Float32MultiArray, '/hand/finger_cmd', 10)
        self._pub_haptic = self.create_publisher(Int32MultiArray,   '/glove/servo_cmd', 10)

        self._sub_glove = self.create_subscription(
            Float32MultiArray, '/glove/finger_pct', self._glove_cb, 10)
        self._sub_fsr = self.create_subscription(
            Float32MultiArray, '/hand/fsr_pct', self._fsr_cb, 10)

        self._lock             = threading.Lock()
        self._latest_glove_pct = [0.0] * 5
        self._latest_fsr_pct   = [0.0] * 5

        haptic_period = 1.0 / self.get_parameter('haptic_rate_hz').value
        self._haptic_timer = self.create_timer(haptic_period, self._haptic_cb)

        self.get_logger().info(
            f'HapticBridgeNode started  '
            f'finger_gain={self._gain}  haptics_enabled={self._haptics_enabled}  '
            f'haptic_rate_hz={1.0/haptic_period:.0f}')

    # ── Forward path: glove → hand ────────────────────────────────────────────

    def _glove_cb(self, msg: Float32MultiArray):
        """Scale glove curl and forward to robot hand finger command."""
        if len(msg.data) < 5:
            return
        out = Float32MultiArray()
        out.data = [max(0.0, min(1.0, float(v) * self._gain)) for v in msg.data[:5]]
        self._pub_hand.publish(out)
        with self._lock:
            self._latest_glove_pct = list(msg.data[:5])

    # ── Haptic path: hand FSR → glove servo ───────────────────────────────────

    def _fsr_cb(self, msg: Float32MultiArray):
        """Cache latest FSR readings for use in the haptic timer callback."""
        if len(msg.data) < 5:
            return
        with self._lock:
            self._latest_fsr_pct = list(msg.data[:5])

    def _haptic_cb(self):
        """
        Per-finger haptic feedback, called at haptic_rate_hz (default 20 Hz).

        For each finger independently:
          - FSR above threshold  →  lock servo at current glove position
          - FSR below threshold  →  servo free (1000)

        Then applies the MAX_LOCKED_FINGERS cap: if too many fingers would be
        locked, the ones with the lowest FSR readings are freed first.
        """
        if not self._haptics_enabled:
            return

        with self._lock:
            glove_pct = list(self._latest_glove_pct)
            fsr_pct   = list(self._latest_fsr_pct)

        # ── Step 1: per-finger lock/free decision ─────────────────────────────
        # Read thresholds from ROS parameters so ros2 param set takes effect
        # immediately without restarting the node.
        thresholds = [
            self.get_parameter('fsr_threshold_thumb').value,
            self.get_parameter('fsr_threshold_index').value,
            self.get_parameter('fsr_threshold_middle').value,
            self.get_parameter('fsr_threshold_ring').value,
            self.get_parameter('fsr_threshold_pinky').value,
        ]

        cmds = []
        for i, name in enumerate(FINGER_NAMES):
            if fsr_pct[i] > thresholds[i]:
                cmd = max(0, min(1000, int((1.0 - glove_pct[i]) * 1000)))
            else:
                cmd = 1000  # free
            cmds.append(cmd)

        # ── Step 2: enforce MAX_LOCKED_FINGERS cap ────────────────────────────
        locked_indices = [i for i, cmd in enumerate(cmds) if cmd < 1000]
        while len(locked_indices) > MAX_LOCKED_FINGERS:
            # Free the locked finger with the weakest FSR signal
            weakest = min(locked_indices, key=lambda i: fsr_pct[i])
            cmds[weakest] = 1000
            locked_indices.remove(weakest)

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
