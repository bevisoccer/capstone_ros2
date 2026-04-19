#!/usr/bin/env python3
"""
haptic_bridge_node.py — Bridges the haptic glove and the robot hand.

  /glove/finger_pct  (Float32MultiArray, 0.0–1.0)  → /hand/finger_cmd
  /hand/fsr_pct      (Float32MultiArray, 0.0–100.0) → /glove/servo_cmd  (when haptics_enabled)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray


class HapticBridgeNode(Node):
    def __init__(self):
        super().__init__('haptic_bridge_node')

        self.declare_parameter('finger_gain',     1.5)
        self.declare_parameter('haptics_enabled', False)
        self._gain            = self.get_parameter('finger_gain').value
        self._haptics_enabled = self.get_parameter('haptics_enabled').value

        self._pub_hand   = self.create_publisher(Float32MultiArray, '/hand/finger_cmd', 10)
        self._pub_haptic = self.create_publisher(Int32MultiArray,   '/glove/servo_cmd', 10)
        self._sub_glove  = self.create_subscription(
            Float32MultiArray, '/glove/finger_pct', self._glove_cb, 10)
        self._sub_fsr    = self.create_subscription(
            Float32MultiArray, '/hand/fsr_pct', self._fsr_cb, 10)

        self.get_logger().info(
            f'HapticBridgeNode started  finger_gain={self._gain}  haptics_enabled={self._haptics_enabled}')

    def _glove_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 5:
            return
        out = Float32MultiArray()
        out.data = [max(0.0, min(1.0, float(v) * self._gain)) for v in msg.data[:5]]
        self._pub_hand.publish(out)

    def _fsr_cb(self, msg: Float32MultiArray):
        if not self._haptics_enabled or len(msg.data) < 5:
            return
        out = Int32MultiArray()
        out.data = [max(0, min(1000, int(float(v) * 10.0))) for v in msg.data[:5]]
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
