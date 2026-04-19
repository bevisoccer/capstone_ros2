#!/usr/bin/env python3
"""
calib_log_buffer_node.py — Buffers log output from all non-glove nodes during
glove calibration and flushes it to stdout when calibration finishes.

Subscribes:
  /glove/calibrating  (std_msgs/Bool)
  /rosout             (rcl_interfaces/msg/Log)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rcl_interfaces.msg import Log

LEVEL_NAMES = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}
SUPPRESS_NODES = {'glove_node', 'calib_log_buffer'}


class CalibLogBufferNode(Node):
    def __init__(self):
        super().__init__('calib_log_buffer')

        self._calibrating = False
        self._buffer = []

        self.create_subscription(Bool, '/glove/calibrating', self._calib_cb, 10)
        self.create_subscription(Log,  '/rosout',            self._log_cb,   100)

    def _calib_cb(self, msg: Bool):
        if msg.data and not self._calibrating:
            self._calibrating = True
            self._buffer.clear()
        elif not msg.data and self._calibrating:
            self._calibrating = False
            self._flush()

    def _log_cb(self, msg: Log):
        if msg.name in SUPPRESS_NODES:
            return
        if self._calibrating:
            self._buffer.append(msg)
        # When not calibrating, let ROS handle normal output (do nothing here)

    def _flush(self):
        if not self._buffer:
            return
        print('\n' + '='*50, flush=True)
        print(f'  Buffered output from calibration ({len(self._buffer)} messages):', flush=True)
        print('='*50, flush=True)
        for msg in self._buffer:
            level = LEVEL_NAMES.get(msg.level, str(msg.level))
            print(f'  [{level}] [{msg.name}]: {msg.msg}', flush=True)
        print('='*50 + '\n', flush=True)
        self._buffer.clear()


def main(args=None):
    rclpy.init(args=args)
    node = CalibLogBufferNode()
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
