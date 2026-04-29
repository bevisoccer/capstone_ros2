#!/usr/bin/env python3
"""
Compare tracker output vs filter output vs arm_control receipt in real time.

Run while the full system is live:
  source ~/ros2_ws/install/setup.bash
  python3 ~/ros2_ws/track_compare.py

Columns:
  RAW   = /raw_robot_arm_target  (glove tracker publishes)
  FILT  = /robot_arm_target      (safe_target_filter publishes)
  DROP  = message on raw that produced no filter output within 150ms
"""

import time
import threading
import rclpy
from rclpy.node import Node
from hand_interfaces.msg import RobotArmTarget

STALE_S = 0.15   # seconds — if no filter output this long after raw, flag as dropped


class TrackCompare(Node):
    def __init__(self):
        super().__init__('track_compare')
        self._lock = threading.Lock()
        self._last_raw  = None
        self._last_filt = None
        self._last_raw_t  = 0.0
        self._last_filt_t = 0.0
        self._raw_count  = 0
        self._filt_count = 0

        self.create_subscription(
            RobotArmTarget, '/raw_robot_arm_target',  self._raw_cb,  10)
        self.create_subscription(
            RobotArmTarget, '/robot_arm_target', self._filt_cb, 10)
        self.create_timer(0.1, self._report)
        print(f"{'TIME':>8}  {'SOURCE':<6}  {'X':>7}  {'Y':>7}  {'Z':>7}  NOTE")
        print("-" * 60)

    def _raw_cb(self, msg):
        with self._lock:
            self._last_raw   = msg
            self._last_raw_t = time.monotonic()
            self._raw_count += 1

    def _filt_cb(self, msg):
        with self._lock:
            self._last_filt   = msg
            self._last_filt_t = time.monotonic()
            self._filt_count += 1
            r = self._last_raw
            note = ''
            if r is not None:
                dy = abs(msg.y - r.y)
                dz = abs(msg.z - r.z)
                if dy > 0.01:
                    note += f' Y_DIFF={dy:+.3f}'
                if dz > 0.01:
                    note += f' Z_DIFF={dz:+.3f}'
        t = time.monotonic()
        print(f"{t%1000:>8.3f}  {'FILT':<6}  {msg.x:>7.3f}  {msg.y:>7.3f}  {msg.z:>7.3f}{note}")

    def _report(self):
        now = time.monotonic()
        with self._lock:
            raw  = self._last_raw
            filt = self._last_filt
            rt   = self._last_raw_t
            ft   = self._last_filt_t
            rc   = self._raw_count
            fc   = self._filt_count
            self._raw_count  = 0
            self._filt_count = 0

        if raw is None:
            return

        # Print raw on every report tick (latest value)
        print(f"{now%1000:>8.3f}  {'RAW':<6}  {raw.x:>7.3f}  {raw.y:>7.3f}  {raw.z:>7.3f}"
              f"  (msgs/100ms: raw={rc} filt={fc})")

        # Flag if raw has updates but filter has been silent
        if rc > 0 and fc == 0:
            dy = abs(raw.y - (filt.y if filt else 0.0))
            dz = abs(raw.z - (filt.z if filt else 0.0))
            print(f"{'':>8}  {'** DROP':6}  filter silent while raw active"
                  f"  raw_y={raw.y:+.3f} filt_y={filt.y if filt else '---':>7}  "
                  f"raw_z={raw.z:+.3f} filt_z={filt.z if filt else '---':>7}")


def main():
    rclpy.init()
    node = TrackCompare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
