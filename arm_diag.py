#!/usr/bin/env python3
"""
Diagnostic: shows tracker output rate + arm_control log patterns.
Run in a separate terminal while the system is live:
  python3 ~/ros2_ws/arm_diag.py
"""
import threading
import time
import glob
import os
import sys
import rclpy
from rclpy.node import Node
from hand_interfaces.msg import RobotArmTarget
from std_msgs.msg import String

LOG_FILE = '/tmp/arm_diag.log'

class Tee:
    """Write to both stdout and a log file simultaneously."""
    def __init__(self, path):
        self._f = open(path, 'w', buffering=1)
    def write(self, s):
        sys.stdout.write(s)
        self._f.write(s)
    def flush(self):
        sys.stdout.flush()
        self._f.flush()

_tee = Tee(LOG_FILE)

def log(msg):
    ts = time.strftime('%H:%M:%S')
    line = f'[{ts}] {msg}\n'
    _tee.write(line)
    _tee.flush()


class DiagNode(Node):
    def __init__(self):
        super().__init__('arm_diag')
        self._last_msg = None
        self._count = 0
        self._t0 = time.monotonic()

        self.create_subscription(RobotArmTarget, '/raw_robot_arm_target', self._raw_cb, 10)
        self.create_subscription(String, '/arm_control_command', self._cmd_cb, 10)
        self.create_timer(1.0, self._report)

    def _raw_cb(self, msg):
        self._count += 1
        self._last_msg = msg

    def _cmd_cb(self, msg):
        log(f"[CMD→arm] {msg.data}")

    def _report(self):
        elapsed = time.monotonic() - self._t0
        rate = self._count / elapsed if elapsed > 0 else 0
        if self._last_msg:
            m = self._last_msg
            log(f"[tracker] {rate:.1f} Hz  y={m.y:+.3f}  z={m.z:+.3f}  x={m.x:+.3f}")
        else:
            log("[tracker] no messages on /raw_robot_arm_target")
        self._count = 0
        self._t0 = time.monotonic()


def tail_arm_control_log():
    """Tail the arm_control_node log file for key patterns."""
    patterns = ('[TELEOP]', '[TRACK]', '[ERROR]', '[CMD]', '[STARTUP]')
    log_root = os.path.expanduser('~/.ros/log')

    # Find the most recent log dir
    while True:
        dirs = sorted(glob.glob(f'{log_root}/*/'), key=os.path.getmtime, reverse=True)
        if dirs:
            break
        time.sleep(0.5)

    log_dir = dirs[0]
    # Find arm_control log file
    log_file = None
    for _ in range(20):
        candidates = glob.glob(f'{log_dir}*arm_control*')
        if candidates:
            log_file = max(candidates, key=os.path.getmtime)
            break
        time.sleep(0.5)

    if not log_file:
        log("[diag] arm_control log file not found — only showing topic data above")
        return

    log(f"[diag] tailing {log_file}")
    with open(log_file, 'r') as f:
        f.seek(0, 2)  # seek to end
        while True:
            line = f.readline()
            if line:
                if any(p in line for p in patterns):
                    log(f"[arm_ctrl] {line.rstrip()}")
            else:
                time.sleep(0.05)


def main():
    log(f"arm_diag started — logging to {LOG_FILE}")
    rclpy.init()
    node = DiagNode()

    log_thread = threading.Thread(target=tail_arm_control_log, daemon=True)
    log_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
