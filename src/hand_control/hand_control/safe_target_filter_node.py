#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from hand_interfaces.msg import RobotArmTarget

def clamp(value, lo, hi):
    return max(lo, min(hi, value))

class SafeTargetFilterNode(Node):
    def __init__(self):
        super().__init__('safe_target_filter_node')

        # Simple box workspace limits — tune these to your physical workspace
        self.x_min, self.x_max = 0.20, 0.60   # front/back (depth)
        self.y_min, self.y_max = -0.30, 0.30   # left/right — expanded to reach M1 joint limits
        self.z_min, self.z_max = 0.10, 0.55    # up/down

        # Per-axis step limits per timer tick (15 Hz)
        # X is slow — prevents jerky depth jumps. Y/Z pass through raw (no rate limit).
        self.x_max_step = 0.006   # ~9 cm/s
        # Per-axis jump limits — reject only impossible teleportation glitches.
        # Values cover the full per-axis workspace so normal fast hand movements pass.
        self.x_max_jump = 0.18   # X workspace 0.40m — rate-limited anyway
        self.y_max_jump = 0.65   # Y workspace 0.60m — needs headroom for fast moves
        self.z_max_jump = 0.50   # Z workspace 0.45m — needs headroom for fast moves
        self.publish_rate_hz    = 15.0
        self.latest_raw_target  = None
        self.current_safe_target = None
        self.has_received_target = False

        self.sub = self.create_subscription(
            RobotArmTarget, '/raw_robot_arm_target', self.target_callback, 10)
        self.pub = self.create_publisher(
            RobotArmTarget, '/robot_arm_target', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)
        self.get_logger().info('Safe target filter node started.')

    def clamp_to_box(self, x, y, z):
        x = clamp(x, self.x_min, self.x_max)
        y = clamp(y, self.y_min, self.y_max)
        z = clamp(z, self.z_min, self.z_max)
        return x, y, z

    def target_callback(self, msg):
        if not all(math.isfinite(v) for v in [msg.x, msg.y, msg.z]):
            self.get_logger().warn('Rejected non-finite target')
            return
        x, y, z = self.clamp_to_box(msg.x, msg.y, msg.z)
        clamped = RobotArmTarget()
        clamped.x, clamped.y, clamped.z = float(x), float(y), float(z)
        if self.current_safe_target is not None:
            # Y/Z glitch detection only — reject on impossible teleportation.
            # X has no jump check: x_max_step in timer_callback already rate-limits it,
            # so a large X jump just means the arm catches up slowly (safe).
            # Rejecting on X was silently killing Y/Z updates whenever the hand moved far.
            if (abs(clamped.y - self.current_safe_target.y) > self.y_max_jump or
                    abs(clamped.z - self.current_safe_target.z) > self.z_max_jump):
                return
        self.latest_raw_target = clamped
        if not self.has_received_target:
            self.current_safe_target = RobotArmTarget()
            self.current_safe_target.x = clamped.x
            self.current_safe_target.y = clamped.y
            self.current_safe_target.z = clamped.z
            self.has_received_target = True

    def timer_callback(self):
        if not self.has_received_target or self.latest_raw_target is None:
            return
        X_DEADBAND = 0.003
        Y_DEADBAND = 0.002
        Z_DEADBAND = 0.002

        def x_step(current, target):
            diff = target - current
            if abs(diff) <= X_DEADBAND:
                return 0.0
            return max(-self.x_max_step, min(self.x_max_step, diff))

        dx = x_step(self.current_safe_target.x, self.latest_raw_target.x)
        # Y and Z pass through directly — no rate limit, full workspace range
        raw_y = self.latest_raw_target.y
        raw_z = self.latest_raw_target.z
        dy = 0.0 if abs(raw_y - self.current_safe_target.y) <= Y_DEADBAND else raw_y - self.current_safe_target.y
        dz = 0.0 if abs(raw_z - self.current_safe_target.z) <= Z_DEADBAND else raw_z - self.current_safe_target.z

        if dx == 0.0 and dy == 0.0 and dz == 0.0:
            return

        next_target = RobotArmTarget()
        next_target.x = self.current_safe_target.x + dx
        next_target.y = self.current_safe_target.y + dy
        next_target.z = self.current_safe_target.z + dz
        x, y, z = self.clamp_to_box(next_target.x, next_target.y, next_target.z)
        next_target.x, next_target.y, next_target.z = float(x), float(y), float(z)
        if (abs(next_target.x - self.current_safe_target.x) > X_DEADBAND or
                abs(next_target.y - self.current_safe_target.y) > 0.001 or
                abs(next_target.z - self.current_safe_target.z) > 0.001):
            self.pub.publish(next_target)
            self.current_safe_target = next_target
            self.get_logger().info(
                f"safe target=({next_target.x:.3f},{next_target.y:.3f},{next_target.z:.3f})")

def main(args=None):
    rclpy.init(args=args)
    node = SafeTargetFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
