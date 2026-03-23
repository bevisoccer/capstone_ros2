#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from hand_interfaces.msg import RobotArmTarget
from std_msgs.msg import String

def clamp(value, lo, hi):
    return max(lo, min(hi, value))

class SafeTargetFilterNode(Node):
    def __init__(self):
        super().__init__('safe_target_filter_node')
        self.cx, self.cy, self.cz = 0.20, 0.0, 0.20
        self.radius = 0.20
        self.y_min, self.y_max = -0.20, 0.20
        self.z_min, self.z_max = 0.15, 0.40
        self.max_cartesian_step = 0.006
        self.publish_rate_hz = 15.0
        self.latest_raw_target = None
        self.current_safe_target = None
        self.has_received_target = False
        self.paused = False

        self.sub = self.create_subscription(
            RobotArmTarget, '/raw_robot_arm_target', self.target_callback, 10)
        self.pub = self.create_publisher(
            RobotArmTarget, '/robot_arm_target', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)
        self.command_sub = self.create_subscription(
            String, '/arm_control_command', self._command_callback, 10)
        self.get_logger().info('Safe target filter node started.')

    def clamp_to_half_hemisphere(self, x, y, z):
        y = clamp(y, self.y_min, self.y_max)
        z = clamp(z, self.z_min, self.z_max)
        x = max(x, self.cx)
        dx, dy, dz = x - self.cx, y - self.cy, z - self.cz
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist > self.radius and dist > 1e-9:
            scale = self.radius / dist
            dx, dy, dz = dx*scale, dy*scale, dz*scale
        x, y, z = self.cx+dx, self.cy+dy, self.cz+dz
        y = clamp(y, self.y_min, self.y_max)
        z = clamp(z, self.z_min, self.z_max)
        return max(x, self.cx), y, z

    def _command_callback(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ('pause', 'park', 'quit'):
            self.paused = True
            self.get_logger().info(f'[FILTER] Paused by "{cmd}".')
        elif cmd == 'resume':
            self.paused = False
            self.get_logger().info('[FILTER] Resumed.')

    def target_callback(self, msg):
        if self.paused:
            return
        if not all(math.isfinite(v) for v in [msg.x, msg.y, msg.z]):
            self.get_logger().warn('Rejected non-finite target')
            return
        x, y, z = self.clamp_to_half_hemisphere(msg.x, msg.y, msg.z)
        clamped = RobotArmTarget()
        clamped.x, clamped.y, clamped.z = float(x), float(y), float(z)
        self.latest_raw_target = clamped
        if not self.has_received_target:
            self.current_safe_target = RobotArmTarget()
            self.current_safe_target.x = clamped.x
            self.current_safe_target.y = clamped.y
            self.current_safe_target.z = clamped.z
            self.has_received_target = True

    def timer_callback(self):
        if self.paused or not self.has_received_target or self.latest_raw_target is None:
            return
        dx = self.latest_raw_target.x - self.current_safe_target.x
        dy = self.latest_raw_target.y - self.current_safe_target.y
        dz = self.latest_raw_target.z - self.current_safe_target.z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist <= 1e-9:
            return
        scale = min(1.0, self.max_cartesian_step / dist)
        next_target = RobotArmTarget()
        next_target.x = self.current_safe_target.x + dx * scale
        next_target.y = self.current_safe_target.y + dy * scale
        next_target.z = self.current_safe_target.z + dz * scale
        x, y, z = self.clamp_to_half_hemisphere(next_target.x, next_target.y, next_target.z)
        next_target.x, next_target.y, next_target.z = float(x), float(y), float(z)
        DEADBAND = 0.0005
        if (abs(next_target.x - self.current_safe_target.x) > DEADBAND or
                abs(next_target.y - self.current_safe_target.y) > DEADBAND or
                abs(next_target.z - self.current_safe_target.z) > DEADBAND):
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
