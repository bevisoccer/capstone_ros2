#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from hand_interfaces.msg import RobotArmTarget


class WorkspaceTraceNode(Node):
    def __init__(self):
        super().__init__('workspace_trace_node')
        self.pub = self.create_publisher(RobotArmTarget, '/raw_robot_arm_target', 10)

        # Half-hemisphere parameters (match your filter)
        self.cx = 0.16
        self.cy = 0.0
        self.cz = 0.24
        self.radius = 0.08

        self.get_logger().info('Workspace trace node started.')

    def publish_point(self, x, y, z):
        msg = RobotArmTarget()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.pub.publish(msg)
        self.get_logger().info(f'Published: x={x:.3f} y={y:.3f} z={z:.3f}')

    def trace_xz_arc(self):
        # Trace top half of the circle in XZ with y = 0
        # theta: 0 -> pi/2 gives forward/up arc from base to top/front
        for deg in range(0, 91, 5):
            theta = math.radians(deg)
            x = self.cx + self.radius * math.cos(theta)
            y = 0.0
            z = self.cz + self.radius * math.sin(theta)
            self.publish_point(x, y, z)
            time.sleep(1.0)

        # Then go back down
        for deg in range(90, -1, -5):
            theta = math.radians(deg)
            x = self.cx + self.radius * math.cos(theta)
            y = 0.0
            z = self.cz + self.radius * math.sin(theta)
            self.publish_point(x, y, z)
            time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceTraceNode()

    try:
        time.sleep(1.0)
        node.trace_xz_arc()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
