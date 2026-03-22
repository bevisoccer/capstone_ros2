#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from hand_interfaces.msg import RobotArmTarget


class WorkspaceDomeTraceNode(Node):
    def __init__(self):
        super().__init__('workspace_dome_trace_node')
        self.pub = self.create_publisher(RobotArmTarget, '/raw_robot_arm_target', 10)

        # Match safe_target_filter_node.py
        self.cx = 0.16
        self.cy = 0.0
        self.cz = 0.24
        self.radius = 0.05

        self.get_logger().info('Workspace dome trace node started.')

    def publish_point(self, x, y, z):
        msg = RobotArmTarget()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.pub.publish(msg)
        self.get_logger().info(f'Published: x={x:.3f} y={y:.3f} z={z:.3f}')

    def trace_dome(self):
        # horizontal rings from low to high
        elevations_deg = [0, 15, 30]
        azimuths_deg = [-10, 0, 10]

        for elev_deg in elevations_deg:
            elev = math.radians(elev_deg)

            # ring radius in y/x plane
            ring_r = self.radius * math.cos(elev)
            z = self.cz + self.radius * math.sin(elev)

            for az_deg in azimuths_deg:
                az = math.radians(az_deg)

                # forward hemisphere centered on +x
                x = self.cx + ring_r * math.cos(az)
                y = self.cy + ring_r * math.sin(az)

                self.publish_point(x, y, z)
                time.sleep(2.0)

            # come back across the ring so motion is continuous
            for az_deg in reversed(azimuths_deg[:-1]):
                az = math.radians(az_deg)
                x = self.cx + ring_r * math.cos(az)
                y = self.cy + ring_r * math.sin(az)

                self.publish_point(x, y, z)
                time.sleep(2.0)

        # top point
        self.publish_point(self.cx, self.cy, self.cz + self.radius)
        time.sleep(2.0)

        # return to a comfortable center-ish point
        self.publish_point(0.20, 0.0, 0.27)

def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceDomeTraceNode()
    try:
        time.sleep(2.0)
        node.trace_dome()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
