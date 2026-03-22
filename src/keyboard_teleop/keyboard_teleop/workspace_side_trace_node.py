#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from hand_interfaces.msg import RobotArmTarget


class WorkspaceSideTraceNode(Node):
    def __init__(self):
        super().__init__('workspace_side_trace_node')
        self.pub = self.create_publisher(RobotArmTarget, '/raw_robot_arm_target', 10)

        # same workspace center as filter
        self.cx = 0.20
        self.cy = 0.0
        self.cz = 0.28

        self.get_logger().info('Side workspace trace started.')

    def publish(self, x, y, z):
        msg = RobotArmTarget()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.pub.publish(msg)
        self.get_logger().info(f'x={x:.3f} y={y:.3f} z={z:.3f}')

    def run(self):

        # sweep left
        for y in [0.0, 0.02, 0.04, 0.05]:
            self.publish(self.cx, y, self.cz)
            time.sleep(1)

        # sweep right
        for y in [0.05, 0.03, 0.01, -0.01, -0.03, -0.05]:
            self.publish(self.cx, y, self.cz)
            time.sleep(1)

        # return to center
        self.publish(self.cx, 0.0, self.cz)


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceSideTraceNode()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
