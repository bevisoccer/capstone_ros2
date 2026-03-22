import rclpy
from rclpy.node import Node
from hand_interfaces.msg import RobotArmTarget

import sys
import termios
import tty


def get_key():
    """Read a single keypress from the terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)

        if ch == '\x1b':  # arrow key sequence
            ch += sys.stdin.read(2)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return ch


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.publisher = self.create_publisher(
            RobotArmTarget,
            'raw_robot_arm_target',
            10
        )

        self.x = 0.2
        self.y = 0.0
        self.z = 0.24

        self.step = 0.05

        self.get_logger().info("Keyboard teleop started")
        self.get_logger().info("Arrow keys = XY movement")
        self.get_logger().info("R/F = Z up/down")
        self.get_logger().info("Q = quit")

        self.run()

    def publish_target(self):
        msg = RobotArmTarget()
        msg.x = float(self.x)
        msg.y = float(self.y)
        msg.z = float(self.z)

        self.publisher.publish(msg)

        self.get_logger().info(
            f"Published target: x={self.x:.3f} y={self.y:.3f} z={self.z:.3f}"
        )

    def run(self):

        while rclpy.ok():

            key = get_key()

            if key == '\x1b[A':  # up arrow → raise arm
                self.z += self.step

            elif key == '\x1b[B':  # down arrow → lower arm
                self.z -= self.step

            elif key == '\x1b[C':  # right arrow → arm right
                self.y -= self.step

            elif key == '\x1b[D':  # left arrow → arm left
                self.y += self.step

            elif key == 'r':  # r → arm away from you
                self.x += self.step

            elif key == 'f':  # f → arm toward you
                self.x -= self.step

            elif key == 'q':
                break

            else:
                continue

            self.publish_target()


def main(args=None):

    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
