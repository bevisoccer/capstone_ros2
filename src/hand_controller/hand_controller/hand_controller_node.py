#!/usr/bin/env python3
"""
hand_controller_node.py
=======================
ROS 2 node that bridges the ServoController (Teensy 4.1 + PCA9685) into the
capstone workspace.

Topics
------
Subscribed:
  /hand/move          std_msgs/Float32MultiArray
      Flat list of [channel, angle, channel, angle, ...] pairs.
      Example: data=[2, 90.0, 5, 45.0]  moves ch2→90° and ch5→45°.

  /hand/move_all      std_msgs/Float32MultiArray
      Exactly 15 floats — one target angle per channel (index = channel).
      Channels set to -1.0 are skipped (no command sent).

  /hand/estop         std_msgs/Bool
      True  → ESTOP (disable all channels immediately).
      False → RESUME_ALL (re-enable all channels).

  /hand/speed         std_msgs/Float32
      Set global slew rate in degrees/second (5–720).

Published:
  /hand/angles        std_msgs/Float32MultiArray
      Current reported angle of each channel (15 floats, index = channel).
      Published at ~10 Hz.

  /hand/ready         std_msgs/Bool
      Latched True once the Teensy sends its READY message after boot.

Parameters
----------
  serial_port  (string,  default '/dev/ttyACM0')
  baud_rate    (int,     default 115200)
  ready_timeout (float, default 20.0)   seconds to wait for READY on startup
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import Bool, Float32, Float32MultiArray

from hand_controller.servo_controller import ServoController, NUM_CHANNELS


class HandControllerNode(Node):
    def __init__(self):
        super().__init__('hand_controller')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('serial_port',   '/dev/ttyACM0')
        self.declare_parameter('baud_rate',     115200)
        self.declare_parameter('ready_timeout', 20.0)

        port          = self.get_parameter('serial_port').get_parameter_value().string_value
        baud          = self.get_parameter('baud_rate').get_parameter_value().integer_value
        ready_timeout = self.get_parameter('ready_timeout').get_parameter_value().double_value

        # ── Serial connection ─────────────────────────────────────────────────
        self.get_logger().info(f'Connecting to Teensy on {port} @ {baud} baud …')
        self.ctrl = ServoController(port, baud, connect=True)

        # Forward Teensy ERR / INFO lines to ROS logger
        self.ctrl.on_event('ERR',  lambda f: self.get_logger().error(f['_line']))
        self.ctrl.on_event('INFO', lambda f: self.get_logger().info(f['_line']))

        # ── Latched publisher for /hand/ready ─────────────────────────────────
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_ready = self.create_publisher(Bool, '/hand/ready', latched_qos)

        # ── Regular publishers ────────────────────────────────────────────────
        self._pub_angles = self.create_publisher(Float32MultiArray, '/hand/angles', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Float32MultiArray, '/hand/move',     self._cb_move,     10)
        self.create_subscription(Float32MultiArray, '/hand/move_all', self._cb_move_all, 10)
        self.create_subscription(Bool,              '/hand/estop',    self._cb_estop,    10)
        self.create_subscription(Float32,           '/hand/speed',    self._cb_speed,    10)

        # ── 10 Hz state publisher ─────────────────────────────────────────────
        self.create_timer(0.1, self._publish_state)

        # ── Wait for Teensy READY (non-blocking — log and continue) ───────────
        self.get_logger().info(f'Waiting up to {ready_timeout}s for Teensy READY …')
        if self.ctrl.wait_ready(timeout=ready_timeout):
            self.get_logger().info('Teensy is READY — hand controller online.')
            self._pub_ready.publish(Bool(data=True))
        else:
            self.get_logger().warn(
                'Timed out waiting for READY. Check USB connection and that '
                'servo_controller.ino is flashed on the Teensy. '
                'The node will continue — commands will be queued.'
            )

    # ── Subscriber callbacks ──────────────────────────────────────────────────

    def _cb_move(self, msg: Float32MultiArray):
        """
        /hand/move — [ch, angle, ch, angle, ...]
        Accepts any even-length list of (channel, angle) pairs.
        """
        data = list(msg.data)
        if len(data) % 2 != 0:
            self.get_logger().warn(
                '/hand/move received odd-length array — ignoring last element'
            )
            data = data[:-1]

        for i in range(0, len(data), 2):
            ch  = int(data[i])
            ang = float(data[i + 1])
            try:
                self.ctrl.move(ch, ang)
                self.get_logger().debug(f'MOVE ch={ch} → {ang:.1f}°')
            except ValueError as e:
                self.get_logger().error(str(e))

    def _cb_move_all(self, msg: Float32MultiArray):
        """
        /hand/move_all — exactly 15 floats (one per channel).
        Set a channel to -1.0 to skip it.
        """
        if len(msg.data) != NUM_CHANNELS:
            self.get_logger().warn(
                f'/hand/move_all expects {NUM_CHANNELS} values, '
                f'got {len(msg.data)} — ignoring'
            )
            return

        for ch, ang in enumerate(msg.data):
            if ang < 0.0:
                continue  # sentinel: skip this channel
            try:
                self.ctrl.move(ch, float(ang))
                self.get_logger().debug(f'MOVE ch={ch} → {ang:.1f}°')
            except ValueError as e:
                self.get_logger().error(str(e))

    def _cb_estop(self, msg: Bool):
        """
        /hand/estop — True = emergency stop, False = resume all
        """
        if msg.data:
            self.get_logger().warn('ESTOP received — disabling all servo channels.')
            self.ctrl.estop()
        else:
            self.get_logger().info('ESTOP cleared — resuming all servo channels.')
            self.ctrl.resume_all()

    def _cb_speed(self, msg: Float32):
        """
        /hand/speed — slew rate in degrees/second
        """
        self.ctrl.set_speed(float(msg.data))
        self.get_logger().info(f'Speed set to {msg.data:.1f} dps')

    # ── State publisher ───────────────────────────────────────────────────────

    def _publish_state(self):
        angles = [self.ctrl.channels[ch].angle for ch in range(NUM_CHANNELS)]
        self._pub_angles.publish(Float32MultiArray(data=angles))

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.get_logger().info('Shutting down — disconnecting from Teensy.')
        self.ctrl.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
