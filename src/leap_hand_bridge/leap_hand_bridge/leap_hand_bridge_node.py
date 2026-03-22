#!/usr/bin/env python3
"""
leap_hand_bridge_node.py
========================
ROS 2 node that:
  1. Spawns leap_publisher as a subprocess
  2. Reads CURLS lines from its stdout
  3. Maps finger curl angles → servo angles
  4. Publishes to /hand/move_all  (picked up by hand_controller_node)

Topics Published
----------------
  /hand/move_all   std_msgs/Float32MultiArray   15 servo angles (index = channel)
  /leap/curls      std_msgs/Float32MultiArray   5 raw curl angles [thumb..pinky]
  /leap/gesture    std_msgs/String              detected gesture name

Parameters
----------
  leap_publisher_path  (string)  path to the compiled leap_publisher binary
  speed_dps            (float)   servo slew rate sent on startup (default 120)
  invert_mapping       (bool)    if True, open finger → 0°, closed → 180° (default False)

Channel map (must match hand_controller / servo_controller.ino wiring):
  Ch  0, 1, 2  → Pinky
  Ch  3, 4, 5  → Ring
  Ch  6, 7, 8  → Middle
  Ch  9, 10,11 → Index  (set to -1.0 to skip if not wired)
  Ch 12, 13    → Thumb
  Ch 14        → Unused (-1.0)
"""

import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

NUM_SERVO_CHANNELS = 15

# Fingers as reported by LeapC (index 0-4)
THUMB  = 0
INDEX  = 1
MIDDLE = 2
RING   = 3
PINKY  = 4

# Curl range from Leap (degrees) — tune these if needed
CURL_MIN = 5.0    # fully open finger
CURL_MAX = 85.0   # fully closed finger

# Servo angle range
SERVO_OPEN   = 180.0
SERVO_CLOSED = 0.0


def curl_to_servo(curl: float, invert: bool = False) -> float:
    """Map a Leap curl angle to a servo angle."""
    t = (curl - CURL_MIN) / (CURL_MAX - CURL_MIN)
    t = max(0.0, min(1.0, t))   # clamp to [0, 1]
    if invert:
        return SERVO_OPEN + t * (SERVO_CLOSED - SERVO_OPEN)
    else:
        return SERVO_CLOSED + t * (SERVO_OPEN - SERVO_CLOSED)


class LeapHandBridgeNode(Node):
    def __init__(self):
        super().__init__('leap_hand_bridge')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('leap_publisher_path', 'leap_publisher')
        self.declare_parameter('speed_dps',           120.0)
        self.declare_parameter('invert_mapping',      False)

        self._leap_bin  = self.get_parameter('leap_publisher_path').get_parameter_value().string_value
        self._speed_dps = self.get_parameter('speed_dps').get_parameter_value().double_value
        self._invert    = self.get_parameter('invert_mapping').get_parameter_value().bool_value

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_move    = self.create_publisher(Float32MultiArray, '/hand/move_all', 10)
        self._pub_curls   = self.create_publisher(Float32MultiArray, '/leap/curls',    10)
        self._pub_gesture = self.create_publisher(String,            '/leap/gesture',  10)

        # ── Set hand speed via /hand/speed ────────────────────────────────────
        from std_msgs.msg import Float32
        self._pub_speed = self.create_publisher(Float32, '/hand/speed', 10)
        self.create_timer(1.0, self._send_speed_once)
        self._speed_sent = False

        # ── Start leap_publisher subprocess ───────────────────────────────────
        self.get_logger().info(f'Launching leap_publisher: {self._leap_bin}')
        try:
            self._proc = subprocess.Popen(
                [self._leap_bin],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
        except FileNotFoundError:
            self.get_logger().error(
                f'leap_publisher binary not found at "{self._leap_bin}". '
                'Build it with CMake and pass the correct path via '
                '--ros-args -p leap_publisher_path:=/path/to/leap_publisher'
            )
            return

        # Reader thread for stderr (Leap diagnostics)
        threading.Thread(target=self._stderr_reader, daemon=True).start()
        # Reader thread for stdout (CURLS lines)
        threading.Thread(target=self._stdout_reader, daemon=True).start()

        self.get_logger().info('Leap hand bridge online — waiting for hand data.')

    # ── Speed init ────────────────────────────────────────────────────────────

    def _send_speed_once(self):
        if not self._speed_sent:
            from std_msgs.msg import Float32
            self._pub_speed.publish(Float32(data=float(self._speed_dps)))
            self.get_logger().info(f'Set hand speed to {self._speed_dps} dps')
            self._speed_sent = True

    # ── Subprocess readers ────────────────────────────────────────────────────

    def _stderr_reader(self):
        for line in self._proc.stderr:
            self.get_logger().info(f'[leap] {line.rstrip()}')

    def _stdout_reader(self):
        for line in self._proc.stdout:
            line = line.strip()
            if not line:
                continue
            if line == 'NO_HAND':
                self._pub_gesture.publish(String(data='NO_HAND'))
                continue
            if line.startswith('CURLS'):
                self._handle_curls(line)

    def _handle_curls(self, line: str):
        """
        Parse:  CURLS <thumb> <index> <middle> <ring> <pinky> <gesture>
        Map curl angles → servo positions and publish.
        """
        parts = line.split()
        if len(parts) < 7:
            return
        try:
            curls = [float(parts[i]) for i in range(1, 6)]
        except ValueError:
            return
        gesture = parts[6]

        # Publish raw curls
        self._pub_curls.publish(Float32MultiArray(data=curls))

        # Publish gesture
        self._pub_gesture.publish(String(data=gesture))

        # ── Map curl → servo angle ────────────────────────────────────────────
        # curl[0] = Thumb, [1] = Index, [2] = Middle, [3] = Ring, [4] = Pinky
        thumb  = curl_to_servo(curls[THUMB],  self._invert)
        index  = curl_to_servo(curls[INDEX],  self._invert)
        middle = curl_to_servo(curls[MIDDLE], self._invert)
        ring   = curl_to_servo(curls[RING],   self._invert)
        pinky  = curl_to_servo(curls[PINKY],  self._invert)

        # Build 15-channel command array
        # Use -1.0 for channels not wired (hand_controller skips negatives)
        angles = [-1.0] * NUM_SERVO_CHANNELS

        # Pinky  → ch 0, 1, 2
        angles[0] = pinky
        angles[1] = pinky
        angles[2] = pinky

        # Ring   → ch 3, 4, 5
        angles[3] = ring
        angles[4] = ring
        angles[5] = ring

        # Middle → ch 6, 7, 8
        angles[6] = middle
        angles[7] = middle
        angles[8] = middle

        # Index  → ch 9, 10, 11  (set to -1.0 if not wired)
        angles[9]  = -1.0   # change to `index` if index finger is wired
        angles[10] = -1.0
        angles[11] = -1.0

        # Thumb  → ch 12, 13
        angles[12] = thumb
        angles[13] = thumb

        # Ch 14 unused
        angles[14] = -1.0

        self._pub_move.publish(Float32MultiArray(data=angles))
        self.get_logger().debug(
            f'[{gesture}] pinky={pinky:.0f} ring={ring:.0f} '
            f'middle={middle:.0f} thumb={thumb:.0f}'
        )

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        if hasattr(self, '_proc') and self._proc.poll() is None:
            self._proc.terminate()
            self.get_logger().info('leap_publisher terminated.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeapHandBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
