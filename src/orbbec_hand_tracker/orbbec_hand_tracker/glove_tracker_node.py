#!/usr/bin/env python3
"""
Glove hand tracker — XYZ position + finger curl.

Drop-in replacement for orbbec_hand_tracker_node.py for glove users.
Publishes the same RobotArmTarget message to the same topic using the same
coordinate math.  Finger curl is computed from MediaPipe joint angles and
published to /hand/finger_cmd for the robotic hand node.

Topics published:
  raw_robot_arm_target  (hand_interfaces/RobotArmTarget)
  /hand/finger_cmd      (std_msgs/Float32MultiArray)  — 5 values 0.0–1.0
                                                         [thumb, index, middle, ring, pinky]

Subscribes to:
  /camera/color/image_raw
  /camera/depth/image_raw
  /camera/depth/camera_info

Run with:
  source ~/ros2_ws/install/setup.bash
  ros2 run orbbec_hand_tracker glove_tracker_node

Press Q to quit.
"""

import math
import time
import threading
import sys
import termios
import tty
import numpy as np
import cv2
import mediapipe as mp

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from hand_interfaces.msg import RobotArmTarget
from std_msgs.msg import Float32MultiArray, String

# ── Glove color adjustments ────────────────────────────────────────────────────
HUE_OFFSET = 179
SAT_BOOST  = 111
VAL_OFFSET = -22

# ── MediaPipe detection resolution ────────────────────────────────────────────
DETECT_WIDTH  = 228
DETECT_HEIGHT = 163

# ── Palm center — average of wrist + index MCP + pinky MCP ────────────────────
# Matches orbbec_hand_tracker_node.py
PALM_IDS = [0, 5, 17]

# ── Arm coordinate home / clamp (matches orbbec_hand_tracker_node.py) ──────────
HOME_X = 0.20
HOME_Y = 0.00
HOME_Z = 0.24
ARM_Z_MIN = 0.10
ARM_Z_MAX = 0.50

# ── Holdover — keep last position when MediaPipe loses the hand ────────────────
HOLDOVER_FRAMES = 80

# ── One Euro Filter params for XYZ position ───────────────────────────────────
# More responsive than curl was — safe_target_filter handles step limiting.
OEF_MIN_CUTOFF = 1.5   # Hz
OEF_BETA       = 0.01
OEF_D_CUTOFF   = 1.0

# ── One Euro Filter params for finger curl ────────────────────────────────────
CURL_MIN_CUTOFF = 2.0
CURL_BETA       = 0.05
CURL_D_CUTOFF   = 1.0

# ── Finger curl joint indices ─────────────────────────────────────────────────
# Each tuple: (MCP, PIP, DIP, TIP)
FINGER_JOINTS = {
    "thumb":  (1,  2,  3,  4),
    "index":  (5,  6,  7,  8),
    "middle": (9,  10, 11, 12),
    "ring":   (13, 14, 15, 16),
    "pinky":  (17, 18, 19, 20),
}
FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky"]

# Max expected joint-angle sum for full curl (degrees)
# For 4 fingers: PIP + DIP angles sum to ~170° when fist
# For thumb: MCP angle ~80° when tucked
CURL_MAX_DEGREES = {"thumb": 80.0, "index": 170.0, "middle": 170.0,
                    "ring": 170.0, "pinky": 170.0}

mp_hands = mp.solutions.hands
mp_draw  = mp.solutions.drawing_utils


# ── Image preprocessing ────────────────────────────────────────────────────────

def apply_hue_sat(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV).astype(np.int32)
    hsv[:,:,0] = np.clip(hsv[:,:,0] + HUE_OFFSET, 0, 179)
    hsv[:,:,1] = np.clip(hsv[:,:,1] + SAT_BOOST,  0, 255)
    hsv[:,:,2] = np.clip(hsv[:,:,2] + VAL_OFFSET,  0, 255)
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)


def prepare_for_mediapipe(img_bgr):
    modified = apply_hue_sat(img_bgr)
    if DETECT_WIDTH and DETECT_HEIGHT:
        modified = cv2.resize(modified, (DETECT_WIDTH, DETECT_HEIGHT),
                              interpolation=cv2.INTER_LINEAR)
    return cv2.cvtColor(modified, cv2.COLOR_BGR2RGB)


def detect_landmarks(hands_detector, img_bgr):
    rgb = prepare_for_mediapipe(img_bgr)
    results = hands_detector.process(rgb)
    if results.multi_hand_landmarks:
        return results.multi_hand_landmarks[0]
    return None


def _joint_angle(pts, a, b, c):
    """
    Angle at landmark b in the chain a→b→c (degrees).
    When straight (a, b, c collinear): ~0°
    When bent 90°: ~90°
    """
    ax, ay, az = pts[a]
    bx, by, bz = pts[b]
    cx, cy, cz = pts[c]
    u = (bx - ax, by - ay, bz - az)
    v = (cx - bx, cy - by, cz - bz)
    dot = u[0]*v[0] + u[1]*v[1] + u[2]*v[2]
    nu  = math.sqrt(u[0]**2 + u[1]**2 + u[2]**2)
    nv  = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    if nu < 1e-6 or nv < 1e-6:
        return 0.0
    cos_a = max(-1.0, min(1.0, dot / (nu * nv)))
    return math.degrees(math.acos(cos_a))


def compute_finger_curl(lm):
    """
    Returns [thumb, index, middle, ring, pinky] curl 0.0 (open) – 1.0 (closed).
    Uses MediaPipe normalized 3D landmark coordinates.
    """
    pts = [(lm.landmark[i].x, lm.landmark[i].y, lm.landmark[i].z)
           for i in range(21)]
    curls = []
    for finger in FINGER_ORDER:
        mcp, pip, dip, tip = FINGER_JOINTS[finger]
        if finger == "thumb":
            # Thumb: single angle at MCP (CMC→MCP→TIP)
            angle = _joint_angle(pts, mcp - 1, mcp, tip)
        else:
            # Other fingers: sum PIP + DIP bend angles
            angle = _joint_angle(pts, mcp, pip, dip) + _joint_angle(pts, pip, dip, tip)
        curl = angle / CURL_MAX_DEGREES[finger]
        curls.append(max(0.0, min(1.0, curl)))
    return curls


# ── One Euro Filter ────────────────────────────────────────────────────────────

class OneEuroFilter:
    """
    Adaptive low-pass filter.
    Smooth when still (min_cutoff), responsive when moving (beta).
    """

    def __init__(self, min_cutoff=OEF_MIN_CUTOFF, beta=OEF_BETA, d_cutoff=OEF_D_CUTOFF):
        self.min_cutoff = min_cutoff
        self.beta       = beta
        self.d_cutoff   = d_cutoff
        self._x  = None
        self._dx = 0.0
        self._t  = None

    @staticmethod
    def _alpha(cutoff, freq):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau * freq)

    def __call__(self, x, t=None):
        now = t if t is not None else time.monotonic()
        if self._x is None:
            self._x, self._t = x, now
            return x
        freq = 1.0 / max(now - self._t, 1e-6)
        self._t = now
        dx = (x - self._x) * freq
        a_d = self._alpha(self.d_cutoff, freq)
        self._dx += a_d * (dx - self._dx)
        cutoff = self.min_cutoff + self.beta * abs(self._dx)
        a = self._alpha(cutoff, freq)
        self._x += a * (x - self._x)
        return self._x


# ── Node ───────────────────────────────────────────────────────────────────────

class GloveTrackerNode(Node):

    def __init__(self):
        super().__init__('glove_tracker_node')
        self.bridge = CvBridge()
        self.hands  = mp_hands.Hands(
            static_image_mode=False,
            model_complexity=0,
            max_num_hands=1,
            min_detection_confidence=0.50,
            min_tracking_confidence=0.42,
        )

        self.fx = self.fy = self.cx = self.cy = None
        self.latest_depth   = None
        self.last_lm        = None
        self.holdover_count = 0

        # One Euro Filter per axis
        self.oef_x = OneEuroFilter()
        self.oef_y = OneEuroFilter()
        self.oef_z = OneEuroFilter()

        self.pub      = self.create_publisher(RobotArmTarget,   'raw_robot_arm_target', 10)
        self.pub_curl = self.create_publisher(Float32MultiArray, '/hand/finger_cmd',    10)
        self.pub_cmd  = self.create_publisher(String,            '/arm_control_command', 10)
        self._tracking_active = False

        # Keyboard thread for wrist control
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()
        self.get_logger().info('Wrist keys: W=up  S=down')

        # One Euro Filters for 5 finger curl values
        self.oef_curl = [
            OneEuroFilter(min_cutoff=CURL_MIN_CUTOFF, beta=CURL_BETA, d_cutoff=CURL_D_CUTOFF)
            for _ in range(5)
        ]

        self.create_subscription(Image,      '/camera/color/image_raw',   self.color_callback, 1)
        self.create_subscription(Image,      '/camera/depth/image_raw',   self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.info_callback,  10)

        self.get_logger().info('Glove tracker node started — focus OpenCV window, Q to quit')

    # ── Sensor callbacks ───────────────────────────────────────────────────────

    def info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg: Image):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def color_callback(self, msg: Image):
        if self.latest_depth is None or self.fx is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        img = cv2.flip(img, 1)
        lm  = detect_landmarks(self.hands, img)

        tracking_now = lm is not None
        if tracking_now:
            self.last_lm        = lm
            self.holdover_count = HOLDOVER_FRAMES
        elif self.holdover_count > 0:
            lm = self.last_lm
            self.holdover_count -= 1

        currently_tracked = lm is not None

        # Publish pause/resume when tracking state changes
        if currently_tracked and not self._tracking_active:
            self._tracking_active = True
            msg = String(); msg.data = 'resume'
            self.pub_cmd.publish(msg)
        elif not currently_tracked and self._tracking_active:
            self._tracking_active = False
            msg = String(); msg.data = 'pause'
            self.pub_cmd.publish(msg)

        if currently_tracked:
            mp_draw.draw_landmarks(img, lm, mp_hands.HAND_CONNECTIONS)
            curls = self._publish_xyz(lm, img.shape)
            if curls:
                self._draw_curl_overlay(img, curls)

        is_holdover = currently_tracked and 0 < self.holdover_count < HOLDOVER_FRAMES
        if is_holdover:
            cv2.putText(img, 'holding...', (10, img.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 150, 255), 1)
        elif not currently_tracked:
            cv2.putText(img, 'TRACKING LOST', (10, img.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.imshow('Glove Tracker', img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            raise KeyboardInterrupt

    # ── XYZ extraction and publish ─────────────────────────────────────────────

    def _publish_xyz(self, lm, img_shape):
        h, w, _ = img_shape

        # Average palm landmarks to get stable palm center pixel
        us, vs = [], []
        for idx in PALM_IDS:
            us.append(int(lm.landmark[idx].x * w))
            vs.append(int(lm.landmark[idx].y * h))

        u = max(0, min(int(sum(us) / len(us)), self.latest_depth.shape[1] - 1))
        v = max(0, min(int(sum(vs) / len(vs)), self.latest_depth.shape[0] - 1))

        depth_raw = self.latest_depth[v, u]
        z = float(depth_raw) / 1000.0 if self.latest_depth.dtype == np.uint16 else float(depth_raw)

        if not math.isfinite(z) or z <= 0.0:
            return None

        # Project to camera-frame 3D (metres)
        cam_x = (u - self.cx) * z / self.fx
        cam_y = (v - self.cy) * z / self.fy

        # Map to arm coordinates (same transform as orbbec_hand_tracker_node.py)
        arm_x = HOME_X + (z     - 0.50)
        arm_y = HOME_Y + (-cam_x)
        arm_z = max(ARM_Z_MIN, min(ARM_Z_MAX, HOME_Z + (-cam_y)))

        # One Euro Filter — smooth XYZ before publishing
        now = time.monotonic()
        arm_x = self.oef_x(arm_x, now)
        arm_y = self.oef_y(arm_y, now)
        arm_z = self.oef_z(arm_z, now)

        msg = RobotArmTarget()
        msg.x, msg.y, msg.z = float(arm_x), float(arm_y), float(arm_z)
        self.pub.publish(msg)

        # Finger curl from landmarks
        raw_curls = compute_finger_curl(lm)
        smooth_curls = [self.oef_curl[i](raw_curls[i], now) for i in range(5)]
        curl_msg = Float32MultiArray()
        curl_msg.data = smooth_curls
        self.pub_curl.publish(curl_msg)
        return smooth_curls


    def _keyboard_loop(self):
        while rclpy.ok():
            try:
                key = _read_key()
                msg = String()
                if key in (b'w', b'W', b'\x1b[A'):
                    msg.data = 'wrist_up'
                    self.pub_cmd.publish(msg)
                elif key in (b's', b'S', b'\x1b[B'):
                    msg.data = 'wrist_down'
                    self.pub_cmd.publish(msg)
            except Exception:
                break

    def _draw_curl_overlay(self, img, curls):
        """Draw finger curl bars in the top-right corner of the frame."""
        h, w = img.shape[:2]
        names = ["T", "I", "M", "R", "P"]
        bar_h, bar_w, pad = 40, 12, 4
        x0 = w - (bar_w + pad) * 5 - pad
        y0 = pad
        for i, (name, curl) in enumerate(zip(names, curls)):
            x = x0 + i * (bar_w + pad)
            fill = int(bar_h * curl)
            cv2.rectangle(img, (x, y0), (x + bar_w, y0 + bar_h), (60, 60, 60), -1)
            if fill > 0:
                cv2.rectangle(img, (x, y0 + bar_h - fill), (x + bar_w, y0 + bar_h),
                              (60, 200, 80), -1)
            cv2.putText(img, name, (x, y0 + bar_h + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)


def _read_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.buffer.read(1)
        if ch == b'\x1b':
            ch += sys.stdin.buffer.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = GloveTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.hands.close()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
