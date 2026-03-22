import math
import cv2
import mediapipe as mp
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from hand_interfaces.msg import RobotArmTarget


# ── MediaPipe landmark indices ─────────────────────────────────────────────────
# Finger landmark layout per finger:
#   MCP (knuckle) → PIP (middle joint) → DIP → TIP
#
# Landmark IDs:
#   Wrist: 0
#   Thumb:  CMC=1  MCP=2  IP=3   TIP=4
#   Index:  MCP=5  PIP=6  DIP=7  TIP=8
#   Middle: MCP=9  PIP=10 DIP=11 TIP=12
#   Ring:   MCP=13 PIP=14 DIP=15 TIP=16
#   Pinky:  MCP=17 PIP=18 DIP=19 TIP=20

FINGER_LANDMARKS = {
    # finger_name: (base, mid, tip)  — used for curl calculation
    'thumb':  (2,  3,  4),
    'index':  (5,  6,  8),
    'middle': (9,  10, 12),
    'ring':   (13, 14, 16),
    'pinky':  (17, 18, 20),
}

# Wrist landmark — used as the base reference for all fingers
WRIST = 0

NUM_SERVO_CHANNELS = 15

# Curl angle range from MediaPipe (degrees)
# ~10° = fully straight, ~150° = fully curled
CURL_OPEN   = 15.0
CURL_CLOSED = 130.0

# Servo output range
SERVO_OPEN   = 180.0
SERVO_CLOSED = 0.0


def landmark_to_vec(lm):
    """Convert a MediaPipe NormalizedLandmark to a numpy 3D vector."""
    return np.array([lm.x, lm.y, lm.z])


def angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
    """Angle in degrees between two vectors."""
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return 0.0
    cos_a = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_a)))


def calc_finger_curl(landmarks, base_id: int, mid_id: int, tip_id: int) -> float:
    """
    Calculate the curl angle for one finger using three landmark IDs.
    Returns degrees: ~0 = straight, ~150 = fully curled.
    Uses the angle at the middle joint (PIP/IP).
    """
    wrist = landmark_to_vec(landmarks[WRIST])
    base  = landmark_to_vec(landmarks[base_id])
    mid   = landmark_to_vec(landmarks[mid_id])
    tip   = landmark_to_vec(landmarks[tip_id])

    v1 = base - wrist   # vector from wrist to base knuckle
    v2 = mid  - base    # vector from base to mid joint
    v3 = tip  - mid     # vector from mid to tip

    angle1 = angle_between(v1, v2)
    angle2 = angle_between(v2, v3)

    return (angle1 + angle2) / 2.0


def curl_to_servo(curl: float) -> float:
    """Map curl angle (degrees) → servo angle (degrees)."""
    t = (curl - CURL_OPEN) / (CURL_CLOSED - CURL_OPEN)
    t = max(0.0, min(1.0, t))
    # open finger → SERVO_OPEN, closed finger → SERVO_CLOSED
    return SERVO_OPEN + t * (SERVO_CLOSED - SERVO_OPEN)


class OrbbecHandTrackerNode(Node):
    def __init__(self):
        super().__init__('orbbec_hand_tracker_node')

        self.bridge = CvBridge()

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.latest_depth = None

        # ── Subscriptions ──────────────────────────────────────────────────────
        self.color_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.color_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self.info_callback, 10
        )

        # ── Publishers ─────────────────────────────────────────────────────────
        self.publisher_     = self.create_publisher(RobotArmTarget,    'raw_robot_arm_target', 10)
        self.pub_move_all   = self.create_publisher(Float32MultiArray, '/hand/move_all',       10)
        self.pub_curls      = self.create_publisher(Float32MultiArray, '/leap/curls',          10)

        # ── MediaPipe Hands ────────────────────────────────────────────────────
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.get_logger().info('Orbbec hand tracker node started')

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

        color_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)

        results = self.hands.process(color_rgb)
        if not results.multi_hand_landmarks:
            return

        hand_landmarks = results.multi_hand_landmarks[0]
        lm = hand_landmarks.landmark

        # ── Arm position (existing logic, unchanged) ───────────────────────────
        ids = [0, 5, 17]
        h, w, _ = color_bgr.shape
        us, vs = [], []
        for idx in ids:
            us.append(int(lm[idx].x * w))
            vs.append(int(lm[idx].y * h))

        u = max(0, min(int(sum(us) / len(us)), self.latest_depth.shape[1] - 1))
        v = max(0, min(int(sum(vs) / len(vs)), self.latest_depth.shape[0] - 1))

        depth_value = self.latest_depth[v, u]
        z = float(depth_value) / 1000.0 if self.latest_depth.dtype == np.uint16 else float(depth_value)

        if math.isnan(z) or math.isinf(z) or z <= 0.0:
            return

        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        arm_msg = RobotArmTarget()
        home_x, home_y, home_z = 0.20, 0.00, 0.24
        arm_msg.x = float(home_x + (z - 0.50))
        arm_msg.y = float(home_y + (-x))
        arm_msg.z = float(max(min(home_z + (-y), 0.30), 0.10))
        self.publisher_.publish(arm_msg)

        # ── Finger curl calculation ────────────────────────────────────────────
        curls = {}
        for finger, (base, mid, tip) in FINGER_LANDMARKS.items():
            curls[finger] = calc_finger_curl(lm, base, mid, tip)

        # Publish raw curls [thumb, index, middle, ring, pinky]
        curl_list = [curls['thumb'], curls['index'], curls['middle'],
                     curls['ring'], curls['pinky']]
        self.pub_curls.publish(Float32MultiArray(data=curl_list))

        # ── Map curls → servo angles ───────────────────────────────────────────
        thumb  = curl_to_servo(curls['thumb'])
        index  = curl_to_servo(curls['index'])
        middle = curl_to_servo(curls['middle'])
        ring   = curl_to_servo(curls['ring'])
        pinky  = curl_to_servo(curls['pinky'])

        # Build 15-channel command (-1.0 = skip channel)
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

        # Index  → ch 9, 10, 11  (change to index if wired)
        angles[9]  = -1.0
        angles[10] = -1.0
        angles[11] = -1.0

        # Thumb  → ch 12, 13
        angles[12] = thumb
        angles[13] = thumb

        # Ch 14 unused
        angles[14] = -1.0

        self.pub_move_all.publish(Float32MultiArray(data=angles))

        self.get_logger().debug(
            f'curls: thumb={curls["thumb"]:.0f}° index={curls["index"]:.0f}° '
            f'middle={curls["middle"]:.0f}° ring={curls["ring"]:.0f}° '
            f'pinky={curls["pinky"]:.0f}° | '
            f'servos: thumb={thumb:.0f} middle={middle:.0f} pinky={pinky:.0f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OrbbecHandTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
