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

# ── White glove mask + skin colorize ─────────────────────────────────────────
# Mask isolates white glove pixels, then colorizes them to skin tone so
# MediaPipe (trained on skin) detects the glove reliably.
MASK_ENABLED    = True
MASK_VAL_MIN    = 69    # pixels darker than this are blacked out
MASK_SAT_MAX    = 40    # pixels more saturated than this are blacked out (not white)
SKIN_HUE        = 11    # target hue for colorized glove (skin = ~5-15)
SKIN_SAT        = 110   # target saturation for colorized glove

# ── MediaPipe detection resolution ────────────────────────────────────────────
DETECT_WIDTH  = 228
DETECT_HEIGHT = 163

# ── Palm center — average of wrist + index MCP + pinky MCP ────────────────────
# Matches orbbec_hand_tracker_node.py
PALM_IDS = [0, 5, 17]

# ── Arm coordinate home / clamp (matches orbbec_hand_tracker_node.py) ──────────
HOME_X = 0.35   # midpoint of arm X range (0.20–0.50) so neutral = center
HOME_Y = 0.00
HOME_Z = 0.24
ARM_X_MIN = 0.20
ARM_X_MAX = 0.50
ARM_Z_MIN = 0.10
ARM_Z_MAX = 0.50

# ── Holdover — keep last position when MediaPipe loses the hand ────────────────
HOLDOVER_FRAMES = 80

# ── Movement sensitivity scale ─────────────────────────────────────────────────
# 1 cm of hand movement → ARM_Z/Y_SCALE cm of arm workspace movement.
# Z needs the most amplification — the arm's M2 range spans 35 cm of arm_z.
ARM_X_SCALE = 0.3
ARM_Z_SCALE = 0.5
ARM_Y_SCALE = 0.5

# ── Gesture recognition ───────────────────────────────────────────────────────
GESTURE_HOLD_FRAMES     = 20   # frames gesture must be held to fire (~0.5s at ~30fps)
GESTURE_COOLDOWN_FRAMES = 45   # frames blocked after a gesture fires (~1.5s)
GEST_OPEN_THRESH = 0.30        # curl < this → finger extended
GEST_CURL_THRESH = 0.60        # curl > this → finger curled

# gesture name → (arm_command_or_None, display_label, bgr_color)
GESTURE_ACTIONS = {
    'open_palm': ('hand_open', 'OPEN PALM',  (0,   200, 220)),
    'fist':      (None,        'FIST',       (80,  220,  80)),
    'thumbs_up': (None,        'THUMBS UP',  (0,   220, 255)),
    'point':     (None,        'POINT',      (255, 180,  50)),
    'peace':     (None,        'PEACE / V',  (200,  80, 200)),
}

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


def classify_gesture(curls):
    """Return gesture name from [thumb,index,middle,ring,pinky] curl values, or None."""
    t, i, m, r, p = curls
    ext = [v < GEST_OPEN_THRESH for v in (t, i, m, r, p)]
    crl = [v > GEST_CURL_THRESH for v in (t, i, m, r, p)]

    if all(ext):
        return 'open_palm'
    if all(crl):
        return 'fist'
    if ext[0] and crl[1] and crl[2] and crl[3] and crl[4]:
        return 'thumbs_up'
    if ext[1] and crl[2] and crl[3] and crl[4]:   # index up, rest curled, thumb free
        return 'point'
    if ext[1] and ext[2] and crl[3] and crl[4]:   # index+middle up, rest curled, thumb free
        return 'peace'
    return None


# ── Image preprocessing ────────────────────────────────────────────────────────

def apply_hue_sat(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV).astype(np.int32)
    hsv[:,:,0] = np.clip(hsv[:,:,0] + HUE_OFFSET, 0, 179)
    hsv[:,:,1] = np.clip(hsv[:,:,1] + SAT_BOOST,  0, 255)
    hsv[:,:,2] = np.clip(hsv[:,:,2] + VAL_OFFSET,  0, 255)
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)


def apply_white_mask(img_bgr):
    """Mask white glove pixels and colorize them to skin tone for MediaPipe."""
    hsv  = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, MASK_VAL_MIN), (179, MASK_SAT_MAX, 255))
    # Colorize masked pixels to skin hue/saturation, keep original brightness
    hsv[:, :, 0] = np.where(mask > 0, SKIN_HUE, 0)
    hsv[:, :, 1] = np.where(mask > 0, SKIN_SAT, 0)
    # Black out non-glove pixels
    hsv[:, :, 2] = np.where(mask > 0, hsv[:, :, 2], 0)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def prepare_for_mediapipe(img_bgr):
    if MASK_ENABLED:
        # Mask on original image before hue/sat boost — thresholds tuned on raw values
        modified = apply_white_mask(img_bgr)
    else:
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
        self.declare_parameter('hand_cv_enabled', True)
        self._hand_cv_enabled = self.get_parameter('hand_cv_enabled').value
        self.bridge = CvBridge()
        self.hands  = mp_hands.Hands(
            static_image_mode=False,
            model_complexity=0,
            max_num_hands=1,
            min_detection_confidence=0.10,
            min_tracking_confidence=0.10,
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
        self.pub_hand = self.create_publisher(String,            '/hand/command',        10)
        self._tracking_active = False
        self._last_xyz     = None
        self._last_raw_xyz = None   # uncalibrated arm coords, used to set calib origin
        self._last_palm_px = None
        self._arm_paused  = False
        self._hand_paused = False
        self._hand_estop  = False
        self._calibrating = True                        # wait for SPACE before tracking
        self._calib_raw   = (HOME_X, HOME_Y, HOME_Z)   # neutral point in raw arm coords
        self._display_frame = None
        self._display_lock  = threading.Lock()
        self._quit = False
        self._last_key_str = ''
        self._last_key_age  = 0

        # Gesture recognition state
        self._gest_candidate  = None   # gesture currently being held
        self._gest_hold       = 0      # consecutive frames held
        self._gest_cooldown   = 0      # frames until next gesture can fire
        self._gest_fired      = None   # most recently fired gesture
        self._gest_fired_age  = 0      # countdown for HUD flash after firing

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

        # Notify arm of tracking state changes (suppressed during calibration)
        if currently_tracked and not self._tracking_active and not self._calibrating:
            self._tracking_active = True
            msg = String(); msg.data = 'tracking_found'
            self.pub_cmd.publish(msg)
        elif not currently_tracked and self._tracking_active:
            self._tracking_active = False
            msg = String(); msg.data = 'tracking_lost'
            self.pub_cmd.publish(msg)

        if currently_tracked:
            mp_draw.draw_landmarks(img, lm, mp_hands.HAND_CONNECTIONS)
            curls = self._publish_xyz(lm, img.shape)
            if curls:
                self._draw_curl_overlay(img, curls)
                self._update_gesture(classify_gesture(curls))
            else:
                self._update_gesture(None)
        else:
            self._update_gesture(None)

        self._draw_xyz_overlay(img)
        self._draw_center_marker(img)
        self._draw_palm_marker(img)
        self._draw_status_hud(img, currently_tracked, self.holdover_count)
        self._draw_gesture_hud(img)
        with self._display_lock:
            self._display_frame = img

    def get_display_frame(self):
        with self._display_lock:
            return None if self._display_frame is None else self._display_frame.copy()

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
        # Landmark coords are already in flipped display space — use directly
        self._last_palm_px = (u, v)

        # Sample a 9x9 patch around the palm center and take the median of valid readings.
        # Single-pixel depth is unreliable when the hand moves to the edge of the frame
        # or the background is visible through the palm — median rejects outliers robustly.
        PATCH = 15
        dh, dw = self.latest_depth.shape[:2]
        v0, v1 = max(0, v - PATCH//2), min(dh, v + PATCH//2 + 1)
        u0, u1 = max(0, u - PATCH//2), min(dw, u + PATCH//2 + 1)
        patch = self.latest_depth[v0:v1, u0:u1].flatten().astype(np.float32)
        valid = patch[patch > 0]
        if len(valid) == 0:
            return None
        depth_raw = float(np.median(valid))
        z = depth_raw / 1000.0 if self.latest_depth.dtype == np.uint16 else depth_raw

        if not math.isfinite(z) or z <= 0.0:
            return None

        # Project to camera-frame 3D (metres)
        cam_x = (u - self.cx) * z / self.fx
        cam_y = (v - self.cy) * z / self.fy

        # Axial depth: remove lateral component so moving left/right doesn't shift x
        z_axial = math.sqrt(max(0.0, z*z - cam_x*cam_x - cam_y*cam_y))

        # Raw arm coords (before calibration offset)
        arm_x_raw = HOME_X + (z_axial - 0.50) * ARM_X_SCALE
        arm_y_raw = HOME_Y + (-cam_x * ARM_Y_SCALE)
        arm_z_raw = HOME_Z + (-cam_y * ARM_Z_SCALE)
        self._last_raw_xyz = (arm_x_raw, arm_y_raw, arm_z_raw)

        # During calibration: show raw position so user can pick neutral, don't publish
        if self._calibrating:
            self._last_xyz = (arm_x_raw,
                              arm_y_raw,
                              max(ARM_Z_MIN, min(ARM_Z_MAX, arm_z_raw)))
            return None

        # Apply calibration offset so confirmed neutral → HOME_X/Y/Z
        cx, cy, cz = self._calib_raw
        arm_x = max(ARM_X_MIN, min(ARM_X_MAX, arm_x_raw - cx + HOME_X))
        arm_y = arm_y_raw - cy + HOME_Y
        arm_z = max(ARM_Z_MIN, min(ARM_Z_MAX, arm_z_raw - cz + HOME_Z))

        # One Euro Filter — smooth XYZ before publishing
        now = time.monotonic()
        arm_x = self.oef_x(arm_x, now)
        arm_y = self.oef_y(arm_y, now)
        arm_z = self.oef_z(arm_z, now)

        self._last_xyz = (arm_x, arm_y, arm_z)

        msg = RobotArmTarget()
        msg.x, msg.y, msg.z = float(arm_x), float(arm_y), float(arm_z)
        self.pub.publish(msg)

        # Finger curl from landmarks
        raw_curls = compute_finger_curl(lm)
        smooth_curls = [self.oef_curl[i](raw_curls[i], now) for i in range(5)]
        if self._hand_cv_enabled:
            curl_msg = Float32MultiArray()
            curl_msg.data = smooth_curls
            self.pub_curl.publish(curl_msg)
        return smooth_curls


    def _send_arm(self, cmd):
        msg = String(); msg.data = cmd
        self.pub_cmd.publish(msg)

    def _send_hand(self, cmd):
        msg = String(); msg.data = cmd
        self.pub_hand.publish(msg)

    def _handle_key(self, key):
        if key == 0xFF:
            return
        if key == ord(' '):
            if self._calibrating and self._last_raw_xyz is not None:
                self._calib_raw   = self._last_raw_xyz
                self._calibrating = False
                self._arm_paused  = False
                # Reset filters so first tracking position starts clean
                self.oef_x = OneEuroFilter()
                self.oef_y = OneEuroFilter()
                self.oef_z = OneEuroFilter()
                # Reset safe_target_filter so stale current_safe_target doesn't
                # jump-reject the new HOME-calibrated targets
                self._send_arm('reset')
                self._send_arm('resume')
                self.get_logger().info(f'Calibrated neutral: raw={self._calib_raw}')
            elif not self._calibrating:
                # Mid-run recalibration: return arm to home, re-enter calibration mode
                self._calibrating     = True
                self._tracking_active = False
                self._arm_paused      = True
                self._send_arm('go_home')
                self.get_logger().info('Recalibrating — arm returning to home position.')
            return
        if key == ord('c') or key == ord('C'):
            self._calibrating     = True
            self._tracking_active = False
            self._send_arm('pause')
            self._arm_paused = True
            return
        print(f"[KEY] {key} (ord1={ord('1')})", flush=True)
        self._last_key_str = chr(key) if 32 <= key < 127 else f'#{key}'
        self._last_key_age  = 60
        if key == ord('1'):
            self._send_arm('wrist_up')
        elif key == ord('2'):
            self._send_arm('wrist_down')
        elif key == ord('3'):
            self._send_arm('elbow_out')
        elif key == ord('4'):
            self._send_arm('elbow_in')
        elif key == ord('5'):
            self._send_arm('palm_cw')
        elif key == ord('6'):
            self._send_arm('palm_ccw')
        elif key == ord('p') or key == ord('P'):
            if self._arm_paused:
                self._arm_paused = False
                self._send_arm('resume')
            else:
                self._arm_paused = True
                self._send_arm('pause')
        elif key == ord('o') or key == ord('O'):
            if self._hand_paused:
                self._hand_paused = False
                self._send_hand('resume')
            else:
                self._hand_paused = True
                self._send_hand('pause')
        elif key == ord('l') or key == ord('L'):
            if self._hand_estop:
                self._hand_estop = False
                self._send_hand('resume_all')
            else:
                self._hand_estop = True
                self._send_hand('estop')
        elif key == ord('x') or key == ord('X'):
            self._send_arm('stop')
        elif key == ord('['):
            self._send_arm('elbow_in')
        elif key == ord(']'):
            self._send_arm('elbow_out')
        elif key == ord('h') or key == ord('H'):
            msg = Float32MultiArray()
            msg.data = [1.0, 1.0, 1.0, 1.0, 1.0]
            self.pub_curl.publish(msg)
        elif key == ord('g') or key == ord('G'):
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.pub_curl.publish(msg)
        elif key == ord('q') or key == ord('Q'):
            self._send_arm('quit')
            time.sleep(2.0)   # ensure message delivered before ROS shuts down
            self._quit = True

    def _update_gesture(self, gesture):
        """Debounce gesture detection: fires after GESTURE_HOLD_FRAMES consecutive frames."""
        if self._gest_fired_age > 0:
            self._gest_fired_age -= 1
        if self._gest_cooldown > 0:
            self._gest_cooldown -= 1
            return

        if gesture != self._gest_candidate:
            self._gest_candidate = gesture
            self._gest_hold      = 0
            return

        if gesture is None:
            return

        self._gest_hold += 1
        if self._gest_hold >= GESTURE_HOLD_FRAMES:
            self._fire_gesture(gesture)
            self._gest_cooldown  = GESTURE_COOLDOWN_FRAMES
            self._gest_candidate = None
            self._gest_hold      = 0

    def _fire_gesture(self, gesture):
        info = GESTURE_ACTIONS.get(gesture)
        if not info:
            return
        cmd, label, _ = info
        self._gest_fired     = gesture
        self._gest_fired_age = 60
        self.get_logger().info(f'[GESTURE] {gesture}')
        if cmd == 'hand_open' and not self._calibrating:
            msg = Float32MultiArray()
            msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]
            self.pub_curl.publish(msg)

    def _draw_gesture_hud(self, img):
        """Draw gesture progress bar and fired-gesture flash below the status pills."""
        if self._calibrating:
            return
        h, w = img.shape[:2]
        y = 32  # just below the status pills row

        # Flash label after a gesture fires
        if self._gest_fired_age > 0 and self._gest_fired:
            info = GESTURE_ACTIONS.get(self._gest_fired)
            if info:
                _, label, color = info
                alpha = min(1.0, self._gest_fired_age / 20.0)
                c = tuple(int(v * alpha) for v in color)
                cv2.putText(img, f'>> {label} <<', (8, y + 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, c, 2)
            return

        # Show candidate + hold-progress bar
        if self._gest_candidate:
            info    = GESTURE_ACTIONS.get(self._gest_candidate)
            label   = self._gest_candidate.replace('_', ' ').upper()
            color   = info[2] if info else (200, 200, 200)
            prog    = min(1.0, self._gest_hold / GESTURE_HOLD_FRAMES)
            bar_w   = 100
            fill    = int(bar_w * prog)
            cv2.putText(img, label, (8, y + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
            cv2.rectangle(img, (8, y + 16), (8 + bar_w, y + 24), (50, 50, 50), -1)
            if fill > 0:
                cv2.rectangle(img, (8, y + 16), (8 + fill, y + 24), color, -1)

    def _draw_status_hud(self, img, tracked, holdover):
        h, w = img.shape[:2]

        # ── Calibration overlay ───────────────────────────────────────────────
        if self._calibrating:
            overlay = img.copy()
            cv2.rectangle(overlay, (0, 0), (w, h), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.45, img, 0.55, 0, img)
            cv2.putText(img, 'CALIBRATING', (w // 2 - 95, h // 2 - 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 220, 255), 2)
            if tracked:
                cv2.putText(img, 'Move hand to neutral position, press SPACE',
                            (w // 2 - 195, h // 2 + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 220), 1)
                cv2.putText(img, 'SPACE to confirm',
                            (w // 2 - 90, h // 2 + 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 100), 2)
            else:
                cv2.putText(img, 'Show glove to camera',
                            (w // 2 - 110, h // 2 + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 150, 255), 1)
            return

        # ── Top-left status pills ──────────────────────────────────────────────
        if not tracked:
            track_label, track_color = 'LOST',     (0,   0, 220)
        elif 0 < holdover < HOLDOVER_FRAMES:
            track_label, track_color = 'HOLDOVER', (0, 150, 255)
        else:
            track_label, track_color = 'TRACKING', (0, 200,  80)

        arm_label,  arm_color  = ('PAUSED',  (0, 200, 220)) if self._arm_paused  else ('ARM OK',  (0, 180, 60))
        if self._hand_estop:
            hand_label, hand_color = 'HAND ESTOP', (0, 0, 220)
        elif self._hand_paused:
            hand_label, hand_color = 'HAND PAUSED', (0, 200, 220)
        else:
            hand_label, hand_color = 'HAND OK', (0, 180, 60)

        pills = [(track_label, track_color), (arm_label, arm_color), (hand_label, hand_color)]
        x = 8
        for label, color in pills:
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(img, (x - 4, 6), (x + tw + 4, 6 + th + 8), color, -1)
            cv2.putText(img, label, (x, 6 + th + 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            x += tw + 16

        # ── Bottom-right key legend ────────────────────────────────────────────
        legend = ['--- KEYS (click window) ---',
                  '1/2: wrist up/dn',
                  '3/4: elbow out/in',
                  '5/6: palm cw/ccw',
                  'P: pause arm',
                  'O: pause hand',
                  'L: hand estop',
                  'X: cut power',
                  'C: recalibrate',
                  'Q: park + quit']
        lx = w - 140
        ly = h - len(legend) * 16 - 6
        for i, line in enumerate(legend):
            color = (120, 220, 255) if i == 0 else (180, 180, 180)
            cv2.putText(img, line, (lx, ly + i * 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1)

        # ── Last key pressed flash ─────────────────────────────────────────────
        if self._last_key_age > 0:
            self._last_key_age -= 1
            alpha = min(1.0, self._last_key_age / 20.0)
            c = tuple(int(255 * alpha) for _ in range(3))
            cv2.putText(img, f'KEY: {self._last_key_str}',
                        (lx, ly - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 2)

    def _draw_palm_marker(self, img):
        """Blue dot at the current tracked palm position; turns yellow at neutral."""
        if self._last_palm_px is None:
            return
        h, w = img.shape[:2]
        px, py = self._last_palm_px
        dist = math.sqrt((px - w // 2) ** 2 + (py - h // 2) ** 2)
        at_neutral = dist < 30  # pixels

        radius = 12

        color_fill  = (0, 220, 220) if at_neutral else (220, 100,   0)
        color_ring  = (0, 255, 255) if at_neutral else (255, 160,   0)
        cv2.circle(img, (px, py), radius,     color_fill, -1)
        cv2.circle(img, (px, py), radius + 2, color_ring,  2)

        if at_neutral:
            cv2.putText(img, 'NEUTRAL POSITION REACHED',
                        (w // 2 - 130, h // 2 - 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    def _draw_center_marker(self, img):
        """Red crosshair at the image center — neutral/home of the configuration space."""
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        r, arm = 8, 18
        cv2.circle(img, (cx, cy), r, (0, 0, 220), 2)
        cv2.line(img, (cx - arm, cy), (cx - r, cy), (0, 0, 220), 2)
        cv2.line(img, (cx + r,   cy), (cx + arm, cy), (0, 0, 220), 2)
        cv2.line(img, (cx, cy - arm), (cx, cy - r), (0, 0, 220), 2)
        cv2.line(img, (cx, cy + r),   (cx, cy + arm), (0, 0, 220), 2)

    def _draw_xyz_overlay(self, img):
        """Draw X/Y/Z position bars and values in the bottom-left corner."""
        h, w = img.shape[:2]

        # Axis display config: (label, value, range_min, range_max, color_bgr)
        # Ranges match the physical arm limits so bar-full = motor at joint limit.
        axes = [
            ('Y lateral', None, -0.30, 0.30, ( 50, 200, 255)),  # → M1
            ('Z height',  None,  0.10, 0.45, ( 80, 220,  80)),  # → M2
        ]

        if self._last_xyz is not None:
            _, ay, az = self._last_xyz
            axes[0] = (axes[0][0], ay, *axes[0][2:])
            axes[1] = (axes[1][0], az, *axes[1][2:])

        bar_w   = 120
        bar_h   = 10
        pad     = 6
        x0      = 10
        y_start = h - (len(axes) * (bar_h + pad + 14)) - 10

        for i, (label, val, lo, hi, color) in enumerate(axes):
            y = y_start + i * (bar_h + pad + 14)

            # Background track
            cv2.rectangle(img, (x0, y), (x0 + bar_w, y + bar_h), (50, 50, 50), -1)

            if val is not None:
                t = max(0.0, min(1.0, (val - lo) / (hi - lo)))
                fill = int(bar_w * t)
                cv2.rectangle(img, (x0, y), (x0 + fill, y + bar_h), color, -1)
                # Marker line
                cv2.line(img, (x0 + fill, y - 2), (x0 + fill, y + bar_h + 2), (255, 255, 255), 2)
                val_str = f'{val:+.3f}'
            else:
                val_str = '---'

            cv2.putText(img, f'{label}: {val_str}',
                        (x0, y + bar_h + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1)

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


KEY_DEBOUNCE_S = 0.15   # minimum seconds between same key firing

def main():
    rclpy.init()
    node = GloveTrackerNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    _key_last = {}
    try:
        while rclpy.ok() and not node._quit:
            frame = node.get_display_frame()
            if frame is not None:
                cv2.imshow('Glove Tracker', frame)
            key = cv2.waitKey(16) & 0xFF   # ~60 Hz, main thread owns the window
            if key != 0xFF:
                now = time.monotonic()
                if now - _key_last.get(key, 0.0) >= KEY_DEBOUNCE_S:
                    _key_last[key] = now
                    node._handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        node.hands.close()
        cv2.destroyAllWindows()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
