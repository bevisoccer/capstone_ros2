#!/usr/bin/env python3
"""
Glove colour & detection tuner — live parameter adjustment GUI.

Subscribes to /camera/color/image_raw and shows a split view:
  Left  : original camera feed  (or mask preview when mask is on)
  Right : exact image MediaPipe receives + hand landmarks overlay

When mask is ON the pipeline matches glove_tracker_node.py exactly:
  mask white pixels → colorize to SKIN_HUE/SKIN_SAT → MediaPipe

OpenCV trackbars let you adjust every parameter in real time.
  P  — print current values as copy-pasteable Python constants
  R  — reset all trackbars to the defaults from glove_tracker_node.py
  Q  — quit

Run:
  source ~/ros2_ws/install/setup.bash
  python3 ~/ros2_ws/glove_tuner.py
"""

import math
import threading
import time

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# ── Defaults (match glove_tracker_node.py) ────────────────────────────────────
DEFAULT_HUE_OFFSET   = 158
DEFAULT_SAT_BOOST    = 100
DEFAULT_VAL_OFFSET   = 56
DEFAULT_DETECT_W     = 224
DEFAULT_DETECT_H     = 168
DEFAULT_DET_CONF     = 10   # × 0.01 → 0.10
DEFAULT_TRK_CONF     = 10   # × 0.01 → 0.10

# ── White-glove mask defaults ─────────────────────────────────────────────────
DEFAULT_MASK_ENABLED  = 1    # 0=off 1=on
DEFAULT_MASK_VAL_MIN  = 100  # pixels darker than this → blacked out
DEFAULT_MASK_SAT_MAX  = 50   # pixels more saturated than this → blacked out (not white)
DEFAULT_SKIN_HUE      = 3    # hue to colorize white-glove pixels (skin ≈ 5–15)
DEFAULT_SKIN_SAT      = 110  # saturation to colorize white-glove pixels

# ── Trackbar offset so negative values are representable ──────────────────────
HUE_OFFSET_BIAS  = 179   # trackbar range 0-358, real = trackbar - 179
BOOST_BIAS       = 255   # trackbar range 0-510, real = trackbar - 255

WINDOW = 'Glove Tuner  |  left=original   right=processed+landmarks   P=print  R=reset  Q=quit'

mp_hands = mp.solutions.hands
mp_draw  = mp.solutions.drawing_utils


# ── Helpers ───────────────────────────────────────────────────────────────────

def apply_hue_sat(img_bgr, hue_offset, sat_boost, val_offset):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV).astype(np.int32)
    hsv[:, :, 0] = np.clip(hsv[:, :, 0] + hue_offset, 0, 179)
    hsv[:, :, 1] = np.clip(hsv[:, :, 1] + sat_boost,  0, 255)
    hsv[:, :, 2] = np.clip(hsv[:, :, 2] + val_offset,  0, 255)
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)


def apply_white_mask(img_bgr, val_min, sat_max, skin_hue, skin_sat):
    """Mask white glove pixels and colorize them to skin tone for MediaPipe.
    Matches glove_tracker_node.py apply_white_mask exactly."""
    hsv  = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, val_min), (179, sat_max, 255))
    hsv[:, :, 0] = np.where(mask > 0, skin_hue, 0)
    hsv[:, :, 1] = np.where(mask > 0, skin_sat, 0)
    hsv[:, :, 2] = np.where(mask > 0, hsv[:, :, 2], 0)
    return cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)


def apply_white_mask_preview(img_bgr, val_min, sat_max):
    """For the left panel: show which pixels are masked (original colors)."""
    hsv  = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, val_min), (179, sat_max, 255))
    return cv2.bitwise_and(img_bgr, img_bgr, mask=mask)


def prepare_for_mediapipe(img_bgr, hue_offset, sat_boost, val_offset, det_w, det_h,
                          mask_on, val_min, sat_max, skin_hue, skin_sat):
    if mask_on:
        # Match tracker: mask on raw image, then resize
        modified = apply_white_mask(img_bgr, val_min, sat_max, skin_hue, skin_sat)
    else:
        modified = apply_hue_sat(img_bgr, hue_offset, sat_boost, val_offset)
    if det_w > 0 and det_h > 0:
        modified = cv2.resize(modified, (det_w, det_h), interpolation=cv2.INTER_LINEAR)
    return modified  # BGR — caller converts to RGB


# ── ROS node ──────────────────────────────────────────────────────────────────

class TunerNode(Node):
    def __init__(self):
        super().__init__('glove_tuner')
        self.bridge      = CvBridge()
        self.latest_img  = None
        self.lock        = threading.Lock()
        self.create_subscription(Image, '/camera/color/image_raw',
                                 self._img_cb, 1)
        self.get_logger().info('Subscribed to /camera/color/image_raw')

    def _img_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self.lock:
            self.latest_img = img

    def get_frame(self):
        with self.lock:
            return None if self.latest_img is None else self.latest_img.copy()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = TunerNode()

    # Spin ROS in a background thread so the main thread owns the GUI
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # ── MediaPipe (rebuilt when confidence params change) ─────────────────────
    hands_state = {'det': DEFAULT_DET_CONF / 100.0,
                   'trk': DEFAULT_TRK_CONF / 100.0,
                   'detector': None}

    def make_detector(det, trk):
        if hands_state['detector'] is not None:
            hands_state['detector'].close()
        hands_state['detector'] = mp_hands.Hands(
            static_image_mode=False,
            model_complexity=0,
            max_num_hands=1,
            min_detection_confidence=det,
            min_tracking_confidence=trk,
        )
        hands_state['det'] = det
        hands_state['trk'] = trk

    make_detector(DEFAULT_DET_CONF / 100.0, DEFAULT_TRK_CONF / 100.0)

    # ── OpenCV window & trackbars ─────────────────────────────────────────────
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)

    def tb(name, val, max_val):
        cv2.createTrackbar(name, WINDOW, val, max_val, lambda _: None)

    # Colour correction  (trackbar stores value + bias)
    tb('HUE offset  (-179..+179)', DEFAULT_HUE_OFFSET + HUE_OFFSET_BIAS, 358)
    tb('SAT boost   (-255..+255)', DEFAULT_SAT_BOOST  + BOOST_BIAS,       510)
    tb('VAL offset  (-255..+255)', DEFAULT_VAL_OFFSET + BOOST_BIAS,       510)

    # Detection resolution  (0 = use original size)
    tb('Detect W  (0=off)', DEFAULT_DETECT_W, 640)
    tb('Detect H  (0=off)', DEFAULT_DETECT_H, 480)

    # MediaPipe confidence  (stored as 0-100 → divide by 100)
    tb('Det confidence  (×0.01)', DEFAULT_DET_CONF, 100)
    tb('Trk confidence  (×0.01)', DEFAULT_TRK_CONF, 100)

    # White-glove mask
    tb('Mask enable  (0=off 1=on)', DEFAULT_MASK_ENABLED, 1)
    tb('Mask VAL min  (brightness)', DEFAULT_MASK_VAL_MIN, 255)
    tb('Mask SAT max  (whiteness)',  DEFAULT_MASK_SAT_MAX, 255)
    tb('Skin HUE  (colorize glove, skin=5-15)', DEFAULT_SKIN_HUE, 179)
    tb('Skin SAT  (colorize glove)',            DEFAULT_SKIN_SAT, 255)

    def read_params():
        hue      = cv2.getTrackbarPos('HUE offset  (-179..+179)', WINDOW) - HUE_OFFSET_BIAS
        sat      = cv2.getTrackbarPos('SAT boost   (-255..+255)', WINDOW) - BOOST_BIAS
        val      = cv2.getTrackbarPos('VAL offset  (-255..+255)', WINDOW) - BOOST_BIAS
        dw       = cv2.getTrackbarPos('Detect W  (0=off)',        WINDOW)
        dh       = cv2.getTrackbarPos('Detect H  (0=off)',        WINDOW)
        dc       = cv2.getTrackbarPos('Det confidence  (×0.01)',  WINDOW) / 100.0
        tc       = cv2.getTrackbarPos('Trk confidence  (×0.01)',  WINDOW) / 100.0
        mask_on  = cv2.getTrackbarPos('Mask enable  (0=off 1=on)', WINDOW) == 1
        val_min  = cv2.getTrackbarPos('Mask VAL min  (brightness)', WINDOW)
        sat_max  = cv2.getTrackbarPos('Mask SAT max  (whiteness)',  WINDOW)
        skin_hue = cv2.getTrackbarPos('Skin HUE  (colorize glove, skin=5-15)', WINDOW)
        skin_sat = cv2.getTrackbarPos('Skin SAT  (colorize glove)',            WINDOW)
        return hue, sat, val, dw, dh, dc, tc, mask_on, val_min, sat_max, skin_hue, skin_sat

    def reset_trackbars():
        cv2.setTrackbarPos('HUE offset  (-179..+179)', WINDOW, DEFAULT_HUE_OFFSET + HUE_OFFSET_BIAS)
        cv2.setTrackbarPos('SAT boost   (-255..+255)', WINDOW, DEFAULT_SAT_BOOST  + BOOST_BIAS)
        cv2.setTrackbarPos('VAL offset  (-255..+255)', WINDOW, DEFAULT_VAL_OFFSET + BOOST_BIAS)
        cv2.setTrackbarPos('Detect W  (0=off)',        WINDOW, DEFAULT_DETECT_W)
        cv2.setTrackbarPos('Detect H  (0=off)',        WINDOW, DEFAULT_DETECT_H)
        cv2.setTrackbarPos('Det confidence  (×0.01)',  WINDOW, DEFAULT_DET_CONF)
        cv2.setTrackbarPos('Trk confidence  (×0.01)',  WINDOW, DEFAULT_TRK_CONF)
        cv2.setTrackbarPos('Mask enable  (0=off 1=on)', WINDOW, DEFAULT_MASK_ENABLED)
        cv2.setTrackbarPos('Mask VAL min  (brightness)', WINDOW, DEFAULT_MASK_VAL_MIN)
        cv2.setTrackbarPos('Mask SAT max  (whiteness)',  WINDOW, DEFAULT_MASK_SAT_MAX)
        cv2.setTrackbarPos('Skin HUE  (colorize glove, skin=5-15)', WINDOW, DEFAULT_SKIN_HUE)
        cv2.setTrackbarPos('Skin SAT  (colorize glove)',            WINDOW, DEFAULT_SKIN_SAT)

    print('Glove Tuner running.  Click glove=sample  A=auto-scan  C=clear samples  P=print  R=reset  Q=quit')

    status_msg  = ''
    last_frame  = None
    samples     = []   # list of (px, py, V, S) from each click

    def _update_thresholds_from_samples():
        if not samples:
            return
        vs = [s[2] for s in samples]
        ss = [s[3] for s in samples]
        new_val_min = max(0,   min(vs) - 30)
        new_sat_max = min(255, max(ss) + 25)
        cv2.setTrackbarPos('Mask VAL min  (brightness)', WINDOW, new_val_min)
        cv2.setTrackbarPos('Mask SAT max  (whiteness)',  WINDOW, new_sat_max)
        cv2.setTrackbarPos('Mask enable  (0=off 1=on)', WINDOW, 1)
        return new_val_min, new_sat_max

    def on_mouse(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or last_frame is None:
            return
        h_f, w_f = last_frame.shape[:2]
        if x >= w_f:   # ignore clicks on right panel
            return
        px = np.clip(x, 0, w_f - 1)
        py = np.clip(y, 0, h_f - 1)
        hsv_frame = cv2.cvtColor(last_frame, cv2.COLOR_BGR2HSV)
        x0, x1 = max(0, px - 4), min(w_f, px + 5)
        y0, y1 = max(0, py - 4), min(h_f, py + 5)
        patch  = hsv_frame[y0:y1, x0:x1]
        v_med  = int(np.median(patch[:, :, 2]))
        s_med  = int(np.median(patch[:, :, 1]))
        samples.append((px, py, v_med, s_med))
        result = _update_thresholds_from_samples()
        nonlocal status_msg
        if result:
            vm, sm = result
            status_msg = (f'{len(samples)} samples — V range {min(s[2] for s in samples)}–{max(s[2] for s in samples)}'
                          f'  S range {min(s[3] for s in samples)}–{max(s[3] for s in samples)}'
                          f'  → val_min={vm} sat_max={sm}')
        print(status_msg)

    cv2.setMouseCallback(WINDOW, on_mouse)

    while True:
        frame = node.get_frame()
        if frame is None:
            placeholder = np.zeros((240, 640, 3), dtype=np.uint8)
            cv2.putText(placeholder, 'Waiting for /camera/color/image_raw ...',
                        (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
            cv2.imshow(WINDOW, placeholder)
            key = cv2.waitKey(100) & 0xFF
            if key == ord('q'):
                break
            continue

        last_frame = frame
        hue, sat, val, dw, dh, det_c, trk_c, mask_on, val_min, sat_max, skin_hue, skin_sat = read_params()

        # Rebuild MediaPipe detector only when confidence params actually change
        if (abs(det_c - hands_state['det']) > 0.005 or
                abs(trk_c - hands_state['trk']) > 0.005):
            make_detector(det_c, trk_c)

        # ── Left panel: original or mask coverage preview ─────────────────────
        left = frame.copy()
        if mask_on:
            mask_preview = apply_white_mask_preview(frame, val_min, sat_max)
            cv2.putText(mask_preview, f'WHITE MASK COVERAGE  ({len(samples)} samples — click glove)', (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 220), 2)
            left = mask_preview
        else:
            cv2.putText(left, f'ORIGINAL  ({len(samples)} samples — click glove)', (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        # Draw sample dots
        for sx, sy, sv, ss in samples:
            cv2.circle(left, (sx, sy), 5, (0, 255, 0), -1)
            cv2.circle(left, (sx, sy), 6, (0, 0, 0),   1)

        # ── Right panel: exact MediaPipe input + landmarks ────────────────────
        mp_bgr = prepare_for_mediapipe(frame, hue, sat, val, dw, dh,
                                       mask_on, val_min, sat_max, skin_hue, skin_sat)
        right = mp_bgr.copy()

        mp_input = cv2.cvtColor(mp_bgr, cv2.COLOR_BGR2RGB)

        results = hands_state['detector'].process(mp_input)

        detected = False
        if results.multi_hand_landmarks:
            detected = True
            lm = results.multi_hand_landmarks[0]
            mp_draw.draw_landmarks(right, lm, mp_hands.HAND_CONNECTIONS)

        status_text  = 'DETECTED' if detected else 'no hand'
        status_color = (0, 220, 0) if detected else (0, 60, 220)
        cv2.putText(right, f'PROCESSED  [{status_text}]', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

        lines = [
            f'skin_hue={skin_hue}  skin_sat={skin_sat}',
            f'val_min={val_min}  sat_max={sat_max}',
        ]
        for i, line in enumerate(lines):
            cv2.putText(right, line, (10, right.shape[0] - 40 + i * 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)

        # ── Combine side by side — both panels at original frame size ────────
        h_l, w_l = left.shape[:2]
        if right.shape[:2] != (h_l, w_l):
            right = cv2.resize(right, (w_l, h_l), interpolation=cv2.INTER_NEAREST)

        combined = np.hstack([left, right])

        if status_msg:
            cv2.putText(combined, status_msg, (10, combined.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

        cv2.imshow(WINDOW, combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            samples.clear()
            status_msg = 'Samples cleared.'
            print(status_msg)
        elif key == ord('r'):
            reset_trackbars()
            samples.clear()
            status_msg = 'Reset to defaults.'
            print(status_msg)
        elif key == ord('a'):
            # ── Auto-scan SKIN_HUE 0-30, pick value that detects hand ─────────
            print('Auto-scanning SKIN_HUE 0–30 (using low confidence detector)...')
            scan_frame = node.get_frame()
            if scan_frame is not None:
                _, _, _, dw2, dh2, _, _, _, vm2, sm2, _, ss2 = read_params()
                # Use a permissive detector for the scan
                scan_det = mp_hands.Hands(
                    static_image_mode=True,
                    model_complexity=0,
                    max_num_hands=1,
                    min_detection_confidence=0.10,
                    min_tracking_confidence=0.10,
                )
                best_hue, best_score = skin_hue, 0
                hits = []
                for h_try in range(0, 31):
                    test_bgr = apply_white_mask(scan_frame, vm2, sm2, h_try, ss2)
                    if dw2 > 0 and dh2 > 0:
                        test_bgr = cv2.resize(test_bgr, (dw2, dh2), interpolation=cv2.INTER_LINEAR)
                    test_rgb = cv2.cvtColor(test_bgr, cv2.COLOR_BGR2RGB)
                    r = scan_det.process(test_rgb)
                    if r.multi_hand_landmarks:
                        hits.append(h_try)
                        if len(r.multi_hand_landmarks[0].landmark) > best_score:
                            best_score = len(r.multi_hand_landmarks[0].landmark)
                            best_hue   = h_try
                scan_det.close()
                if hits:
                    cv2.setTrackbarPos('Skin HUE  (colorize glove, skin=5-15)', WINDOW, best_hue)
                    status_msg = f'Auto-scan: detected at HUE={hits}  →  set to {best_hue}'
                else:
                    status_msg = 'Auto-scan: no detection at any HUE — lower Det confidence or re-sample glove'
                print(status_msg)
            else:
                status_msg = 'No frame for auto-scan.'
        elif key == ord('p'):
            hue, sat, val, dw, dh, det_c, trk_c, mask_on, val_min, sat_max, skin_hue, skin_sat = read_params()
            print('\n# ── Tuned parameters ───────────────────────────────')
            print(f'HUE_OFFSET    = {hue}')
            print(f'SAT_BOOST     = {sat}')
            print(f'VAL_OFFSET    = {val}')
            print(f'DETECT_WIDTH  = {dw}')
            print(f'DETECT_HEIGHT = {dh}')
            print(f'# MediaPipe')
            print(f'min_detection_confidence = {det_c:.2f}')
            print(f'min_tracking_confidence  = {trk_c:.2f}')
            print(f'# White glove mask')
            print(f'MASK_ENABLED  = {mask_on}')
            print(f'MASK_VAL_MIN  = {val_min}')
            print(f'MASK_SAT_MAX  = {sat_max}')
            print(f'SKIN_HUE      = {skin_hue}')
            print(f'SKIN_SAT      = {skin_sat}')
            print('# ──────────────────────────────────────────────────\n')

    # ── Cleanup ───────────────────────────────────────────────────────────────
    if hands_state['detector']:
        hands_state['detector'].close()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
