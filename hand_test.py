#!/usr/bin/env python3
"""
hand_test.py — Interactive test for the robotic hand (Teensy over serial).

  cd ~/ros2_ws && python3 hand_test.py [--port /dev/ttyACM0]

Commands:
  o              open all fingers (home)
  c              close all fingers
  1-5            curl individual finger  (1=thumb 2=index 3=middle 4=ring 5=pinky)
  ENTER          re-open that finger
  s              show live FSR + current snapshot
  q              quit (homes first)
"""

import sys
import time
import argparse
import threading

sys.path.insert(0, '/home/bev/ros2_ws/install/hand_control/lib/python3.10/site-packages')

from hand_control.hand_control_interface import HandController, FINGER_ORDER

FINGER_NAMES = FINGER_ORDER   # ["thumb","index","middle","ring","pinky"]
CLOSED_ANGLE = 0.0
OPEN_ANGLE   = 150.0


def print_state(ctrl):
    snap = ctrl.get_state()
    print("\n── Finger state ──────────────────────────────")
    for i, finger in enumerate(FINGER_NAMES):
        fsr_ch   = next((k for k, v in snap["fsr"].items()   if snap["fsr"][k].finger   == finger), None)
        ina_ch   = next((k for k, v in snap["ina"].items()   if snap["ina"][k].finger   == finger), None)
        fsr_pct  = f"{snap['fsr'][fsr_ch].pct:.0f}%" if fsr_ch is not None else "  ?"
        cur_ma   = f"{snap['ina'][ina_ch].current_ma:.0f}mA" if ina_ch is not None else "    ?"
        print(f"  {i+1} {finger:<7}  FSR={fsr_pct:>5}  current={cur_ma:>7}")
    print()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    ctrl = HandController(port=args.port, baud=args.baud)

    # Print INA/FSR updates as they arrive
    def on_fsr(fields):
        pass  # suppress noise — use 's' to snapshot

    def on_ina(fields):
        pass

    ctrl.on_fsr_update = on_fsr
    ctrl.on_ina_update = on_ina

    print(f"Connecting to Teensy on {args.port}...")
    if not ctrl.connect():
        print("ERROR: could not open serial port.")
        sys.exit(1)

    print("Connected. Homing all fingers...")
    ctrl.home_all()
    time.sleep(2.0)
    print("Ready.\n")

    print("Commands:  o=open all  c=close all  1-5=curl finger  s=sensor snapshot  q=quit\n")

    active_finger = None

    while True:
        try:
            cmd = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if cmd == 'q':
            break
        elif cmd == 'o':
            print("Opening all...")
            for finger in FINGER_NAMES:
                ctrl.move_finger_curve1(finger, OPEN_ANGLE)
                if finger != "thumb":
                    ctrl.move_finger_curve2(finger, OPEN_ANGLE)
            active_finger = None
        elif cmd == 'c':
            print("Closing all...")
            for finger in FINGER_NAMES:
                ctrl.move_finger_curve1(finger, CLOSED_ANGLE)
                if finger != "thumb":
                    ctrl.move_finger_curve2(finger, CLOSED_ANGLE)
        elif cmd in ('1', '2', '3', '4', '5'):
            idx = int(cmd) - 1
            active_finger = FINGER_NAMES[idx]
            print(f"Curling {active_finger}...")
            ctrl.move_finger_curve1(active_finger, CLOSED_ANGLE)
            if active_finger != "thumb":
                ctrl.move_finger_curve2(active_finger, CLOSED_ANGLE)
        elif cmd == '' and active_finger:
            print(f"Opening {active_finger}...")
            ctrl.move_finger_curve1(active_finger, OPEN_ANGLE)
            if active_finger != "thumb":
                ctrl.move_finger_curve2(active_finger, OPEN_ANGLE)
            active_finger = None
        elif cmd == 's':
            time.sleep(0.1)  # let streams settle
            print_state(ctrl)
        else:
            print("  o=open all  c=close all  1-5=curl finger  ENTER=open last  s=sensors  q=quit")

    print("Opening and disconnecting...")
    for finger in FINGER_NAMES:
        ctrl.move_finger_curve1(finger, OPEN_ANGLE)
        if finger != "thumb":
            ctrl.move_finger_curve2(finger, OPEN_ANGLE)
    time.sleep(2.0)
    ctrl.disconnect()
    print("Done.")
    sys.exit(0)


if __name__ == "__main__":
    main()
