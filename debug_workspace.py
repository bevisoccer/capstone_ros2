#!/usr/bin/env python3
"""
debug_workspace.py — Move arm directly to calibrated workspace extremes.

Commands motors to the exact raw positions recorded during calibration,
bypassing all mapping functions to verify the limits are correct.

Run:
  cd ~/ros2_ws && python3 debug_workspace.py
"""

import asyncio
from hand_control.MotorController import MoteusMotorController

MOTOR_IDS = [1, 2, 3, 4, 5]
VELOCITY  = 0.15
ACCEL     = 0.08
TORQUE    = 12.0
SETTLE    = 4.0

# ── Calibrated raw positions (absolute Moteus positions) ──────────────────────
POSES = {
    "Lowest / Nearest reach (arm forward, close to table)": {
        2: -0.4845,
        4:  0.0375,
        5:  0.0106,
    },
    "Highest (arm straight up)": {
        2: -0.2981,
        4:  0.0167,
        5: -0.0292,
    },
    "Elbow folded (hand far from camera)": {
        2: -0.3616,
        4:  0.2798,
        5:  0.0556,
    },
}


async def move_to(mc, targets: dict, label: str):
    print(f"\n>>> {label}")
    for motor_id, raw in sorted(targets.items()):
        print(f"    Commanding M{motor_id} → {raw:.4f}")
        await mc.set_motor_position(motor_id, raw, VELOCITY, ACCEL, TORQUE)
    await asyncio.sleep(SETTLE)
    print("    Actual positions:")
    for motor_id in sorted(targets.keys()):
        actual    = await mc.get_raw_motor_positions(motor_id)
        commanded = targets[motor_id]
        err       = actual - commanded
        print(f"      M{motor_id}: actual={actual:.4f}  commanded={commanded:.4f}  err={err:+.4f}")


async def main():
    mc = MoteusMotorController(MOTOR_IDS, {})

    print("=== Workspace Debug ===")
    print("Will move through these poses:")
    for label, targets in POSES.items():
        print(f"  {label}")
        for m, v in sorted(targets.items()):
            print(f"    M{m}: {v:.4f}")

    input("\nPress Enter to start (make sure arm has clearance)...")

    for label, targets in POSES.items():
        await move_to(mc, targets, label)
        input("  Press Enter to continue...")

    print("\n[DONE] Stopping motors.")
    await mc.stop_all()


if __name__ == "__main__":
    asyncio.run(main())
