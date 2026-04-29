#!/usr/bin/env python3
"""
motor_test.py — Simple interactive motor tester.

Commands one motor at a time to a raw position and reads back actual position.

Run:
  cd ~/ros2_ws && python3 motor_test.py
"""

import asyncio
from hand_control.MotorController import MoteusMotorController

MOTOR_IDS = [1, 2, 3, 4, 5]
VELOCITY  = 0.10
ACCEL     = 0.06
TORQUE    = 8.0
SETTLE    = 2.0


async def main():
    mc = MoteusMotorController(MOTOR_IDS, {})

    print("\n=== Motor Test ===")
    print("Commands:")
    print("  r          — read all motor positions")
    print("  m <id> <pos> — move motor to raw position (e.g. 'm 2 -0.05')")
    print("  s          — stop all motors")
    print("  q          — quit")
    print("==================\n")

    while True:
        cmd = input(">> ").strip().lower()

        if cmd == "r":
            for mid in MOTOR_IDS:
                pos = await mc.get_raw_motor_positions(mid)
                print(f"  M{mid}: {pos:.4f}")

        elif cmd.startswith("m "):
            parts = cmd.split()
            if len(parts) != 3:
                print("  Usage: m <motor_id> <raw_pos>")
                continue
            try:
                mid = int(parts[1])
                pos = float(parts[2])
            except ValueError:
                print("  Invalid motor id or position")
                continue
            print(f"  Moving M{mid} → {pos:.4f}...")
            await mc.set_motor_position(mid, pos, VELOCITY, ACCEL, TORQUE)
            await asyncio.sleep(SETTLE)
            actual = await mc.get_raw_motor_positions(mid)
            print(f"  M{mid}: actual={actual:.4f}  commanded={pos:.4f}  err={actual-pos:+.4f}")

        elif cmd == "s":
            await mc.stop_all()
            print("  Motors stopped.")

        elif cmd in ("q", "quit"):
            await mc.stop_all()
            break

        else:
            print("  Unknown command")


if __name__ == "__main__":
    asyncio.run(main())
