#!/usr/bin/env python3
"""
lift_m2_test.py — Find the correct lift direction for M2.

The workspace calibration shows:
  HEIGHT_M2_HIGH_RAW = -0.2841  (arm high)
  HEIGHT_M2_LOW_RAW  = -0.4821  (arm low)

More negative = LOWER arm. So commanding M2 from +0.0183 toward -0.3138
goes in the LOWERING direction — probably driving into the table.
This script tests the POSITIVE direction first to find what actually lifts.

  cd ~/ros2_ws && python3 lift_m2_test.py
"""

import asyncio
from hand_control.MotorController import MoteusMotorController

MOTOR_IDS = [2]

VELOCITY = 0.08
ACCEL    = 0.10
TORQUE   = 20.0


async def read_m2(mc):
    pos = await mc.get_raw_motor_positions(2)
    print(f"  M2 current: {pos:.4f}")
    return pos


async def nudge(mc, target, label):
    print(f"\n--- {label}  (target={target:+.4f}) ---")
    before = await read_m2(mc)
    await mc.set_motor_position(2, target, VELOCITY, ACCEL, TORQUE)
    await asyncio.sleep(2.5)
    after = await read_m2(mc)
    delta = after - before
    print(f"  delta={delta:+.4f}  {'MOVED' if abs(delta) > 0.01 else 'no movement'}")
    return after


async def main():
    mc = MoteusMotorController(MOTOR_IDS, {})

    print("=== M2 Direction Test ===")
    print("Arm should be flat on the table. Only M2 is commanded.")
    print("Watch/listen: does the arm lift or press into the table?\n")

    # Query full state to check for faults
    ctrl = mc.controllers[2]
    state = await ctrl.query()
    print("M2 full state:")
    for reg, val in state.values.items():
        print(f"  {reg}: {val}")

    print("\nClearing any fault with set_stop()...")
    await ctrl.set_stop()
    await asyncio.sleep(0.5)

    state2 = await ctrl.query()
    print("M2 state after stop:")
    for reg, val in state2.values.items():
        print(f"  {reg}: {val}")

    input("\nFault cleared. Press Enter to continue with movement tests...")

    start = await read_m2(mc)

    # Test negative direction (current startup code does this)
    input(f"\n[TEST A] Press Enter to nudge M2 NEGATIVE ({start:.4f} → {start-0.05:.4f})")
    print("  Watch: does the arm lift UP or press DOWN?")
    pos_a = await nudge(mc, start - 0.05, "Negative nudge (-0.05)")

    input(f"\nReset back to {start:.4f}. Press Enter...")
    await mc.set_motor_position(2, start, VELOCITY, ACCEL, 10.0)
    await asyncio.sleep(2.0)
    await read_m2(mc)

    # Test positive direction
    input(f"\n[TEST B] Press Enter to nudge M2 POSITIVE ({start:.4f} → {start+0.05:.4f})")
    print("  Watch: does the arm lift UP or press DOWN?")
    pos_b = await nudge(mc, start + 0.05, "Positive nudge (+0.05)")

    print("\n=== Summary ===")
    print("Which direction lifted the arm? That's the correct lift direction.")
    print("If POSITIVE lifted: all M2 constants in arm_control_node.py need to be negated.")
    print("If NEGATIVE lifted: the issue is torque/fault, not direction.")

    print("\n[DONE] Stopping M2.")
    await mc.stop_motor(2)


if __name__ == "__main__":
    asyncio.run(main())
