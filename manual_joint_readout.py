import asyncio
import math
import numpy as np

from hand_control.MotorController import MoteusMotorController


MOTOR_IDS = [1, 2, 3, 4, 5]


async def hold_current_pose(mc: MoteusMotorController):
    print("\n[INFO] Holding current pose...")
    for motor_id in sorted(mc.controllers.keys()):
        raw = await mc.get_raw_motor_positions(motor_id)
        current_deg = (raw - mc.zero_offsets.get(motor_id, 0.0)) * 360.0
        print(f"[INFO] Motor {motor_id}: holding at {current_deg:.2f} deg")
        await mc.set_motor_angle(
            motor_id,
            current_deg,
            velocity_limit=0.05,
            accel_limit=1,
            torque_limit=10,
        )


async def print_joint_positions(mc: MoteusMotorController):
    print("\nCurrent joint positions relative to calibrated zero:")
    for motor_id in sorted(mc.controllers.keys()):
        raw = await mc.get_raw_motor_positions(motor_id)
        zero = mc.zero_offsets.get(motor_id, 0.0)
        rel_deg = (raw - zero) * 360.0
        print(
            f"  Motor {motor_id}: "
            f"raw={raw:.4f}, zero={zero:.4f}, relative={rel_deg:.2f} deg"
        )


async def main():
    mc = MoteusMotorController(MOTOR_IDS)

    print("[INFO] Calibrating zero offsets from CURRENT pose...")
    print("[INFO] Put the arm in your neutral/reference pose before continuing.\n")
    input("Press Enter when ready...")

    await mc.calibrate_zero_offsets()
    await print_joint_positions(mc)

    print("\nCommands:")
    print("  r = read current joint positions")
    print("  h = hold current pose")
    print("  s = stop/release motors")
    print("  q = quit")

    while True:
        cmd = input("\nCommand [r/h/s/q]: ").strip().lower()

        if cmd == "r":
            await print_joint_positions(mc)

        elif cmd == "h":
            await hold_current_pose(mc)

        elif cmd == "s":
            print("\n[INFO] Stopping all motors...")
            await mc.stop_all()

        elif cmd == "q":
            print("\n[INFO] Quitting. Stopping all motors first...")
            await mc.stop_all()
            break

        else:
            print("[WARN] Unknown command")


if __name__ == "__main__":
    asyncio.run(main())
