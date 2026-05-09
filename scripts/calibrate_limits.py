#!/usr/bin/env python3
"""Live motor position readout for joint limit calibration.
Motors must be stopped (no torque) so joints can be backdriven manually.
Run this, then move each joint to its limits and record the values shown.
"""
import asyncio
import moteus

MOTOR_IDS = [1, 2, 3, 4, 5]

async def main():
    controllers = {i: moteus.Controller(id=i) for i in MOTOR_IDS}

    print("Live motor positions — move joints manually and record values.")
    print("Ctrl+C to stop.\n")

    while True:
        line_parts = []
        for i in MOTOR_IDS:
            try:
                r = await asyncio.wait_for(controllers[i].query(), timeout=0.3)
                pos = r.values[moteus.Register.POSITION]
                line_parts.append(f"M{i}: {pos:+.4f} ({pos*360:+6.1f}°)")
            except asyncio.TimeoutError:
                line_parts.append(f"M{i}: TIMEOUT")
            except Exception as e:
                line_parts.append(f"M{i}: ERR")
        print("  |  ".join(line_parts), end="\r", flush=True)
        await asyncio.sleep(0.1)

try:
    asyncio.run(main())
except KeyboardInterrupt:
    print("\nDone.")
