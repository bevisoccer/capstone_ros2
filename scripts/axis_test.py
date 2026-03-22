import asyncio
import subprocess
import time
import json
from datetime import datetime

# Publish to raw_robot_arm_target and read motor responses
import moteus

async def read_motors():
    positions = {}
    for i in [1, 2, 3, 4]:
        c = moteus.Controller(id=i)
        r = await c.query()
        positions[i] = round(r.values[moteus.Register.POSITION] * 360, 2)
    return positions

def pub(x, y, z):
    subprocess.run([
        'ros2', 'topic', 'pub', '--once',
        '/raw_robot_arm_target',
        'hand_interfaces/msg/RobotArmTarget',
        f'{{x: {x}, y: {y}, z: {z}}}'
    ], capture_output=True)

async def test_axis(name, positions, log):
    print(f"\nTesting {name}...")
    results = []
    for x, y, z in positions:
        pub(x, y, z)
        await asyncio.sleep(1.5)  # wait for arm to move
        motors = await read_motors()
        entry = {"xyz": [x, y, z], "motors_deg": motors}
        results.append(entry)
        print(f"  xyz=({x:.3f},{y:.3f},{z:.3f}) -> motors={motors}")
    log[name] = results

async def main():
    log = {}

    # Origin reference
    print("Setting origin pose...")
    pub(0.20, 0.0, 0.24)
    await asyncio.sleep(2.0)
    origin_motors = await read_motors()
    log["origin"] = {"xyz": [0.20, 0.0, 0.24], "motors_deg": origin_motors}
    print(f"Origin motors: {origin_motors}")

    # Test y axis (left/right)
    await test_axis("y_positive", [
        (0.20, 0.04, 0.24),
        (0.20, 0.08, 0.24),
        (0.20, 0.10, 0.24),
    ], log)

    pub(0.20, 0.0, 0.24)
    await asyncio.sleep(2.0)

    await test_axis("y_negative", [
        (0.20, -0.04, 0.24),
        (0.20, -0.08, 0.24),
        (0.20, -0.10, 0.24),
    ], log)

    pub(0.20, 0.0, 0.24)
    await asyncio.sleep(2.0)

    # Test x axis (reach)
    await test_axis("x_positive", [
        (0.22, 0.0, 0.24),
        (0.26, 0.0, 0.24),
        (0.30, 0.0, 0.24),
    ], log)

    pub(0.20, 0.0, 0.24)
    await asyncio.sleep(2.0)

    await test_axis("x_negative", [
        (0.18, 0.0, 0.24),
    ], log)

    # Save log
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = f"/home/bev/ros2_ws/axis_test_{ts}.json"
    with open(path, "w") as f:
        json.dump(log, f, indent=2)
    print(f"\nLog saved to {path}")
    return path

asyncio.run(main())
