import subprocess
import time

def pub(x, y, z):
    subprocess.run([
        'ros2', 'topic', 'pub', '--once',
        '/raw_robot_arm_target',
        'hand_interfaces/msg/RobotArmTarget',
        f'{{x: {x}, y: {y}, z: {z}}}'
    ], capture_output=True)

print("Starting at origin...")
pub(0.20, 0.0, 0.24)
time.sleep(2)

print("y = +0.05 (left)")
pub(0.20, 0.05, 0.24)
time.sleep(2)

print("y = +0.10 (left max)")
pub(0.20, 0.10, 0.24)
time.sleep(2)

print("y = 0.0 (center)")
pub(0.20, 0.0, 0.24)
time.sleep(2)

print("y = -0.05 (right)")
pub(0.20, -0.05, 0.24)
time.sleep(2)

print("y = -0.10 (right max)")
pub(0.20, -0.10, 0.24)
time.sleep(2)

print("y = 0.0 (back to center)")
pub(0.20, 0.0, 0.24)
print("Done")
