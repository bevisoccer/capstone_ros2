import numpy as np
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from ikpy.chain import Chain

share_dir = Path(get_package_share_directory("hand_control"))
robot_arm = Chain.from_urdf_file(str(share_dir / "five_dof_arm_new.urdf"))

# Actual measured motor positions at origin pose
deg = [97.25, -52.99, 5.04, -17.28]
rad = [np.deg2rad(d) for d in deg]
joints = [0.0] + list(rad) + [0.0, 0.0]
T = robot_arm.forward_kinematics(joints)
print(f"FK at actual origin pose:")
print(f"  x={T[0,3]:.4f}  y={T[1,3]:.4f}  z={T[2,3]:.4f}")
print(f"  Expected: x~0.26, y~0.0, z~0.16")
