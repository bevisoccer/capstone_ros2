import numpy as np
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from ikpy.chain import Chain

share_dir = Path(get_package_share_directory("hand_control"))
robot_arm = Chain.from_urdf_file(str(share_dir / "five_dof_arm_new.urdf"))

# What angle gives z=0.16 at origin?
# shoulder at z=0.14, tip at z=0.16, so z_contribution = 0.02
# sin(angle) = 0.02/0.45 = 0.044, angle = 2.5 degrees
print("Testing small motor 2 angles to match real z=0.16:")
for m2 in [0, -5, -10, -15, -20, -49.18]:
    rad = [np.deg2rad(d) for d in [90.0, m2, 4.03, 3.06]]
    joints = [0.0] + list(rad) + [0.0, 0.0]
    T = robot_arm.forward_kinematics(joints)
    print(f"  motor2={m2:7.2f}deg: x={T[0,3]:.4f} y={T[1,3]:.4f} z={T[2,3]:.4f}")
