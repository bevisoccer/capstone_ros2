import numpy as np
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from ikpy.chain import Chain

share_dir = Path(get_package_share_directory("hand_control"))

# Try a grid of elbow_x and pitch values to find best match
target_x = 0.26
target_z = 0.16
deg = [97.25, -52.99, 5.04, -17.28]
rad = [np.deg2rad(d) for d in deg]

best = None
best_err = 999

for elbow_x in np.arange(0.05, 0.20, 0.01):
    for pitch in np.arange(0.80, 1.80, 0.05):
        # Write temp URDF
        urdf = f"""<robot name="robot_arm">
<link name="base_link"/>
<link name="base_rot"/>
<joint name="base_joint" type="revolute">
  <parent link="base_link"/><child link="base_rot"/>
  <origin xyz="0 0 0.14" rpy="0 0 0"/><axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14"/>
</joint>
<link name="arm1"/>
<joint name="shoulder_joint" type="revolute">
  <parent link="base_rot"/><child link="arm1"/>
  <origin xyz="0.05 0 0" rpy="0 0 -1.5708"/><axis xyz="0 1 0"/>
  <limit lower="-1.832" upper="1.832"/>
</joint>
<link name="arm2"/>
<joint name="elbow_joint" type="revolute">
  <parent link="arm1"/><child link="arm2"/>
  <origin xyz="{elbow_x:.3f} 0 0" rpy="0 {pitch:.3f} 0"/><axis xyz="-1 0 0"/>
  <limit lower="-3.14" upper="3.14"/>
</joint>
<link name="tilt"/>
<joint name="rot_joint" type="revolute">
  <parent link="arm2"/><child link="tilt"/>
  <origin xyz="0.09 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
  <limit lower="-6" upper="6"/>
</joint>
<link name="top"/>
<joint name="tilt_joint" type="revolute">
  <parent link="tilt"/><child link="top"/>
  <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 -1 0"/>
  <limit lower="-1.832" upper="1.832"/>
</joint>
<link name="needle"/>
<joint name="rot2_joint" type="revolute">
  <parent link="top"/><child link="needle"/>
  <origin xyz="0.210 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
  <limit lower="-4" upper="4"/>
</joint>
</robot>"""
        
        with open('/tmp/test_urdf.urdf', 'w') as f:
            f.write(urdf)
        
        try:
            robot = Chain.from_urdf_file('/tmp/test_urdf.urdf')
            joints = [0.0] + list(rad) + [0.0, 0.0]
            T = robot.forward_kinematics(joints)
            x, z = T[0,3], T[2,3]
            err = abs(x - target_x) + abs(z - target_z)
            if err < best_err:
                best_err = err
                best = (elbow_x, pitch, x, T[1,3], z)
        except:
            pass

print(f"Best match: elbow_x={best[0]:.3f} pitch={best[1]:.3f}")
print(f"  FK: x={best[2]:.4f} y={best[3]:.4f} z={best[4]:.4f}")
print(f"  Error: {best_err:.4f}")
