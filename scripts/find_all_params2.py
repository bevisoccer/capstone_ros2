import numpy as np
from ikpy.chain import Chain

target_x, target_z = 0.26, 0.16
deg = [97.25, -52.99, 5.04, -17.28]
rad = [np.deg2rad(d) for d in deg]

best = None
best_err = 999

for shoulder_offset in np.arange(0.03, 0.12, 0.01):
    for elbow_x in np.arange(0.10, 0.22, 0.01):
        for pitch in np.arange(0.80, 1.60, 0.05):
            urdf = f"""<robot name="r">
<link name="base_link"/><link name="base_rot"/>
<joint name="base_joint" type="revolute">
  <parent link="base_link"/><child link="base_rot"/>
  <origin xyz="0 0 0.14" rpy="0 0 0"/><axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14"/>
</joint>
<link name="arm1"/>
<joint name="shoulder_joint" type="revolute">
  <parent link="base_rot"/><child link="arm1"/>
  <origin xyz="{shoulder_offset:.3f} 0 0" rpy="0 0 -1.5708"/><axis xyz="0 1 0"/>
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
  <origin xyz="0.05 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
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
  <origin xyz="0.109 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
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
                    best = (shoulder_offset, elbow_x, pitch, x, T[1,3], z)
            except:
                pass

print(f"Best: shoulder={best[0]:.3f} elbow_x={best[1]:.3f} pitch={best[2]:.3f}")
print(f"  FK: x={best[3]:.4f} y={best[4]:.4f} z={best[5]:.4f}")
print(f"  Error: {best_err:.4f}")
