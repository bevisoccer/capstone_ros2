import numpy as np

# At origin pose with shoulder at -53deg:
# x = 0.05 (base offset) + upper_arm * cos(53) + forearm_x_contribution
# z = 0.14 + upper_arm * sin(53) + forearm_z_contribution

# From best elbow params: elbow_x=0.05, pitch=1.0
# forearm contribution at motor angles 5deg roll, -17deg wrist:
# approximately x_contrib = 0.30 * cos(1.0) = 0.162, z_contrib = -0.30 * sin(1.0) = -0.252... 
# but those aren't right either

# Let's just solve for upper arm length directly
# Real elbow position: x_elbow = 0.05 + L * cos(53), z_elbow = 0.14 + L * sin(53)
# Real tip: x=0.26, z=0.16
# Measured elbow height: z=0.28

shoulder_angle = np.deg2rad(52.99)
z_elbow_measured = 0.28
z_shoulder = 0.14

# From z: L * sin(53) = 0.28 - 0.14 = 0.14
L = 0.14 / np.sin(shoulder_angle)
print(f"Upper arm length from z measurement: {L:.4f}m")

x_elbow = 0.05 + L * np.cos(shoulder_angle)
print(f"Elbow x position: {x_elbow:.4f}m")
print(f"Remaining x from elbow to tip: {0.26 - x_elbow:.4f}m")
print(f"Remaining z from elbow to tip: {0.16 - z_elbow_measured:.4f}m")

reach = np.sqrt((0.26 - x_elbow)**2 + (0.16 - z_elbow_measured)**2)
print(f"Forearm+wrist reach needed: {reach:.4f}m")
print(f"Current forearm+wrist in URDF: 0.09 + 0.21 = 0.30m")
