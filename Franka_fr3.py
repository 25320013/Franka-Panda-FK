import mujoco
import numpy as np

# 1. Load the Franka FR3 model and data
model = mujoco.MjModel.from_xml_path("/home/iitgn-robotics/Pant/mujoco_menagerie/franka_fr3/scene.xml")
data = mujoco.MjData(model)

# 2. Set custom joint angles (in radians) for the 7 joints of the FR3 arm
# You can change these values to test different configurations
joint_angles = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

# Assign the joint positions to the model's generalized coordinates (qpos)
data.qpos[:7] = joint_angles

# 3. Compute forward kinematics
# This updates all body positions, sites, and matrices based on the current qpos
mujoco.mj_forward(model, data)

# 4. Extract end-effector pose
site_name = "attachment_site"  # Standard end-effector site in Menagerie models

try:
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
    pos = data.site_xpos[site_id]
    mat = data.site_xmat[site_id].reshape(3, 3) # 3x3 Rotation matrix
    
    print(f"=== Franka FR3 Forward Kinematics ===")
    print(f"Joint Angles (rad): {joint_angles}")
    print(f"End-Effector Site: {site_name}")
    print(f"Position (x, y, z): [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] meters")
    print(f"Rotation Matrix:\n{mat}")

except Exception as e:
    print(f"Could not find site '{site_name}': {e}")
    # Fallback to the 'hand' body if site lookup fails
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")
    pos = data.xpos[body_id]
    print(f"=== Franka FR3 FK (Body Fallback) ===")
    print(f"Position (x, y, z): [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] meters")
