import numpy as np
from interbotix_xs_modules.arm import InterbotixManipulatorXS
import rospy

# Define the transformation from camera to end-effector frame (fixed)
# Example: camera is above the end-effector by 0.1m along the z-axis
def get_camera_to_ee_transform():

    Tc_e = np.eye(4)  
    Tc_e[2, 3] = 0.1  
    return Tc_e

def transform_object_to_ee_frame(object_pos_camera, bot):
    # Step 1: Get the end-effector pose in the base frame
    ee_pose = bot.arm.get_ee_pose()
    
    Tc_e = get_camera_to_ee_transform()

  
    object_pos_camera = np.array([object_pos_camera[0], object_pos_camera[1], object_pos_camera[2], 1.0])
    
    T_base_to_ee = np.linalg.inv(ee_pose)  # Inverse of end-effector to base transformation to get base to end-effector
    
    object_pos_base = np.dot(ee_pose, np.dot(Tc_e, object_pos_camera))

    object_pos_ee = object_pos_base[:3]  
    return object_pos_ee

# Example usage
if __name__ == "__main__":
    bot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name='puppet_left', init_node=True)

    object_pos_camera = [6.705854574911913, 71.29857383884278, 28.8]  # Example object position in camera frame (x, y, z)

    object_pos_ee = transform_object_to_ee_frame(object_pos_camera, bot)

    print(f"Object Position in End-Effector Frame: x = {object_pos_ee[0]}, y = {object_pos_ee[1]}, z = {object_pos_ee[2]}")
