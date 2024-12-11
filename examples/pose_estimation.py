import numpy as np
from interbotix_xs_modules.arm import InterbotixManipulatorXS
import rospy



def torque_on(bot):
    bot.dxl.robot_torque_enable("group", "arm", True)
    bot.dxl.robot_torque_enable("single", "gripper", True)



bot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name='puppet_left', init_node=True)
torque_on(bot)

bot.arm.set_ee_pose_components(x=0.2, y=0.0, z=0.6, moving_time=4) #------------------ 1st cucu 

# bot.arm.set_ee_pose_components(x=0.2, y=0.0, z=0.35, moving_time=4) #------------------ 1st cucu 




ee_pose = bot.arm.get_ee_pose()


P_camera = np.array([ 38.7 , -17.8822778447963 , 17.281989530103925, 1])

ee_pose_inverse = np.linalg.inv(ee_pose)


object_pos_ee = ee_pose_inverse @ P_camera



object_position_robot = object_pos_ee[:3] / 100

x, y, z = object_position_robot

print(f"Object position in robot frame: X, Y, Z: {x:.2f}, {y:.2f}, {z:.2f}")



