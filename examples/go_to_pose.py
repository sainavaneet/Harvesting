import numpy as np
from interbotix_xs_modules.arm import InterbotixManipulatorXS
import rospy




def torque_on(bot):
    bot.dxl.robot_torque_enable("group", "arm", True)
    bot.dxl.robot_torque_enable("single", "gripper", True)



bot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name='puppet_left', init_node=True)
torque_on(bot)



bot.arm.set_ee_pose_components(x=0.35, y=0, z=0, moving_time=5)


