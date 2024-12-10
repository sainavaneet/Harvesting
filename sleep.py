import numpy as np
from interbotix_xs_modules.arm import InterbotixManipulatorXS
import rospy


from scripts.robot_utils import *

GRIPPER_JOINT_CLOSE = 0.4




def torque_on(bot):
    bot.dxl.robot_torque_enable("group", "arm", True)
    bot.dxl.robot_torque_enable("single", "gripper", True)





bot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name='puppet_left', init_node=True)
torque_on(bot)




bot.arm.go_to_sleep_pose()
