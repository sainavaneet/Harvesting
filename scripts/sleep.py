
from interbotix_xs_modules.arm import InterbotixManipulatorXS
import time
import numpy as np
from robot_utils import move_arms, torque_on, torque_off , move_grippers
import rospy


rospy.init_node('sleep_robot_puppet_1', anonymous=True)

puppet_sleep_position = (0, -1.7, 1.55, 0.12, 0.65, 0)

bot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name='puppet_left', init_node=False)

move_arms([bot], [puppet_sleep_position], move_time=4)
rospy.sleep(2)