import numpy as np
import time
from interbotix_xs_msgs.msg import JointSingleCommand
from var import *
import IPython
e = IPython.embed


DT = 0.002

class Utils:
    @staticmethod
    def get_arm_gripper_positions(bot):
        """
        Get the current gripper position from the bot.

        Parameters:
            bot (object): The robot instance.
        
        Returns:
            float: The current gripper position.
        """
        joint_position = bot.gripper.core.joint_states.position[6]
        return joint_position

    @staticmethod
    def move_grippers(bot_list, target_pose_list, move_time):
        """
        Move the grippers of the bots to the target positions.

        Parameters:
            bot_list (list): A list of robot instances.
            target_pose_list (list): A list of target gripper positions for each bot.
            move_time (float): The time to move the grippers.
        """
        gripper_command = JointSingleCommand(name="gripper")
        num_steps = int(move_time / DT)
        curr_pose_list = [Utils.get_arm_gripper_positions(bot) for bot in bot_list]
        traj_list = [np.linspace(curr_pose, target_pose, num_steps) 
                     for curr_pose, target_pose in zip(curr_pose_list, target_pose_list)]
        
        for t in range(num_steps):
            for bot_id, bot in enumerate(bot_list):
                gripper_command.cmd = traj_list[bot_id][t]
                bot.gripper.core.pub_single.publish(gripper_command)
            time.sleep(DT)

    @staticmethod
    def gripper_position(bot, position):
        """
        Set the gripper position of a bot to 'open' or 'close'.

        Parameters:
            bot (object): The robot instance.
            position (str): The target position of the gripper ('open' or 'close').
            move_time (float): The time to move the gripper.
        """
        if position == "open":
            Utils.move_grippers([bot], [GRIPPER_JOINT_OPEN], move_time=0.5)
        elif position == "close":
            Utils.move_grippers([bot], [GRIPPER_JOINT_CLOSE], move_time=0.5)
        else:
            print("WRONG GRIPPER POSITION")

    @staticmethod
    def cal_offset(current_pose, offset):
        """
        Calculate the new position by adding an offset to the current position.

        Parameters:
            current_pose (numpy.ndarray): The current position.
            offset (list or numpy.ndarray): The offset to add to the current position.
        
        Returns:
            numpy.ndarray: The new position.
        """
        return current_pose + np.array(offset)

    @staticmethod
    def check_tolerance(current_pose, target):
        """
        Check if the current position is within a specified tolerance of the target position.

        Parameters:
            current_pose (numpy.ndarray): The current position.
            target (list or numpy.ndarray): The target position.
        
        Returns:
            bool: True if the position is within tolerance, False otherwise.
        """
        tolerance = 0.01
        return np.all(np.abs(current_pose - np.array(target)) <= tolerance)

    @staticmethod
    def standard_pose(self):
        """
        Set the end-effector to a standard pose.
        """
        self.set_end_effector_pose(x=0.2, y=0, z=0.3, moving_time=2)

    @staticmethod
    def torque_off(bot):
        """
        STOP torque for the both joints and Gripper
        """
        bot.dxl.robot_torque_enable("group", "arm", False)
        bot.dxl.robot_torque_enable("single", "gripper", False)

    @staticmethod
    def torque_on(bot):
        """
        ENABLE torque for the both joints and Gripper
        """
        bot.dxl.robot_torque_enable("group", "arm", True)
        bot.dxl.robot_torque_enable("single", "gripper", True)

    @staticmethod
    def get_arm_joint_positions(bot):
        return bot.arm.core.joint_states.position[:6]
    
    @staticmethod
    def move_arms(bot_list, target_pose_list, move_time=1):
        num_steps = int(move_time / DT)
        curr_pose_list = [Utils.get_arm_joint_positions(bot) for bot in bot_list]
        traj_list = [np.linspace(curr_pose, target_pose, num_steps) for curr_pose, target_pose in zip(curr_pose_list, target_pose_list)]
        for t in range(num_steps):
            for bot_id, bot in enumerate(bot_list):
                bot.arm.set_joint_positions(traj_list[bot_id][t], blocking=False)
            time.sleep(DT)
