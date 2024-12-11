from interbotix_xs_modules.arm import InterbotixManipulatorXS
import time
import numpy as np
from robot_utils import move_arms, torque_on, torque_off , move_grippers
import rospy

from constants import PUPPET_GRIPPER_JOINT_CLOSE , PUPPET_GRIPPER_JOINT_OPEN

class CucumberHarvesting:
    def __init__(self, robot_name='puppet_left'):
        rospy.init_node('harvest', anonymous=True)
        self.bot = InterbotixManipulatorXS(robot_model="vx300s", group_name="arm", gripper_name="gripper", robot_name=robot_name, init_node=False)
        self.GRIPPER_JOINT_OPEN = 1.4910
        self.GRIPPER_JOINT_CLOSE = 0.4
        self.move_time = 2
        self.home_pose = (0.02761165425181389, -1.0921943187713623, 1.0308351516723633, -0.03988350182771683, 0.03221359848976135, -0.015339808538556099)


    def home_cucu_pose(self , move_time):
        rospy.sleep(1)
        move_arms([self.bot], [self.home_pose], move_time)

    def opening_cermony(self):
        torque_on(self.bot)

        time.sleep(1)

        self.bot.dxl.robot_reboot_motors("single", "gripper", True)

        self.home_cucu_pose(self.move_time)

        move_grippers([self.bot], [self.GRIPPER_JOINT_OPEN], move_time=1)

    def pick_cucu_1(self):
        
        cucu_pick_pose = (0.004601942375302315, -0.48166999220848083, 0.4832039475440979, -0.004601942375302315, 0.05062136799097061, -0.07209710031747818)
        pre_place_pose = (-0.01840776950120926, -0.7869321703910828, 0.7577865123748779, 0.026077674701809883, 1.118272066116333, -0.03834952041506767)
        place_pose = (-0.08436894416809082, 0.4693981409072876, 0.7823302149772644, -0.07516506314277649, 0.16873788833618164, 0.10124273598194122)


        #task-1
    
        time.sleep(1)
        move_arms([self.bot], [cucu_pick_pose], move_time=self.move_time) 
        
        time.sleep(1)  
        move_grippers([self.bot], [self.GRIPPER_JOINT_CLOSE], move_time=1)

        self.home_cucu_pose(self.move_time) 

        time.sleep(1)  

        move_arms([self.bot], [pre_place_pose], move_time=self.move_time)  

        time.sleep(1)  
        move_arms([self.bot], [place_pose], move_time=self.move_time)  

        time.sleep(1)  
        move_grippers([self.bot], [self.GRIPPER_JOINT_OPEN], move_time=1)

        self.home_cucu_pose(5)  


    def pick_cucu_2(self):

        pick_cucu_2 = (-0.1089126393198967, 0.6688156127929688, 0.8160778284072876, -0.16413594782352448, -1.5171070098876953, 0.015339808538556099)
        next_cucu_2_pose = (-0.1089126393198967, 0.6688156127929688, 0.8160778284072876, -0.16413594782352448, -1.5171070098876953, 0.015339808538556099)
        place_cucu_2_pose = (-0.08436894416809082, 0.8897088766098022, 0.40036898851394653, -0.06902913749217987, 0.40036898851394653, 1.4327380657196045)

        rospy.sleep(2)

        move_arms([self.bot], [pick_cucu_2], move_time=self.move_time) 
        

        time.sleep(1)  
        move_grippers([self.bot], [self.GRIPPER_JOINT_CLOSE], move_time=1)

        move_arms([self.bot], [next_cucu_2_pose], move_time=self.move_time) 

        move_arms([self.bot], [place_cucu_2_pose], move_time= 4) 
        rospy.sleep(1)
        move_grippers([self.bot], [self.GRIPPER_JOINT_OPEN], move_time=1)

        rospy.sleep(1)

        self.home_cucu_pose(6)





if __name__ == '__main__':
    cucumber_harvest = CucumberHarvesting()

    cucumber_harvest.opening_cermony()

    cucumber_harvest.pick_cucu_1()
    # cucumber_harvest.pick_cucu_2()

