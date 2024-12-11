from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
from robot_utils import move_arms, torque_on, torque_off 
from time import sleep
import rospy

# rospy.init_node('perform_task_harvest_puppet', anonymous=True)

class Test:
    def __init__(self, robot_name='puppet_left'):
        self.bot = InterbotixManipulatorXS("vx300s", "arm", "gripper", robot_name=robot_name, moving_time=5)
        self.pick_cucu_2 = (-0.1089126393198967, 0.6688156127929688, 0.8160778284072876, -0.16413594782352448, -1.5171070098876953, 0.015339808538556099)
        self.place_trajectory = [
            {0.0: [0.010737866163253784, -1.3867186307907104, 0.7853981852531433, 0.03681553900241852, 0.6412039995193481, -0.02761165425181389]},
            {2.0: [0.010737866163253784, -1.3867186307907104, 0.7853981852531433, 0.03681553900241852, 0.6412039995193481, -0.02761165425181389]},
            {4.0: [-0.45712628960609436, -1.1090681552886963, 0.08897088468074799, 0.5537670850753784, 0.9464661478996277, -0.1426602154970169]},
            {6.0: [-0.9480001330375671, -1.026233196258545, -0.5322913527488708, 1.8008935451507568, 1.3253594636917114, 0.06902913749217987]}
        ]


    def execute_trajectory(self):
        self.bot.dxl.robot_write_trajectory('group', 'arm', 'position', self.place_trajectory)

    def place_in_basket(self):
        self.execute_trajectory()
        sleep(10)
        self.bot.gripper.open()

    def go_to_sleep_pose(self):
        self.bot.arm.go_to_sleep_pose()

    def opening_cermony(self):
        torque_on(self.bot)
        self.bot.dxl.robot_reboot_motors("single", "gripper", True)

    def cucu_1(self):

        self.set_end_effector_pose(x=0.37,y=0, z=0.7, moving_time=4)

        self.bot.gripper.close()

        rospy.sleep(1)

    def cucu_2(self):

        self.set_end_effector_pose(x=0.39,y =-0.03,z=0.35, moving_time=4)

        self.bot.gripper.close()

        rospy.sleep(1)



    def set_end_effector_pose(self,x,y,z, moving_time):
        self.bot.arm.set_ee_pose_components(x=x, y=y ,z=z, moving_time=moving_time)


    def standard_pose(self):
        self.set_end_effector_pose(x=0.2,y=0, z=0.3, moving_time=4)



    def main(self):

        print("---------->Task Started<-------------------------")

        self.opening_cermony()

        self.cucu_1()

        rospy.sleep(1)

        self.standard_pose()

        self.place_in_basket()

        self.standard_pose()

        rospy.sleep(1)

        print("---------->Moving to next Cucu<-------------------------")

        self.cucu_2()

        self.standard_pose()

        self.place_in_basket()

        self.standard_pose()

        self.go_to_sleep_pose()







if __name__ == '__main__':
    cucumber_harvest = Test()
    cucumber_harvest.main()
