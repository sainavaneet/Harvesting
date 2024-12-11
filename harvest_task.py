from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
from robot_utils import move_arms, torque_on, torque_off , move_grippers
from time import sleep
import rospy
from base_control.agv_control import BaseMove
from transform_co import Detection

class Harvest:
    def __init__(self, robot_name='puppet_left'):
        self.bot = InterbotixManipulatorXS("vx300s", "arm", "gripper", robot_name=robot_name, moving_time=4)

        self.base_robot = BaseMove()


        self.GRIPPER_JOINT_OPEN = 1.4910
        self.GRIPPER_JOINT_CLOSE = 0.4


        self.place_trajectory = [
            {0.0: [0.010737866163253784, -1.3867186307907104, 0.7853981852531433, 0.03681553900241852, 0.6412039995193481, -0.02761165425181389]},
            {2.0: [0.010737866163253784, -1.3867186307907104, 0.7853981852531433, 0.03681553900241852, 0.6412039995193481, -0.02761165425181389]},
            {4.0: [-0.45712628960609436, -1.1090681552886963, 0.08897088468074799, 0.5537670850753784, 0.9464661478996277, -0.1426602154970169]},
            {6.0: [-0.9480001330375671, -1.026233196258545, -0.5322913527488708, 1.8008935451507568, 1.3253594636917114, 0.06902913749217987]}
        ]

        self.detect_obj_pose = Detection(self.bot)

        self.moving_time = 3

    def execute_trajectory(self):
        self.bot.dxl.robot_write_trajectory('group', 'arm', 'position', self.place_trajectory)

    def place_in_basket(self):
        
        self.execute_trajectory()
        sleep(10)
        self.gripper_open()
        rospy.sleep(1)

        self.standard_pose()

    

    def go_to_sleep_pose(self):
        self.bot.arm.go_to_sleep_pose()


    def standard_pose(self):
        self.set_end_effector_pose(x=0.2,y=0, z=0.3, moving_time=2)



    def opening_cermony(self):
        torque_on(self.bot)

        self.bot.dxl.robot_reboot_motors("single", "gripper", True)

        self.bot.dxl.robot_set_operating_modes("single", "gripper", "current_based_position")
        
        # self.go_to_sleep_pose()


    def place_in_basket(self):
        self.execute_trajectory()
        sleep(10)
        self.bot.gripper.open()


    def set_end_effector_pose(self,x,y,z, moving_time):
        self.bot.arm.set_ee_pose_components(x=x, y=y ,z=z, moving_time=moving_time)


    def cal_offset(self , cucu_pose , off_x , off_y, off_z):

        x = cucu_pose[0] + off_x

        # y = cucu_pose[1] + off_y

        y = 0

        z = cucu_pose[2] + off_z

        return np.array([ x , y , z])

    def move_backward(self):
        self.base_robot.backward()

    def move_forward(self):
        self.base_robot.forward()

    def move_to_start(self):
        self.base_robot.move_to_start()


    def gripper_close(self):

        move_grippers([self.bot], [self.GRIPPER_JOINT_CLOSE], move_time=0.5)

    def gripper_open(self):
        move_grippers([self.bot], [self.GRIPPER_JOINT_OPEN], move_time=0.5)


    def check_tolarence(self ,cucu_pose , target_x , target_y , target_z):
            tolerance = 0.01
            
            x = cucu_pose[0]
            y = cucu_pose[1]
            z = cucu_pose[2]

            if not (abs(x - target_x) <= tolerance and abs(y - target_y) <= tolerance and abs(z - target_z) <= tolerance):
                x, y, z = target_x, target_y, target_z

                return np.array([x , y , z])

    def cucu1(self):
        self.set_end_effector_pose(0.2, 0, 0.6, self.moving_time)
        self.gripper_open()
        cucu1_pose = self.detect_obj_pose.object_pose()
        move_cucu1 = self.cal_offset(cucu1_pose, 0.1, 0.1, 0.1)
        target_x = 0.35
        target_y = 0.055
        target_z = 0.54
        result = self.check_tolarence(move_cucu1, target_x, target_y, target_z)
        x, y, z = result[0], result[1], result[2]
        print(f"cucu1pose--------------------------------------------- {result}")
        

        rospy.sleep(1)

        self.set_end_effector_pose(x, y, z, self.moving_time)

        rospy.sleep(1)
        self.gripper_close()
        rospy.sleep(1)
        self.place_in_basket()



    def cucu2(self):
        self.set_end_effector_pose(0.2, 0, 0.67, self.moving_time)
        self.gripper_open()
        rospy.sleep(2)
        cucu2_pose = self.detect_obj_pose.object_pose()

        move_cucu2 = self.cal_offset(cucu2_pose, 0.1, 0.1, 0.1)
        target_x = 0.33
        target_y = -0.01
        target_z = 0.7

        result = self.check_tolarence(move_cucu2, target_x, target_y, target_z)

        x, y, z = result[0], result[1], result[2]
        print(f"cucu2pose--------------------------------------------- {result}")
        self.set_end_effector_pose(x, y, z, self.moving_time)
        rospy.sleep(1)
        self.gripper_close()
        rospy.sleep(1)
        self.place_in_basket()


    def cucu3(self):
        self.set_end_effector_pose(0.15, 0, 0.35, self.moving_time)
        self.gripper_open()

        rospy.sleep(2)

        cucu3_pose = self.detect_obj_pose.object_pose()
        move_cucu3 = self.cal_offset(cucu3_pose, 0.1, 0.1, 0.1)
        target_x = 0.34
        target_y = 0.0
        target_z = 0.37
        result = self.check_tolarence(move_cucu3, target_x, target_y, target_z)
        x, y, z = result[0], result[1], result[2]
        print(f"cucu3pose--------------------------------------------- {result}")
        self.set_end_effector_pose(x, y, z, self.moving_time)
        rospy.sleep(1)
        self.gripper_close()
        rospy.sleep(1)

        self.standard_pose()
        rospy.sleep(1)

        self.place_in_basket()


    def cucu4(self):
        self.set_end_effector_pose(0.15, 0, 0.6, self.moving_time)
        self.gripper_open()

        rospy.sleep(2)

        cucu4_pose = self.detect_obj_pose.object_pose()
        move_cucu4 = self.cal_offset(cucu4_pose, 0.1, 0.1, 0.1)
        target_x = 0.32
        target_y = 0
        target_z = 0.7


        result = self.check_tolarence(move_cucu4, target_x, target_y, target_z)
        x, y, z = result[0], result[1], result[2]
        print(f"cucu4pose--------------------------------------------- {result}")
        self.set_end_effector_pose(x, y, z, self.moving_time)
        rospy.sleep(1)
        self.gripper_close()
        rospy.sleep(1)


        self.place_in_basket()

    def cucu5(self):
        self.set_end_effector_pose(0.15, 0, 0.35, self.moving_time)
        self.gripper_open()

        rospy.sleep(2)

        cucu5_pose = self.detect_obj_pose.object_pose()
        move_cucu5 = self.cal_offset(cucu5_pose, 0.1, 0.1, 0.1)
        target_x = 0.37
        target_y = -0.03
        target_z = 0.37

        result = self.check_tolarence(move_cucu5, target_x, target_y, target_z)
        x, y, z = result[0], result[1], result[2]
        print(f"cucu5pose--------------------------------------------- {result}")
        self.set_end_effector_pose(x, y, z, self.moving_time)
        rospy.sleep(1)
        self.gripper_close()
        rospy.sleep(1)

        self.standard_pose()

        self.place_in_basket()
        rospy.sleep(1)

    def cucu6(self):
        self.set_end_effector_pose(0.2, 0, 0.6, self.moving_time)
        self.gripper_open()
        cucu6_pose = self.detect_obj_pose.object_pose()
        move_cucu6 = self.cal_offset(cucu6_pose, 0.1, 0.1, 0.1)
        target_x = 0.32
        target_y = -0.02
        target_z = 0.7
        result = self.check_tolarence(move_cucu6, target_x, target_y, target_z)
        x, y, z = result[0], result[1], result[2]
        print(f"cucu6pose--------------------------------------------- {result}")
        self.set_end_effector_pose(x, y, z, self.moving_time)
        rospy.sleep(1)
        self.gripper_close()

        rospy.sleep(0.1)

        self.standard_pose()

        self.place_in_basket()

    def cucu7(self):
        self.set_end_effector_pose(0.15, 0, 0.35, self.moving_time)
        self.gripper_open()
        cucu7_pose = self.detect_obj_pose.object_pose()
        move_cucu7 = self.cal_offset(cucu7_pose, 0.1, 0.1, 0.1)
        target_x = 0.37
        target_y = -0.03
        target_z = 0.37
        result = self.check_tolarence(move_cucu7, target_x, target_y, target_z)
        x, y, z = result[0], result[1], result[2]
        print(f"cucu7pose--------------------------------------------- {result}")
        self.set_end_effector_pose(x, y, z, self.moving_time)
        rospy.sleep(1)
        self.gripper_close()
        rospy.sleep(0.1)
        self.standard_pose()

        self.place_in_basket()
        

    def cucu8(self):
        self.set_end_effector_pose(0.2, 0, 0.5, self.moving_time)
        self.gripper_open()
        cucu8_pose = self.detect_obj_pose.object_pose()
        move_cucu8 = self.cal_offset(cucu8_pose, 0.1, 0.1, 0.1)
        target_x = 0.34
        target_y = -0.03
        target_z = 0.55
        result = self.check_tolarence(move_cucu8, target_x, target_y, target_z)
        x, y, z = result[0], result[1], result[2]
        print(f"cucu7pose--------------------------------------------- {result}")
        self.set_end_effector_pose(x, y, z, self.moving_time)
        rospy.sleep(1)
        self.gripper_close()
        rospy.sleep(0.1)
        self.standard_pose()
        self.place_in_basket()


    def cucu9(self):
        self.set_end_effector_pose(0.2, 0, 0.3, self.moving_time)
        self.gripper_open()
        cucu9_pose = self.detect_obj_pose.object_pose()
        move_cucu9 = self.cal_offset(cucu9_pose, 0.1, 0.1, 0.1)
        target_x = 0.37
        target_y = -0.04
        target_z = 0.28
        result = self.check_tolarence(move_cucu9, target_x, target_y, target_z)
        x, y, z = result[0], result[1], result[2]
        print(f"cucu7pose--------------------------------------------- {result}")
        self.set_end_effector_pose(x, y, z, self.moving_time)
        rospy.sleep(1)
        self.gripper_close()
        rospy.sleep(0.1)
        self.standard_pose()
        self.place_in_basket()



if __name__ == "__main__":

    task = Harvest()

    task.opening_cermony()

    rospy.sleep(1)

    task.cucu1()

    task.move_forward()
    
    task.cucu2()

    rospy.sleep(1)

    task.cucu3()

    task.move_forward()

    task.cucu4()

    rospy.sleep(1)

    task.cucu5()

    rospy.sleep(1)

    task.move_forward()

    task.cucu6()

    rospy.sleep(1)
    
    task.cucu7()

    rospy.sleep(1)

    task.move_forward()

    task.cucu8()

    rospy.sleep(1)

    task.move_forward()
    
    task.cucu9()
    
    rospy.sleep(1)

    task.go_to_sleep_pose()

    rospy.sleep(1)

    task.move_to_start()















