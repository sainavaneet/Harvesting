from interbotix_xs_modules.arm import InterbotixManipulatorXS
import rospy
from time import sleep
from config import GRIPPER_JOINT_OPEN, GRIPPER_JOINT_CLOSE, TARGETS
from utilities import Utils
from base_control.agv_control import BaseMove
from transform_co import Detection

class Harvest:
    def __init__(self, robot_name='puppet_left'):
        self.bot = InterbotixManipulatorXS("vx300s", "arm", "gripper", robot_name=robot_name, moving_time=4)
        self.base_robot = BaseMove()
        self.detect_obj_pose = Detection(self.bot)
        self.moving_time = 3

        self.place_trajectory = [
            {0.0: [0.010737866163253784, -1.3867186307907104, 0.7853981852531433, 0.03681553900241852, 0.6412039995193481, -0.02761165425181389]},
            {2.0: [0.010737866163253784, -1.3867186307907104, 0.7853981852531433, 0.03681553900241852, 0.6412039995193481, -0.02761165425181389]},
            {4.0: [-0.45712628960609436, -1.1090681552886963, 0.08897088468074799, 0.5537670850753784, 0.9464661478996277, -0.1426602154970169]},
            {6.0: [-0.9480001330375671, -1.026233196258545, -0.5322913527488708, 1.8008935451507568, 1.3253594636917114, 0.06902913749217987]}
        ]



    def opening_ceremony(self , bot):
        bot.dxl.robot_reboot_motors("single", "gripper", True)
        bot.dxl.robot_set_operating_modes("group", "arm", "position")
        bot.dxl.robot_set_operating_modes("single", "gripper", "current_based_position")
        Utils.torque_on(bot)


    def execute_trajectory(self, trajectory):
        self.bot.dxl.robot_write_trajectory('group', 'arm', 'position', trajectory)

    def place_in_basket(self):
        rospy.sleep(1)
        self.execute_trajectory(self.place_trajectory)
        sleep(9)
        self.bot.gripper.open()

    def move_backward(self):
        self.base_robot.backward()

    def move_forward(self):
        self.base_robot.forward()


    def set_end_effector_pose(self, x, y, z, moving_time):
        self.bot.arm.set_ee_pose_components(x=x, y=y, z=z, moving_time=moving_time)



    def operation(self, cucu_id):

        target = TARGETS.get(cucu_id, None)
        
        if target is None:
            rospy.logerr(f"Target {cucu_id} not found!")
            return

        initial_pose = target['initial_pose']
        

        self.set_end_effector_pose(initial_pose['x'], initial_pose['y'], initial_pose['z'], self.moving_time)
        
     
        Utils.gripper_posiition("open")

  
        object_pose = self.detect_obj_pose.object_pose()
        
 
        offset = [0.1, 0, 0.1]
        new_pose = Utils.cal_offset(object_pose, offset)
        

        if not Utils.check_tolerance(new_pose, target['position'].values()):
            new_pose = target['position']
        

        self.set_end_effector_pose(*new_pose, self.moving_time)
        rospy.sleep(1)
        

        Utils.gripper_position('close')
        
 
        Utils.standard_pose()
        

        self.place_in_basket()



if __name__ == "__main__":
    task = Harvest()

    task.operation('cucu1')


