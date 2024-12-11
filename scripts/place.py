from interbotix_xs_modules.arm import InterbotixManipulatorXS
from time import sleep
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robot_utils import move_arms, torque_on, torque_off , move_grippers

class PuppetPlacer:
    def __init__(self, robot_model="vx300s", robot_name="puppet_left", moving_time=6):
        rospy.init_node('place_puppet', anonymous=True)
        self.trajectory = [
            {0.0: [0.010737866163253784, -1.3867186307907104, 0.7853981852531433, 0.03681553900241852, 0.6412039995193481, -0.02761165425181389]},
            {2.0: [0.010737866163253784, -1.3867186307907104, 0.7853981852531433, 0.03681553900241852, 0.6412039995193481, -0.02761165425181389]},
            {4.0: [-0.45712628960609436, -1.1090681552886963, 0.08897088468074799, 0.5537670850753784, 0.9464661478996277, -0.1426602154970169]},
            {6.0: [-0.9480001330375671, -1.026233196258545, -0.5322913527488708, 1.8008935451507568, 1.3253594636917114, 0.06902913749217987]}
        ]
        self.bot = InterbotixManipulatorXS(
            robot_model=robot_model,
            group_name="arm",
            gripper_name="gripper",
            robot_name=robot_name,
            init_node=False,
            moving_time=moving_time
        )

    def go_to_home_pose(self):
        self.bot.arm.go_to_home_pose()

    def go_to_sleep_pose(self):
        self.bot.arm.go_to_sleep_pose()

    def close_gripper(self):
        self.bot.gripper.close()

    def open_gripper(self):
        self.bot.gripper.open()

    def set_end_effector_pose(self, x, z, moving_time):
        self.bot.arm.set_ee_pose_components(x=x, z=z, moving_time=moving_time)

    def execute_trajectory(self):
        self.bot.dxl.robot_write_trajectory('group', 'arm', 'position', self.trajectory)

    def standard_pose(self):
        torque_on(self.bot)
        self.close_gripper()
        self.set_end_effector_pose(x=0.2, z=0.3, moving_time=4)
  


    def perform_task(self):
        self.execute_trajectory()
        sleep(10)
        self.open_gripper()


if __name__ == '__main__':
    placer = PuppetPlacer()
    placer.home_pose()
    placer.perform_task()
    placer.go_to_sleep_pose()

