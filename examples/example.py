# from interbotix_xs_modules.arm import InterbotixManipulatorXS
# import numpy as np

# class CucumberHarvesting:
#     def __init__(self, robot_name='puppet_left'):
        
#         self.bot = InterbotixManipulatorXS("vx300s", "arm", "gripper", robot_name=robot_name)

#     def perform_harvest(self):
       
#         self.bot.arm.go_to_home_pose()
#         self.bot.arm.set_ee_pose_components(x=0.4, z=0.3)
#         self.bot.arm.set_single_joint_position("waist", np.pi / 2.0)
#         self.bot.gripper.open()

#         # Move to harvesting position and close gripper
#         self.bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
#         self.bot.gripper.close()

#         # Retract and reset waist
#         self.bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
#         self.bot.arm.set_single_joint_position("waist", -np.pi / 2.0)

#         # Adjust pitch and reset waist again
#         self.bot.arm.set_ee_cartesian_trajectory(pitch=1.5)
#         self.bot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
#         self.bot.arm.set_single_joint_position("waist", np.pi / 2.0)

#         # Open gripper and return to home pose
#         self.bot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.16)
#         self.bot.gripper.open()
#         self.bot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.16)
#         self.bot.arm.go_to_home_pose()
#         self.bot.arm.go_to_sleep_pose() 

# if __name__ == '__main__':
#     cucumber_harvest = CucumberHarvesting()
#     cucumber_harvest.perform_harvest()
