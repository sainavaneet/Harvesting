from object_detection.detection_realsenes import ObjectPoseEstimator
import rospy
import tf
from geometry_msgs.msg import PoseStamped
import numpy as np
from interbotix_xs_modules.arm import InterbotixManipulatorXS


class Detection():

    def __init__(self , bot):
        self.object_detector = ObjectPoseEstimator()
        self.bot = bot
    def get_pose(self):
        self.object_detector.run() 

        if self.object_detector: 
            pose = self.object_detector.use_stable_pose()
            if pose:
                rospy.loginfo(f"Detected and stable cucumber pose: {pose}")
        if pose is not None:
        
            return pose
        else:
            rospy.WARN("Pose not recived")

    def change_wrt_robot(self , ee_pose , obj_pose):
        
        obj_pose_array = np.array([*obj_pose, 1])
        ee_pose_inverse = np.linalg.inv(ee_pose)
        object_pos_ee = ee_pose_inverse @ obj_pose_array
        object_position_robot = object_pos_ee[:3] / 100
        rospy.loginfo(f"Detected and stable cucumber pose wrt robot: {object_position_robot}")
        return object_position_robot

    def robot_ee_pose(self , bot):
        ee_pose = bot.arm.get_ee_pose()
        return ee_pose
    
    def object_pose(self):

        obj_pose = self.get_pose()

        ee_pose = self.robot_ee_pose(self.bot)
    
        pose_wrt_robot = self.change_wrt_robot(ee_pose , obj_pose)

        return pose_wrt_robot



if __name__ == "__main__":

    transform = Detection()

    



