from object_detection.detection_realsenes import ObjectPoseEstimator
import rospy
import tf
from geometry_msgs.msg import PoseStamped

class Detection():

    def __init__(self):
        self.object_detector = ObjectPoseEstimator()

    def pose(self):
        self.object_detector.main()

        if self.object_detector.stable: 
            pose = self.object_detector.use_stable_pose()
            if pose:
                rospy.loginfo(f"Detected and stable cucumber pose: {pose}")
        if pose is not None:
        
            return pose
        else:
            rospy.WARN("Pose not recived")

if __name__ == "__main__":

    transform = Detection()
    transform.pose()

