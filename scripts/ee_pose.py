import rospy
import tf
from geometry_msgs.msg import PoseStamped

class ee_pose:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('tf_listener', anonymous=True)
        
        # Create a TransformListener instance
        self.listener = tf.TransformListener()

    def get_position_orientation(self, target_frame):
            self.listener.waitForTransform('/world', target_frame, rospy.Time(0), rospy.Duration(3.0))
       
            (trans, rot) = self.listener.lookupTransform('/world', target_frame, rospy.Time(0))
            
            position = {
                'x': trans[0],
                'y': trans[1],
                'z': trans[2]
            }

            orientation = {
                'x': rot[0],
                'y': rot[1],
                'z': rot[2],
                'w': rot[3]
            }

            return position, orientation

if __name__ == '__main__':
    # Create a TFListener object
    tf_listener = ee_pose()

    # Specify the target frame (in this case puppet_left/fingers_link)
    target_frame = 'puppet_left/ee_gripper_link'

    # Loop to continuously get and print the position and orientation
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Get position and orientation
        position, orientation = tf_listener.get_position_orientation(target_frame)

        if position and orientation:
            rospy.loginfo("Position: %s", position)
            rospy.loginfo("Orientation: %s", orientation)
        else:
            rospy.logerr("Failed to get the transform data.")
        
        # Sleep to maintain the rate (10 Hz)
        rate.sleep()
