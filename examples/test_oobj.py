import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

def object_position_transformed(object_pos_camera_frame):
    # Initialize ROS node (if not already initialized)
    rospy.init_node('object_transform_listener', anonymous=True)

    # Create a TF2 listener
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    
    # Define the object position in camera frame as a PointStamped
    object_point = PointStamped()
    object_point.header.frame_id = "camera_frame_id" 
    object_point.header.stamp = rospy.Time.now()
    object_point.point.x = object_pos_camera_frame['x']
    object_point.point.y = object_pos_camera_frame['y']
    object_point.point.z = object_pos_camera_frame['z']

    # Wait for the transform to be available
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # Look up the transformation from camera frame to gripper link frame
            transform = tf_buffer.lookup_transform('puppet_left/ee_gripper_link', 'camera_frame_id', rospy.Time(0), rospy.Duration(1.0))
            
            # Transform the object's position to the gripper link frame
            transformed_point = tf2_geometry_msgs.do_transform_point(object_point, transform)
            print("Object position in gripper link frame:", transformed_point.point)
            return transformed_point.point
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            rate.sleep()


if __name__ == "__main__":
    object_pos_camera_frame = {'x': 1.0, 'y': 2.0, 'z': 3.0}  # Example positions
    object_position_transformed(object_pos_camera_frame)
