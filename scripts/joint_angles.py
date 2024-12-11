#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

# Store joint positions
joint_positions = []

# Callback function that gets called whenever new data is published
def joint_state_callback(msg):
    global joint_positions
    # Store the positions from the JointState message
    joint_positions = msg.position

def main():
    # Initialize the ROS node
    rospy.init_node('print_joint_positions', anonymous=True)
    
    # Subscribe to the /puppet_left/joint_states topic
    rospy.Subscriber('/puppet_left/joint_states', JointState, joint_state_callback)
    
    # Keep the node running and printing every second
    rate = rospy.Rate(0.5)  # 1 Hz (1 second)
    
    while not rospy.is_shutdown():
        if joint_positions:
            # Print one joint position (e.g., first joint's position)
            rospy.loginfo(f"Joint position: {joint_positions[:6]}")  # Prints the position of the first joint
        else:
            rospy.loginfo("No joint data yet.")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
