#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import json
import time

# Initialize variables to store joint positions and the start time
current_positions = []
start_time = None

# Callback function that gets called whenever new data is published
def joint_state_callback(msg):
    global current_positions
    # Update the current joint positions
    current_positions = list(msg.position)[:6]

def main():
    global start_time
    # Initialize the ROS node
    rospy.init_node('print_joint_positions', anonymous=True)

    # Subscribe to the /puppet_left/joint_states topic
    rospy.Subscriber('/puppet_left/joint_states', JointState, joint_state_callback)

    # Record start time
    start_time = time.time()
    
    trajectory = []
    rate = rospy.Rate(0.5)  # 0.5 Hz corresponds to every 2 seconds

    while not rospy.is_shutdown():
        if current_positions:
   
            elapsed_time = round(time.time() - start_time, 1)
            trajectory.append({elapsed_time: current_positions})

            rospy.loginfo(f"Current joint positions: {trajectory[-1]}")

            with open('/home/dexweaver/Documents/GitHub/cucumber-harvesting/trajectory.txt', 'w') as file:
                json.dump(trajectory, file, indent=4)
        else:
            rospy.loginfo("No joint data yet.")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.loginfo(f"An error occurred: {str(e)}")
