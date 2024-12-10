#!/usr/bin/env python

import rospy
from math import sin, cos
from geometry_msgs.msg import Twist, Quaternion, Pose, Point, Vector3
from nav_msgs.msg import Odometry
from yhs_can_msgs.msg import lf_wheel_fb, rf_wheel_fb  # Correctly import custom message types
from tf.broadcaster import TransformBroadcaster
import tf

class OdometryCalculator:
    def __init__(self):
        rospy.init_node('odometry_calculator')

        # Subscribers to wheel feedback using the correct message types
        rospy.Subscriber("/lf_wheel_fb", lf_wheel_fb, self.left_wheel_callback)
        rospy.Subscriber("/rf_wheel_fb", rf_wheel_fb, self.right_wheel_callback)

        # Publisher for odometry
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
        self.odom_broadcaster = TransformBroadcaster()

        self.left_wheel_pulses = 0
        self.right_wheel_pulses = 0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_time = rospy.Time.now()

    def left_wheel_callback(self, msg):
        # Use pulse count for odometry calculations
        self.left_wheel_pulses = msg.lf_wheel_fb_pulse

    def right_wheel_callback(self, msg):
        # Use pulse count for odometry calculations
        self.right_wheel_pulses = msg.rf_wheel_fb_pulse

    def update_odometry(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time

        # Constants for conversion and robot geometry
        # You will need to determine the distance per pulse
        pulse_to_distance = 0.002007  # Placeholder value: distance traveled per pulse
        axle_length = 1.0  # Distance between wheels

        # Calculate wheel displacement based on pulses
        left_distance = self.left_wheel_pulses * pulse_to_distance
        right_distance = self.right_wheel_pulses * pulse_to_distance
        self.left_wheel_pulses = 0
        self.right_wheel_pulses = 0

        # Average distance
        distance = (left_distance + right_distance) / 2

        # Update theta
        delta_th = (right_distance - left_distance) / axle_length
        delta_x = distance * cos(self.th + delta_th / 2)
        delta_y = distance * sin(self.th + delta_th / 2)

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Create quaternion from theta
        odom_quat = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, self.th))

        # First, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            (odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w),
            current_time,
            "base_link",
            "odom"
        )

        # Next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), odom_quat)
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(distance / dt, 0, 0), Vector3(0, 0, delta_th / dt))

        self.odom_pub.publish(odom)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.update_odometry()
            rate.sleep()

if __name__ == '__main__':
    odom_calc = OdometryCalculator()
    odom_calc.run()
