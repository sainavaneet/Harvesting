#! /usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
from __future__ import absolute_import
import rospy
from geometry_msgs.msg import Twist
from yhs_can_msgs.msg import ctrl_cmd
from std_msgs.msg import String
import sys, select, termios, tty

class TeleopController:
    X_LIN_VEL_STEP_SIZE = 0.1  # Step size for linear X velocity
    Y_LIN_VEL_STEP_SIZE = 0.1  # Step size for linear Y velocity
    Z_ANG_VEL_STEP_SIZE = 5.0  # Step size for angular velocity

    X_MAX_LIN_VEL = 2.0        # Max linear X velocity
    Y_MAX_LIN_VEL = 2.0        # Max linear Y velocity
    Z_MAX_ANG_VEL = 90.0       # Max angular velocity

    CMD_STOP = 0
    CMD_FORWARD = 1
    CMD_LEFT = 2
    CMD_RIGHT = 3

    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        rospy.init_node('fwmid_teleop_node')
        self.pub = rospy.Publisher('/ctrl_cmd', ctrl_cmd, queue_size=10)
        self.rate = rospy.Rate(100)
        self.x_linear = 0.0
        self.y_linear = 0.0

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self):
        return "Linear velocities -> X: {} m/s, Y: {} m/s".format(self.x_linear, self.y_linear)

    def constrain(self, input_val, low, high):
        return max(low, min(input_val, high))

    def publish_cmd(self):
        ctrl_cmd_msg = ctrl_cmd()
        ctrl_cmd_msg.ctrl_cmd_gear = 6  # Default move target gear
        ctrl_cmd_msg.ctrl_cmd_x_linear = self.x_linear
        ctrl_cmd_msg.ctrl_cmd_y_linear = self.y_linear
        ctrl_cmd_msg.ctrl_cmd_z_angular = 0
        self.pub.publish(ctrl_cmd_msg)

    def handle_movement(self):
        while not rospy.is_shutdown():
            key = self.get_key()
            if key == '\x03':  # CTRL-C
                break
            elif key == '\x1B[A':  # Up arrow
                self.x_linear = self.constrain(self.x_linear + self.X_LIN_VEL_STEP_SIZE, -self.X_MAX_LIN_VEL, self.X_MAX_LIN_VEL)
            elif key == '\x1B[B':  # Down arrow
                self.x_linear = self.constrain(self.x_linear - self.X_LIN_VEL_STEP_SIZE, -self.X_MAX_LIN_VEL, self.X_MAX_LIN_VEL)
            elif key == '\x1B[C':  # Right arrow
                self.y_linear = self.constrain(self.y_linear + self.Y_LIN_VEL_STEP_SIZE, -self.Y_MAX_LIN_VEL, self.Y_MAX_LIN_VEL)
            elif key == '\x1B[D':  # Left arrow
                self.y_linear = self.constrain(self.y_linear - self.Y_LIN_VEL_STEP_SIZE, -self.Y_MAX_LIN_VEL, self.Y_MAX_LIN_VEL)
            elif key == 's':
                self.x_linear = 0.0
                self.y_linear = 0.0

            print(self.vels())
            self.publish_cmd()
            self.rate.sleep()

if __name__ == "__main__":
    controller = TeleopController()
    controller.handle_movement()
