#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from yhs_can_msgs.msg import ctrl_cmd
from std_msgs.msg import String
import sys, select, termios, tty, threading
import traceback

# Constants
X_LIN_VEL_STEP_SIZE = 0.1
Y_LIN_VEL_STEP_SIZE = 0.1
Z_ANG_VEL_STEP_SIZE = 5.0
DEFAULT_MOVE_TARGET_GEAR = 6
X_MAX_LIN_VEL = 2.0
Y_MAX_LIN_VEL = 2.0
Z_MAX_ANG_VEL = 90.0
DEFAULT_FWORD_SPEED = 0.01
DEFAULT_BACK_SPEED = 0.01

CMD_STOP = 0
CMD_START = 1
CMD_PAUSE = 2
CMD_RESUME = 3
CMD_x = 16
CMD_m = 17
CMD_l = 18

# Global Variables
move_status = CMD_STOP
target_gear = 0
target_x_linear_vel = 0.0
target_y_linear_vel = 0.0
target_angular_vel = 0.0
control_x_linear_vel = 0.0
control_y_linear_vel = 0.0
control_angular_vel = 0.0
rate = None
pub = None
key_thread_active = True


def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
    return key


def keyboard_listener():
    global move_status, key_thread_active
    try:
        while key_thread_active:
            key = get_key()
            if key == 'w':
                move_status = CMD_START
            elif key == 's':
                move_status = CMD_STOP
            elif key == 'r':
                move_status = CMD_RESUME
            elif key == 'p':
                move_status = CMD_PAUSE
            elif key == 'b':
                move_status = CMD_x
            elif key == '\x03':  # Ctrl+C to exit
                rospy.signal_shutdown("Keyboard interrupt")
                break
    except Exception as ex:
        rospy.logerr(f"Error in keyboard_listener: {traceback.format_exc()}")


def vels(target_gear, target_x_linear_vel, target_y_linear_vel, target_angular_vel):
    return f"currently: gear {target_gear}, x_linear vel {target_x_linear_vel}, " \
           f"y_linear vel {target_y_linear_vel}, angular vel {target_angular_vel}"


def constrain(input, low, high):
    return max(low, min(input, high))


def check_linear_x_limit_velocity(vel):
    return constrain(vel, -X_MAX_LIN_VEL, X_MAX_LIN_VEL)


def check_linear_y_limit_velocity(vel):
    return constrain(vel, -Y_MAX_LIN_VEL, Y_MAX_LIN_VEL)


def check_angular_limit_velocity(vel):
    return constrain(vel, -Z_MAX_ANG_VEL, Z_MAX_ANG_VEL)


def make_simple_profile(output, input, slop):
    if input > output:
        return min(input, output + slop)
    elif input < output:
        return max(input, output - slop)
    else:
        return input


def pose_cmd_cb(data):
    global move_status

    cmd = data.data
    command_map = {
        "start": CMD_START,
        "stop": CMD_STOP,
        "resume": CMD_RESUME,
        "pause": CMD_PAUSE,
        "x": CMD_x
    }
    move_status = command_map.get(cmd, move_status)


def move_cmd(cmd):
    global target_gear, target_x_linear_vel, target_y_linear_vel, target_angular_vel
    global control_x_linear_vel, control_y_linear_vel, control_angular_vel

    try:
        if cmd == CMD_STOP:
            target_gear = 0
            target_x_linear_vel = 0.0
            target_y_linear_vel = 0.0
            target_angular_vel = 0.0

        elif cmd == CMD_START:
            target_gear = DEFAULT_MOVE_TARGET_GEAR
            target_x_linear_vel = check_linear_x_limit_velocity(DEFAULT_FWORD_SPEED)

        elif cmd == CMD_PAUSE:
            target_x_linear_vel = 0.0

        elif cmd == CMD_RESUME:
            target_gear = DEFAULT_MOVE_TARGET_GEAR
            target_x_linear_vel = check_linear_x_limit_velocity(DEFAULT_FWORD_SPEED)

        elif cmd == CMD_x:
            target_gear = DEFAULT_MOVE_TARGET_GEAR
            target_x_linear_vel = check_linear_x_limit_velocity(-DEFAULT_BACK_SPEED)

        # Publish the updated control command
        ctrl_cmd_msg = ctrl_cmd()
        control_x_linear_vel = make_simple_profile(control_x_linear_vel, target_x_linear_vel, X_LIN_VEL_STEP_SIZE)
        control_y_linear_vel = make_simple_profile(control_y_linear_vel, target_y_linear_vel, Y_LIN_VEL_STEP_SIZE)
        control_angular_vel = make_simple_profile(control_angular_vel, target_angular_vel, Z_ANG_VEL_STEP_SIZE)

        ctrl_cmd_msg.ctrl_cmd_gear = target_gear
        ctrl_cmd_msg.ctrl_cmd_x_linear = control_x_linear_vel
        ctrl_cmd_msg.ctrl_cmd_y_linear = control_y_linear_vel
        ctrl_cmd_msg.ctrl_cmd_z_angular = control_angular_vel

        pub.publish(ctrl_cmd_msg)
        # rospy.loginfo(vels(target_gear, control_x_linear_vel, control_y_linear_vel, control_angular_vel))
        rate.sleep()
    except Exception as ex:
        rospy.logerr(f"Error in move_cmd: {traceback.format_exc()}")


if __name__ == "__main__":
    rospy.init_node("fwmid_teleop_node")
    pub = rospy.Publisher("ctrl_cmd", ctrl_cmd, queue_size=10)
    rospy.Subscriber("pose_command", String, pose_cmd_cb)
    rate = rospy.Rate(100)

    # Start keyboard listener thread
    key_thread = threading.Thread(target=keyboard_listener)
    key_thread.daemon = True
    key_thread.start()

    print("Node initialized. Listening for keyboard commands...")

    try:
        while not rospy.is_shutdown():
            # print(f"Current move_status: {move_status}")
            move_cmd(move_status)
    except rospy.ROSInterruptException:
        # print("ROS Interrupt Exception. Shutting down...")
        key_thread_active = False
        key_thread.join()
    except KeyboardInterrupt:
        # print("Keyboard Interrupt. Shutting down...")
        key_thread_active = False
        key_thread.join()
