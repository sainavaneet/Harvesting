import rospy
from geometry_msgs.msg import Twist
from yhs_can_msgs.msg import ctrl_cmd
from std_msgs.msg import String
import sys
import threading
import traceback
import tkinter as tk
from tkinter import ttk

# Constants
X_LIN_VEL_STEP_SIZE = 0.01
Y_LIN_VEL_STEP_SIZE = 0.01
Z_ANG_VEL_STEP_SIZE = 5.0
DEFAULT_MOVE_TARGET_GEAR = 6
X_MAX_LIN_VEL = 1.0
Y_MAX_LIN_VEL = 1.0
Z_MAX_ANG_VEL = 90.0
DEFAULT_FWORD_SPEED = 0.2
DEFAULT_BACK_SPEED = 0.2

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

def start_move():
    global move_status
    move_status = CMD_START

def stop_move():
    global move_status
    move_status = CMD_STOP

def pause_move():
    global move_status
    move_status = CMD_PAUSE

def resume_move():
    global move_status
    move_status = CMD_RESUME

def move_backwards():
    global move_status
    move_status = CMD_x

def on_linear_x_slider_change(val):
    global target_x_linear_vel
    target_x_linear_vel = float(val)

def on_angular_z_slider_change(val):
    global target_angular_vel
    target_angular_vel = float(val)

def on_linear_y_slider_change(val):
    global target_y_linear_vel
    target_y_linear_vel = float(val)

def main():
    global pub, rate

    rospy.init_node("fwmid_teleop_node")
    pub = rospy.Publisher("ctrl_cmd", ctrl_cmd, queue_size=10)
    rate = rospy.Rate(100)

    # Tkinter GUI
    root = tk.Tk()
    root.title("Robot Control")
    root.geometry("300x200")  

    # Start button
    start_button = tk.Button(root, text="forward", command=start_move)
    start_button.grid(row=0, column=0)

    # Stop button
    stop_button = tk.Button(root, text="Stop", command=stop_move)
    stop_button.grid(row=0, column=1)

    # Pause button
    pause_button = tk.Button(root, text="Pause", command=pause_move)
    pause_button.grid(row=0, column=2)

    # Resume button
    resume_button = tk.Button(root, text="Resume", command=resume_move)
    resume_button.grid(row=0, column=3)

    # Move backwards button
    move_back_button = tk.Button(root, text="Backwards", command=move_backwards)
    move_back_button.grid(row=1, column=0, columnspan=4)
    try:
        while not rospy.is_shutdown():
            move_cmd(move_status)
            root.update_idletasks()
            root.update()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        stop_move()  
        rospy.loginfo("Shutting down ROS node and Tkinter GUI.")
    finally:
        
        root.quit()
        root.destroy()

if __name__ == "__main__":
    main()
