import rospy
from yhs_can_msgs.msg import ctrl_cmd
import time

class BaseMove:
    def __init__(self):
        rospy.init_node('auto_teleop_node')
        self.pub = rospy.Publisher('/ctrl_cmd', ctrl_cmd, queue_size=10)

    def publish_cmd(self, x_linear, y_linear):
        ctrl_cmd_msg = ctrl_cmd()
        ctrl_cmd_msg.ctrl_cmd_gear = 6  # Default move target gear
        ctrl_cmd_msg.ctrl_cmd_x_linear = x_linear
        ctrl_cmd_msg.ctrl_cmd_y_linear = y_linear
        ctrl_cmd_msg.ctrl_cmd_z_angular = 0
        self.pub.publish(ctrl_cmd_msg)

    def forward(self):
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < 3.3:
            self.publish_cmd(0.2, 0.2) 
            rospy.sleep(0.1)  
        
        
        self.publish_cmd(0.0, 0.0)



if __name__ == "__main__":
    controller = BaseMove()
    controller.forward()