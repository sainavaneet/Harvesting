import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import tf
from geometry_msgs.msg import PointStamped
import os
from decimal import Decimal, ROUND_HALF_UP

from config import *

class ee_pose:
    def __init__(self):
        self.listener = tf.TransformListener()

    def transform_to_frame(self, from_frame, to_frame, position):
        now = rospy.Time.now()
        self.listener.waitForTransform(to_frame, from_frame, now, rospy.Duration(3.0))
        point = PointStamped()
        point.header.frame_id = from_frame
        point.header.stamp = now
        point.point.x = position[0]
        point.point.y = position[1]
        point.point.z = position[2]
        transformed_point = self.listener.transformPoint(to_frame, point)
        return (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z)

class ObjectPoseEstimator:
    def __init__(self):
        rospy.init_node('object_pose_estimator', anonymous=True)
        self.bridge = CvBridge()

        self.model_path = MODEL_PATH
        self.model = YOLO(MODEL_PATH).to("cuda")
        
        self.stable_threshold = THR  
        self.last_depth_image = None
        self.color_image = None

        self.detection_active = True

        self.intrinsic = INTRINSIC

        self.last_pose = None  

        self.should_exit = False

        self.fixed_cucumber_pose = None

    def initialize_subscribers(self):
        self.detection_active = True
        self.should_exit = False
        rospy.sleep(1)
        self.color_sub = rospy.Subscriber(COLOUR_IMAGE_TOPIC, Image, self.color_callback)
        self.depth_sub = rospy.Subscriber(DEPTH_IMAGE_TOPIC, Image, self.depth_callback)
        

    def shutdown_subscribers(self):
        if self.color_sub:
            self.color_sub.unregister()
        if self.depth_sub:
            self.depth_sub.unregister()
        self.detection_active = False


    def depth_callback(self, msg):
        try:
            self.last_depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except CvBridgeError as e:
            rospy.logerr(e)


    def use_stable_pose(self):
        return self.fixed_cucumber_pose

    def stop_detection(self):
        self.detection_active = False
        self.should_exit = True

        
    def show_image(self):
        if self.color_image is not None:
            cv2.imshow("Object Detection", self.color_image)
            cv2.waitKey(1)

    def color_callback(self , msg):

        if not self.detection_active or self.last_depth_image is None:
            return
    
        
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")


        label = "cucumber"  
        results = self.model(self.color_image)
        resized_depth_image = cv2.resize(self.last_depth_image, (self.color_image.shape[1], self.color_image.shape[0]), interpolation=cv2.INTER_NEAREST)
        for result in results:
            for box in result.boxes:
                conf = box.conf
                if conf < 0.4:
                    continue
                
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                x_center = (x1 + x2) // 2
                y_center = (y1 + y2) // 2
                
                dist = resized_depth_image[y_center, x_center]


                if dist > 0:
                    Xtemp = dist * (x_center - self.intrinsic[0][2]) / self.intrinsic[0][0]
                    Ytemp = dist * (y_center - self.intrinsic[1][2]) / self.intrinsic[1][1]
                    Ztemp = dist

                    theta = 0  
                    Xtarget = Xtemp 
                    # Xtarget = Xtemp - 0.035  

                    Ytarget = -(Ztemp * math.sin(theta) + Ytemp * math.cos(theta))
                    Ztarget = Ztemp * math.cos(theta) + Ytemp * math.sin(theta)


                    #changed for robotic arm 
                    x , y , z = Xtarget , Ytarget , Ztarget
                    # x_cm  , y_cm  , z_cm = x , y , z /10

                    y_cm, z_cm, x_cm = x, y, z /10

                    y_cm = -y_cm
                    z_cm = -z_cm
                    if x_cm is not None and y_cm is not None and z_cm is not None:
                        if self.last_pose and (abs(x_cm - self.last_pose[0]) < 2 and abs(y_cm - self.last_pose[1]) < 2 and abs(z_cm - self.last_pose[2]) < 2):
                            self.pose_stable_count += 1
                        else:
                            self.pose_stable_count = 0  

                            self.last_pose = (x_cm, y_cm, z_cm)


                        
                        if self.pose_stable_count >= self.stable_threshold:
                            self.stable = True
                            self.fixed_cucumber_pose = (x_cm, y_cm, z_cm)
                            rospy.loginfo(f"Cucumber pose stable: x={x_cm:.2f} cm, y={y_cm:.2f} cm, z={z_cm:.2f} cm")
                            self.stop_detection()  # Stop object detection


                        rospy.loginfo(f" x , y , z = {x_cm} , {y_cm} , {z_cm}")
                        cv2.rectangle(self.color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(self.color_image, (x_center, y_center), 3, (0, 0, 255), -1)
                        cv2.putText(self.color_image, f"{label} ({x_cm:.2f}, {y_cm:.2f}, {z_cm:.2f} )cm", (x_center + 10, y_center), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)


    def run(self):

        self.initialize_subscribers()
        try:
            while not rospy.is_shutdown()and not self.should_exit:
                self.show_image()
        finally:
            cv2.destroyAllWindows()
            self.shutdown_subscribers()



if __name__ == "__main__":
    detector = ObjectPoseEstimator()
    detector.run()
