import rospy
import tf2_ros
import tf2_geometry_msgs
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from ultralytics import YOLO
from geometry_msgs.msg import PoseStamped
import tf
from geometry_msgs.msg import PointStamped
import os
import sys
import time
os.environ["YOLO_LOG"] = "0"

class ee_pose:
    def __init__(self):
        self.listener = tf.TransformListener()

    def transform_to_frame(self, from_frame, to_frame, position):
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform(to_frame, from_frame, now, rospy.Duration(3.0))
            point = PointStamped()
            point.header.frame_id = from_frame
            point.header.stamp = now
            point.point.x = position[0]
            point.point.y = position[1]
            point.point.z = position[2]
            transformed_point = self.listener.transformPoint(to_frame, point)
            return transformed_point.point.x, transformed_point.point.y, transformed_point.point.z
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
            rospy.logwarn(f"Transformation failed: {ex}")
            return None, None, None

class ObjectPoseEstimator:
    def __init__(self):
        rospy.init_node('object_pose_estimator', anonymous=True)
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        model_path = "/home/dexweaver/Github/cucumber-harvesting/object_detection/runs/detect/train/weights/best.pt"

        self.model = YOLO(model_path).to("cuda")

        self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.color_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.color_info_callback)
        self.depth_info_sub = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.depth_info_callback)
        self.tf_listener = ee_pose()

        self.x = 0
        self.y = 0
        self.z = 0
        self.depth_image = None
        self.color_intrinsics = None
        self.depth_intrinsics = None

        # Variables for pose stability check
        self.last_pose = None  # Stores the last detected pose
        self.pose_stable_count = 0  # Counter for how long the pose has been stable
        self.stable_threshold = 100  
        self.fixed_cucumber_pose = None  
        self.last_logged_cucumber = None 
        self.stable = False  # To track if the pose has been stable for the required time

    def color_info_callback(self, msg):
        self.color_intrinsics = np.array(msg.K).reshape((3, 3))

    def depth_info_callback(self, msg):
        self.depth_intrinsics = np.array(msg.K).reshape((3, 3))

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def color_callback(self, msg):
        if self.depth_image is None or self.color_intrinsics is None or self.depth_intrinsics is None:
            return

        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(self.color_image)
        resized_depth_image = cv2.resize(self.depth_image, (self.color_image.shape[1], self.color_image.shape[0]), interpolation=cv2.INTER_NEAREST)

        for result in results:
            for box in result.boxes:
                conf = box.conf[0]
                if conf < 0.85:  # Only consider detections with confidence above 0.85
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = "cucumber"  # You have a fixed label as cucumber
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                depth = resized_depth_image[center_y, center_x]


                width = x2 - x1
                height = y2 - y1
                area = width * height

                if depth > 0:
                    x, y, z = self.pixel_to_camera(center_x, center_y, depth, self.color_intrinsics)
                    y_cm, z_cm, x_cm = x, y, z /10

                    y_cm = -y_cm

                    z_cm = -z_cm

                    if x_cm is not None and y_cm is not None and z_cm is not None:
                        if self.last_pose and (abs(x_cm - self.last_pose[0]) < 2 and abs(y_cm - self.last_pose[1]) < 2 and abs(z_cm - self.last_pose[2]) < 2):
                            self.pose_stable_count += 1
                        else:
                            self.pose_stable_count = 0  # Reset if pose is not stable

                        self.last_pose = (x_cm, y_cm, z_cm)

                        if self.pose_stable_count >= self.stable_threshold:
                            self.stable = True
                            self.fixed_cucumber_pose = (x_cm, y_cm, z_cm)
                            rospy.loginfo(f"Cucumber pose stable: x={x_cm:.2f} cm, y={y_cm:.2f} cm, z={z_cm:.2f} cm")
                            self.stop_detection()  # Stop object detection

                        cv2.rectangle(self.color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(self.color_image, (center_x, center_y), 5, (0, 0, 255), -1)
                        cv2.putText(self.color_image, f"{label} ({x_cm:.2f}, {y_cm:.2f}, {z_cm:.2f} cm) Area={area} px²", (center_x + 10, center_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        cv2.imshow("YOLO Object Detection with Pose", self.color_image)
        cv2.waitKey(1)

    def stop_detection(self):
        rospy.signal_shutdown("Pose stabilized for 5 seconds, stopping detection.")

    def use_stable_pose(self):
        return self.fixed_cucumber_pose

    def pixel_to_camera(self, u, v, depth, intrinsics):
        fx, fy = intrinsics[0, 0], intrinsics[1, 1]
        cx, cy = intrinsics[0, 2], intrinsics[1, 2]
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        return x, y, z

    def main(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == "__main__":

    transform = ObjectPoseEstimator()
    transform.main()