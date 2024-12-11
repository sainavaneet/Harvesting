import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class ObjectPoseEstimator:
    def __init__(self):
        rospy.init_node('object_pose_estimator', anonymous=True)
        self.bridge = CvBridge()
        self.model = YOLO("/home/dexweaver/Github/cucumber-harvesting/object_detection/runs/detect/train/weights/best.pt").to("cuda")
        
        self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)

        self.last_depth_image = None
        self.color_image = None

    def depth_callback(self, msg):
        try:
            self.last_depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except CvBridgeError as e:
            rospy.logerr(e)

    def color_callback(self, msg):
        if self.last_depth_image is None:
            rospy.loginfo("Depth image not yet received.")
            return
        
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        results = self.model(self.color_image)
        resized_depth_image = cv2.resize(self.last_depth_image, (self.color_image.shape[1], self.color_image.shape[0]), interpolation=cv2.INTER_NEAREST)
        
        for result in results:
            for box in result.boxes:
                conf = box.conf
                if conf < 0.75:
                    continue
                
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                x_center = (x1 + x2) // 2
                y_center = (y1 + y2) // 2
                
                depth_value = resized_depth_image[y_center, x_center]

                rospy.loginfo(f"Depth at center: {depth_value:.2f} meters")
                
                cv2.rectangle(self.color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(self.color_image, (x_center, y_center), 5, (0, 0, 255), -1)
                cv2.putText(self.color_image, f"Depth: {depth_value:.2f}m", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.imshow("Depth Estimation", self.color_image)
        cv2.waitKey(1)

    def main(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    detector = ObjectPoseEstimator()
    detector.main()
