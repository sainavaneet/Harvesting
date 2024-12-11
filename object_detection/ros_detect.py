#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class ObjectDetector:
    def __init__(self):
      
        model_path = "/home/dexweaver/Github/cucumber-harvesting/object_detection/runs/detect/train2/weights/best.pt"
        self.model = YOLO(model_path)

      
        rospy.init_node('yolo_object_detection', anonymous=True)
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

    def image_callback(self, data):
        try:
            
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("YOLOv8 Real-Time Detection", cv_image)
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        

        processed_image = self.process_frame(cv_image)

    
        cv2.imshow("YOLOv8 Real-Time Detection", processed_image)
        cv2.waitKey(1)

    def process_frame(self, frame):
        results = self.model(frame)

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = box.conf[0]
                cls = int(box.cls[0])
                label = self.model.names[cls]

                # Draw the bounding boxes and labels
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        return frame

    def main(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down")
            cv2.destroyAllWindows()

if __name__ == "__main__":
    od = ObjectDetector()
    od.main()
