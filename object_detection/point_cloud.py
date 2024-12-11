#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class ObjectDetector:
    def __init__(self):
        model_path = "/home/dexweaver/Github/cucumber-harvesting/object_detection/runs/detect/train2/weights/best.pt"
        self.model = YOLO(model_path)

        rospy.init_node('yolo_object_detection', anonymous=True)
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.point_cloud_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.point_cloud_callback)
        self.current_point_cloud = None

    def point_cloud_callback(self, data):
        self.current_point_cloud = data

    def image_callback(self, data):
        if not self.current_point_cloud:
            rospy.logwarn("No point cloud data received yet")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
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
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                
                # Get the depth information from the point cloud at the center point of the detected object
                center_depth = self.get_depth_at_pixel(center_x, center_y)

                # Draw the bounding boxes, labels, and center point
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"Depth: {center_depth:.2f}m", (center_x, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
        return frame
    def get_depth_at_pixel(self, x, y):
        # Check if the point cloud is set
        if not self.current_point_cloud:
            rospy.logwarn("Point cloud data is not available.")
            return 0.0

        # Ensure the x, y coordinates are within the image dimensions
        width = self.current_point_cloud.width
        height = self.current_point_cloud.height
        if x >= width or y >= height:
            rospy.logwarn("Requested pixel (x={}, y={}) is out of point cloud bounds (width={}, height={}).".format(x, y, width, height))
            return 0.0

        # Calculate the index for the organized point cloud
        index = (y * width) + x
        if not hasattr(self, 'point_list') or self.point_list is None:
            # Convert the entire point cloud to a list
            self.point_list = list(pc2.read_points(self.current_point_cloud, skip_nans=False))

        if index >= len(self.point_list):
            rospy.logwarn("Point index {} is out of the list bounds.".format(index))
            return 0.0

        # Access the point and extract depth
        point = self.point_list[index]
        depth = point[2]  # z coordinate is the depth
        if depth is None or np.isnan(depth):
            rospy.logwarn("Depth at index {} is NaN or None.".format(index))
            return 0.0

        rospy.loginfo("Depth at (x={}, y={}) is {:.3f} meters.".format(x, y, depth))
        return depth


    def main(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down")
            cv2.destroyAllWindows()

if __name__ == "__main__":
    od = ObjectDetector()
    od.main()
