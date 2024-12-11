from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy

class ImageViewer:
    def __init__(self):
        rospy.init_node('image_viewer', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

    def image_callback(self, data):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Convert to grayscale
            cv_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Display the grayscale image
            cv2.imshow("Image Window", cv_gray)
            cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        except Exception as e:
            rospy.logerr("Unexpected error: {0}".format(e))

    def main(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        finally:
            cv2.destroyAllWindows()

if __name__ == '__main__':
    viewer = ImageViewer()
    viewer.main()
