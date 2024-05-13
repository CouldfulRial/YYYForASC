#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class YellowEllipseDetector:
    def __init__(self):
        self.node_name = "yellow_ellipse_detector"
        rospy.init_node(self.node_name)

        # Subscriber to the camera image topic
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        
        # To convert ROS images to OpenCV format
        self.bridge = CvBridge()

    def image_callback(self, data):
        try:
            # Convert the image from ROS to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        
        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define range of yellow color in HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        
        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Fit ellipses to contours found
        for contour in contours:
            if len(contour) >= 5:  # Need at least 5 points to fit ellipse
                ellipse = cv2.fitEllipse(contour)
                cv2.ellipse(cv_image, ellipse, (0, 255, 0), 2)

        # Display the resulting frame
        cv2.imshow('Detected Yellow Ellipses', cv_image)
        cv2.waitKey(3)

def main():
    try:
        yellow_ellipse_detector = YellowEllipseDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()