#!/usr/bin/env python
import rospy
import rospkg
import csv
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import tf

class YellowEllipseDetector:
    def __init__(self):
        self.node_name = "yellow_ellipse_detector"
        rospy.init_node(self.node_name)
        self.x = self.y = self.yaw = 0.0
        self.x_loc = self.y_loc = 0.0
        self.i = 0
        # Subscriber to the camera image topic
        self.image_sub = rospy.Subscriber("/asc/camera_image", Image, self.image_callback)

        self.xypis_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        # To convert ROS images to OpenCV format
        self.bridge = CvBridge()

        self.save_path = self.get_save_path()

    #def save_image(self, image, time):
      # f"{self.save_path}/mask_{int(time)}.png"
      # cv2.imwrite(filename, image)
       #rospy.loginfo(f"Saved mask image at {filename}")

    def odom_callback(self, data:Odometry):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y 
        self.yaw = self.quat_to_euler(data.pose.pose.orientation)

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
        lower_yellow = np.array([20, 100, 150])
        upper_yellow = np.array([30, 255, 255])
        
        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # Fit ellipses to contours found
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= 8000:
                #print("total area is:",area)
                ## if len(contour) >= 5:  # Need at least 5 points to fit ellipse
                ellipse = cv2.fitEllipse(contour)
                (x,y),(MA,ma),angle = ellipse
                if x>=400 and x<=1280 and y>=840 and y<=1050:
                #curve fitting results to calculate location of object
                    dx=5.107+0.001285*x-0.006314*y-0.000000104*x*x-0.000001056*x*y+0.000002204*y*y
                    dy=-43.71+0.001276*x+0.09346*y-0.00000003025*x*x-0.0000002705*x*y-0.00005073*y*y
                    #print('test', self.yaw)
                    if self.yaw >=-np.pi/2 and self.yaw <= np.pi/2:
                       x_loc = self.x+dx
                       y_loc = self.y-dy
                    if self.yaw >= np.pi/2 or self.yaw <= -np.pi/2:
                       x_loc = self.x-dx
                       y_loc = self.y+dy
                    #print("location x",x_loc,"and",y_loc)
                # Save the data
                self.data(x, y, area)
                
                cv2.ellipse(mask, ellipse, (0, 255, 0), 2)
                cv2.imshow("22",mask)
                #print("amd,yes")
                #print("location are",self.x,self.y)
                #print("dx & dy are",dx, dy)
                # self.save_image(cv_image,1)
                #cv2.imshow("33",binary_image)
        # Display the resulting frame
        
        cv2.waitKey(3)

	# compare
        distance_object = np.sqrt((self.x_loc-x_loc)**2+(self.y_loc-y_loc)**2)
        if distance_object >= 1:
               print("yes, amd",distance_object)
               save_directory = '/home/asc/saved_camera_images'
               filename = f"{save_directory}/image{self.i}.jpg"
               cv2.imwrite(filename,cv_image)
               self.i=self.i+1
        #print(self.x_loc,"and",x_loc)
        self.x_loc = x_loc
        self.y_loc = y_loc
        

    def data(self, x, y, area):
        with open(f'{self.save_path}yellow_detected.csv', 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([x, y, area])

    def get_save_path(self):
        rospack = rospkg.RosPack()
        return rospack.get_path("localisation") + '/src/data/'

    @staticmethod
    def quat_to_euler(quat):
        if quat is not None:
            euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            return euler[2]  # return the yaw, which is the orientation around z-axis
        else:
            return 0  # If intialised to None, return 0

def main():
    try:
        yellow_ellipse_detector = YellowEllipseDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
