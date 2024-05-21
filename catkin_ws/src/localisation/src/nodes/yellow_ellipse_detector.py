#!/usr/bin/env python
import rospy
import rospkg
import csv
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from asclinic_pkg.msg import LeftRightFloat32
from nav_msgs.msg import Odometry
import tf
from std_msgs.msg import Float32MultiArray

class YellowEllipseDetector:
    def __init__(self):
        self.node_name = "yellow_ellipse_detector"
        rospy.init_node(self.node_name)
        self.x = self.y = self.yaw = 0.0
        # self.x_loc = self.y_loc = 0.0
        self.xlocs = self.ylocs = []
        self.i = 0
        # Subscriber to the camera image topic
        self.image_sub = rospy.Subscriber("/asc/camera_image", Image, self.image_callback)

        self.xypis_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        self.pub_yellow = rospy.Publisher('/object_location', LeftRightFloat32, queue_size=10)
        # self.pub_blue = rospy.Publisher('/object_location_blue', LeftRightFloat32, queue_size=10)
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
        
        lower_blue = np.array([80, 100 ,100])
        upper_blue = np.array([110, 255 ,255])
        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # Fit ellipses to contours found
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= 8000:
                #print("total area is:",area)
                ## if len(contour) >= 5:  # Need at least 5 points to fit ellipse
                ellipse = cv2.fitEllipse(contour)
                (x,y),(MA,ma),angle = ellipse
                ##print('x:',x,'y',y)
                if x>=400 and x<=1480 and y>=840 and y<=1050:
                    
                #curve fitting results to calculate location of object
                    dx=5.107+0.001285*x-0.006314*y-0.000000104*x*x-0.000001056*x*y+0.000002204*y*y
                    dy=-43.71+0.001276*x+0.09346*y-0.00000003025*x*x-0.0000002705*x*y-0.00005073*y*y
                    #print('yellow', self.yaw, '\n',self.x,'and',self.y)
                    if self.yaw >= -np.pi/4 and self.yaw <= np.pi/4:
                        x_loc = self.x + dx
                        y_loc = self.y - dy
                        # print(1)
                    if  self.yaw >= np.pi/4 and self.yaw <= 0.75*np.pi:
                        x_loc = self.x + dy
                        y_loc = self.y + dx
                    if  self.yaw <= -np.pi/4 and self.yaw >= -0.75*np.pi:
                        x_loc = self.x - dy
                        y_loc = self.y - dx
                    if  self.yaw >=0.75*np.pi or self.yaw <= -0.75*np.pi:
                        x_loc = self.x - dx
                        y_loc = self.y + dy

                    self.pub_yellow.publish(LeftRightFloat32(left=x_loc, right=y_loc))
                    #print("location x:",x_loc,"and y:", y_loc )
                    #print('\n dx ',dx, '&dy ',dy,'\n',x,'&',y,'x_loc: ',x_loc,'y_loc: ',y_loc,)
        for contour in contours_blue:
            area = cv2.contourArea(contour)
            if area >= 8000:
                ellipse = cv2.fitEllipse(contour)
                (x,y),(MA,ma),angle = ellipse
                if x>=400 and x<=1480 and y>=840 and y<=1050:
                    dx=5.107+0.001285*x-0.006314*y-0.000000104*x*x-0.000001056*x*y+0.000002204*y*y
                    dy=-43.71+0.001276*x+0.09346*y-0.00000003025*x*x-0.0000002705*x*y-0.00005073*y*y
                    # print('blue', self.yaw, '\n',self.x,'and',self.y)
                    if self.yaw >= -np.pi/4 and self.yaw <= np.pi/4:
                        x_loc_blue = self.x + dx
                        y_loc_blue = self.y - dy
                        
                    if  self.yaw >= np.pi/4 and self.yaw <= 0.75*np.pi:
                        x_loc_blue = self.x + dy
                        y_loc_blue = self.y + dx
                    if  self.yaw <= -np.pi/4 and self.yaw >= -0.75*np.pi:
                        x_loc_blue = self.x - dy
                        y_loc_blue = self.y - dx
                    if  self.yaw >=0.75*np.pi or self.yaw <= -0.75*np.pi:
                        x_loc_blue = self.x - dx
                        y_loc_blue = self.y + dy

                    self.pub_yellow.publish(LeftRightFloat32(left=x_loc_blue, right=y_loc_blue))
                # Save the data
                #self.data(x, y, area)
                #cv2.ellipse(mask, ellipse, (0, 255, 0), 2)
        # cv2.imshow("22", mask_blue)
                #print("amd,yes")
                #print("location are",self.x,self.y)
                #print("dx & dy are",dx, dy)
                # self.save_image(cv_image,1)
                #cv2.imshow("33",binary_image)
        # Display the resulting frame
        
        cv2.waitKey(3)

	    # compare
        # distance_object = np.sqrt((self.x_loc-x_loc)**2+(self.y_loc-y_loc)**2)
        # if distance_object >= 1:
        # save_directory = self.save_path
        # filename = f"{save_directory}{x_loc:.3f},{y_loc:.3f}.png"  # Save the current image using its location
        # cv2.imwrite(filename,cv_image)
        # self.i=self.i+1
        #print('location are x:',x_loc,'y:',y_loc)
        #print(self.x_loc,"and",x_loc)
        # self.x_loc = x_loc
        # self.y_loc = y_loc

        # Save as a list of locations
        # self.xlocs.append(x_loc)
        # self.ylocs.append(y_loc)

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
