
#! /usr/bin/env python

'''
This node provides the location esitmaion of the robot based on the camera

Subscribed topics:
    /asc/aruco_detections  [asclinic_pkg/FiducialMarkerArray]
Published topics:
    vodom                  [nav_msgs/Odometry]
    vodom_failure          [std_msgs/Bool]
'''

# Core imports
import rospy
import rospkg

# Messages imports
from asclinic_pkg.msg import FiducialMarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Bool

# Algorithms imports
import numpy as np
from tf.transformations import quaternion_from_euler, euler_from_matrix
import cv2

# Constants
X = 1
Y = 2
Z = 3
PSI = 4
TIMED_OUT = 0.3

class VisionBasedLocalisation:
    def __init__(self):
         # Initialise node
        self.node_name = 'vision_based_localisation'
        rospy.init_node(self.node_name)

        # Get user parameter
        try:
            self.parm  = rospy.get_param(self.node_name)
            self.verbosity = self.parm["verbosity"]
        except KeyError:
            self.verbosity = 1

        # Subscribers
        # rospy.Subscriber(topic_name, msg_type, callback_function)
        self.sub = rospy.Subscriber('/asc/aruco_detections', FiducialMarkerArray, self.marker_callback)

        # Track the last updated time
        self.last_recv = rospy.Time.now()
        self.current_time = rospy.Time.now()
        # Timer: Calls the timer_callback function at frequency in Hz
        # The vodom will be published at 10Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        # Publisher
        self.vodom_pub = rospy.Publisher("vodom", Odometry, queue_size=10)
        self.vodom_failure_pub = rospy.Publisher("vodom_failure", Bool, queue_size=10)

        # Initialise parameters and data
        self.x, self.y, self.psi = 0.0, 0.0, 0.0
        self.data_path = VisionBasedLocalisation.get_save_path()
        self.markers = VisionBasedLocalisation.read_marker_positions(self.data_path + "markers.csv")

    def marker_callback(self, data:FiducialMarkerArray):
        # This callback func is triggered only if a marker is detected
        id_num = data.num_markers

        # Initialise the pose lists
        x_list = []
        y_list = []
        psi_list = []

        # Iterate through markers
        for marker in data.markers:
            id = marker.id
            distance_vec = marker.tvec  # T
            rotation_vec = marker.rvec  # R

            if id == 26 or id > 28:
                continue

            # Transformations
            R, _ = cv2.Rodrigues(rotation_vec)
            T = np.array(distance_vec)
            euler_angles = euler_from_matrix(R,'sxyz')

            # Update the pose info
            x, y, psi = self.get_position(T, id, euler_angles[1])
            x_list.append(x)
            y_list.append(y)
            psi_list.append(psi)

            if self.verbosity == 1:
                rospy.loginfo("-"*25 + "Vision Based Localisation" + "-"*25 +
                            f"\n marker id: {id}" + 
                            f"\n x: {x:3.5f}, y: {y:3.5f}, psi: {psi/np.pi:3.5f}pi")

        # If either is not empty
        if x_list and y_list and psi_list:
            self.x = np.mean(x_list)
            self.y = np.mean(y_list)
            self.psi = np.mean(psi_list)

        # Track the last updated time
        self.last_recv = rospy.Time.now()

    def get_position(self, T, marker_id, theta):
        # Get marker position
        marker_x = self.markers[marker_id][X]
        marker_y = self.markers[marker_id][Y]
        marker_Psi = self.markers[marker_id][PSI]
        marker_Psi = np.deg2rad(marker_Psi)  # Since the marker info is in deg

        marker_x_wrt_c = T[2]
        marker_y_wrt_c = -T[0]
        
        x = marker_x - marker_x_wrt_c
        y = marker_y - marker_y_wrt_c
        psi = np.pi - theta + marker_Psi
        psi = self.wrap_angle(psi)
        
        return x, y, psi

    def timer_callback(self, event):
        # Get current time
        self.current_time = rospy.Time.now()

        # Publish odometry
        self.publish_odom()
        
        # Check if the last updated time is within timedout duration
        if self.current_time - self.last_recv > rospy.Duration(TIMED_OUT):
            rospy.logwarn("No marker detected!")
            self.vodom_failure_pub.publish(True)
        else:
            self.vodom_failure_pub.publish(False)

    def publish_odom(self):
        # Create the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.current_time
        odom_msg.header.frame_id = "vodom"
        odom_msg.child_frame_id = "base_link"

        # Set the position in the odometry message
        odom_msg.pose.pose.position = Point(self.x, self.y, 0)
        odom_quat = quaternion_from_euler(0, 0, self.psi)
        odom_msg.pose.pose.orientation = Quaternion(*odom_quat)

        # Publish the odometry message
        self.vodom_pub.publish(odom_msg)


    ##############################################################################################################
    ############################## Untilities ####################################################################
    ##############################################################################################################

    @staticmethod
    def read_marker_positions(filename):
        marker_positions = np.loadtxt(filename, delimiter=",", skiprows=1)
        return marker_positions

    @staticmethod
    def get_save_path():
        rospack = rospkg.RosPack()
        return rospack.get_path("localisation") + '/src/data/'

    @staticmethod
    def wrap_angle(angle):
        # Angle wrapping
        return (angle + np.pi) % (2 * np.pi) - np.pi

if __name__ == '__main__':
    vbl = VisionBasedLocalisation()
    rospy.spin()
