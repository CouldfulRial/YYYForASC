#!/usr/bin/env python
'''
This node implements the finite state machine
Subscribed topics:
    odom                 [nav_msgs/Odometry]
    /asc/camera_image    [sensor_msgs/Image]
    object_location     [asclinic_pkg/LeftRightFloat32]
    tar_reached          [std_msgs/Bool]
Published topics:
    state                [std_msgs/String]
    goal                 [geometry_msgs/Point]
'''

# ROS
import rospy
import rospkg
import tf

# Messges
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from asclinic_pkg.msg import LeftRightFloat32
from geometry_msgs.msg import Point, PointStamped

# Algorithm
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

# CONSTANT
RADIUS = 1 # m

# STATES
STOP    = 0
SEARCH  = 1
CAPTURE = 2
PROCESS = 3

class FSM:
    def __init__(self):
        # Initialise node
        self.node_name = 'fsm'
        rospy.init_node(self.node_name, anonymous=True)

        # Get user parameter
        try:
            self.parm  = rospy.get_param(self.node_name)
            self.verbosity = self.parm["verbosity"]
        except KeyError:
            self.verbosity = 1

        # Initialise variables
        self.x = self.y = self.yaw = 0.0
        self.x_loc = self.y_loc = 0.0
        self.prev_state = self.state = STOP
        self.goal_idx = 0
        self.detected = False
        self.capture = False
        self.stop = True
        self.searched = False
        self.imgs = []
        self.img_captured = Image()
        self.save_path = self.get_save_path()

        # Subscribed topics
        self.odom_sub  = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.image_sub = rospy.Subscriber("/asc/camera_image", Image, self.image_callback)
        self.ooi_sub   = rospy.Subscriber('object_location', LeftRightFloat32, self.ooi_callback)
        self.tar_sub   = rospy.Subscriber('tar_reached', Bool, self.tar_callback)

        # Timer for fsm loopfsm_callback
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        # Published topics:
        self.state_pub = rospy.Publisher('state', String, queue_size=10)
        self.markers_pub = rospy.Publisher("objects", Marker, queue_size=10)  # Visualise the detected markers for RVIZ
        # self.goal_pub  = rospy.Publisher('goal', Point, queue_size=10)

    def timer_callback(self, event):
        ## FSM state transition logic
        if self.state == STOP:
            self.state = SEARCH
            self.stop = False

        elif self.state == SEARCH:
            if self.searched:
                self.state = STOP
            elif self.detected and self.capture:
                self.state = CAPTURE

        elif self.state == CAPTURE:
            # rospy.sleep(5)
            self.state = SEARCH

        elif self.state == PROCESS:
            pass


        ## FSM state action
        if self.state == STOP:
            self.state_pub.publish("STOP")

        elif self.state == SEARCH:
            self.state_pub.publish("SEARCH")

        elif self.state == CAPTURE:
            self.state_pub.publish("CAPTURE")

        ## log
        if self.verbosity == 1:
            rospy.loginfo(f"State: {self.state}, detected: {self.detected}, capture: {self.capture}, searched: {self.searched}")

        # Visualise the objects
        self.publish_detected_markers()

    def publish_detected_markers(self):
        # Create the marker message
        marker_msg = Marker()
        marker_msg.header.frame_id = "map"
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.ns = "objects"
        marker_msg.id = 1
        marker_msg.type = Marker.POINTS
        marker_msg.action = Marker.ADD
        marker_msg.pose.orientation.w = 1.0
        marker_msg.scale.x = 0.2
        marker_msg.scale.y = 0.2
        marker_msg.color.a = 1.0
        marker_msg.color.r = 0.0  # Red component
        marker_msg.color.g = 1.0  # Green component
        marker_msg.color.b = 0.0  # Blue component

        # Set the position of the markers
        for loc in self.imgs:
            x, y = loc.split(",")
            point = Point()
            point.x = float(x)
            point.y = float(y)
            point.z = 0.0
            marker_msg.points.append(point)

        # Publish the marker message
        self.markers_pub.publish(marker_msg)
        
    def tar_callback(self, data:Bool):
        if data.data:
            self.searched = True

    def odom_callback(self, data:Odometry):
        # Extract the current pose from the odometry message
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.current_theta = data.pose.pose.orientation
        self.yaw = self.quat_to_euler(self.current_theta)

    def image_callback(self, data:Image):
        if self.state != CAPTURE:
            return
        self.image_captured = data

    def ooi_callback(self, data:LeftRightFloat32):
        # If an object of interest is detected, ready to capture image
        self.x_loc = data.left
        self.y_loc = data.right
        self.detected = True
        self.captured = False
        print("img list: ", self.imgs)

       # Iterate through the image dictionary to find the closest image, and get the distance
        if self.imgs:
            for loc in self.imgs:
                x, y = loc.split(",")
                x = float(x)
                y = float(y)
                distance = np.sqrt((x - self.x_loc) ** 2 + (y - self.y_loc) ** 2)
                print("distance: ", distance)
                if distance < RADIUS:
                    # The object is within the radius of captured image
                    self.capture = False
                    return
        # Hence the location is new
        self.capture = True

        # Save
        try:
            # Convert the image from ROS to OpenCV format
            cv_image = CvBridge().imgmsg_to_cv2(self.image_captured , "bgr8")
        except (CvBridgeError, AttributeError):
            return

        string = f"{self.x_loc:.2f}, {self.y_loc:.2f}"
        self.imgs.append(string)
        filename = f"{self.save_path}{string}.png"  # Save the current image using its location
        cv2.imwrite(filename, cv_image)

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

if __name__ == '__main__':
    controller = FSM()
    rospy.spin()
