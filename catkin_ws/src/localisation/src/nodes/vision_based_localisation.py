
#! /usr/bin/env python

'''
This node provides the location esitmaion of the robot based on the camera

Subscribed topics:
    /asc/aruco_detections  [asclinic_pkg/FiducialMarkerArray]
Published topics:
    vodom                  [nav_msgs/Odometry]
'''

import rospy
import rospkg
from asclinic_pkg.msg import FiducialMarkerArray
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler
import cv2

X = 1
Y = 2
Z = 3
THETA = 4

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

        # Timer: Calls the timer_callback function at frequency in Hz
        # The vodom will be published at 10Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        # Publisher
        self.vodom_pub = rospy.Publisher("vodom", Odometry, queue_size=10)

        # Initialise parameters and data
        self.x, self.y = 0.0, 0.0
        self.data_path = VisionBasedLocalisation.get_save_path()
        self.markers = VisionBasedLocalisation.read_marker_positions(self.data_path + "markers.csv")

    def marker_callback(self, data):
        id_num = data.num_markers
        # if id_num == 0:
        #     self.x = 0.0
        #     self.y = 0.0
        #     return
        n = 0
        results_1 = 0
        # ini=np.zeros((3,50))
        # ini[1][2]=-0.3
        # ini[2][2]=1.2
        # ini[0][2]=0.4
        # ini[1][24]=-0.3
        # ini[2][24]=1.2
        # ini[0][24]=-0.2
        # ini[1][28]=-0.3
        # ini[2][28]=1.5
        # ini[0][28]=-0.6
        # ini[1][25]=-0.3
        # ini[2][25]=1.2
        # ini[0][25]=0.6
        x_list = []
        y_list = []
        for marker in data.markers:
            distance_vec = marker.tvec  # T
            rotation_vec = marker.rvec  # R

            R, _ = cv2.Rodrigues(rotation_vec)
            T = np.array(distance_vec)
            id = marker.id

            # Discard marker 12
            if id == 12:
                continue

            # Discard the far markers
            # if T[2] > 4:
            #     continue
            
            # Qu Xian Jiu Guo
            x, y = self.get_position(T, id)
            x_list.append(x)
            y_list.append(y)

        #     rospy.loginfo("-"*25 + "Vision Based Localisation" + "-"*25 +
        #                 f"\n marker id: {id}" + 
        #                 f"\n x: {x:3.2f}, y: {y:3.2f}")
        # rospy.loginfo(
        #             f"\n xs: {x_list}" + 
        #             f"\n ys: {y_list}")

        self.x = np.mean(x_list)
        self.y = np.mean(y_list)
        # print(f"x: {self.x}, y: {self.y}")
            # Get the orientation of the robot in the inertial frame

            
            # Rmat, _ = cv2.Rodrigues(rotation_vec)
            # #Rinv=np.transpose(Rmat)
            # #camera_ini=np.array([0,0,0])
            # marker_ini=ini[:,marker.id]
            # print('this is initial of marker',marker_ini,'and id',marker.id)
            # R_Marker=np.dot(Rmat,marker_ini)
            # results=R_Marker+distance_vec
            # if results[2]>=2:
            #   continue
            # print('results is',results)
            # results_1=(results_1*n+results)/(n+1)
            # n=n+1
        # print("final result", results_1,n)

        # Map from camera frame to world frame
        # if n>0:
        #     self.x = -results_1[2]
        #     self.y = results_1[0]

    def get_position(self, T, marker_id):
        # Get marker position
        marker_x = self.markers[marker_id][X]
        marker_y = self.markers[marker_id][Y]

        marker_x_wrt_c = T[2]
        marker_y_wrt_c = -T[0]

        x = marker_x - marker_x_wrt_c
        y = marker_y - marker_y_wrt_c

        return x, y
        

    def timer_callback(self, event):
        # Logging
        # if self.verbosity ==1:
        #     rospy.loginfo("-"*25 + "Vision Based Localisation" + "-"*25 +
        #                     f"\nself.x: {self.x:3.2f}, self.y: {self.y:3.2f}")
        
        # Get current time
        self.current_time = rospy.Time.now()
        
        # Publish odometry
        self.publish_odom()

    def publish_odom(self):
        # Create the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.current_time
        odom_msg.header.frame_id = "vodom"
        odom_msg.child_frame_id = "base_link"

        # Set the position in the odometry message
        odom_msg.pose.pose.position = Point(self.x, self.y, 0)
        odom_quat = quaternion_from_euler(0, 0, 0)
        odom_msg.pose.pose.orientation = Quaternion(*odom_quat)  # * is used to unpack the tuple

        # Publish the odometry message
        self.vodom_pub.publish(odom_msg)


    ##############################################################################################################
    ############################## Untilities ####################################################################
    ##############################################################################################################

    @staticmethod
    def read_marker_positions(filename):
        marker_positions = np.loadtxt(filename, delimiter=",", skiprows=1)
        return marker_positions

    def get_T_IM(self, marker_id):
        # marker: [id, XL, YL, ZL, theta] = markers[marker_id]
        T_IM = np.zeros((4, 4))  # Initilize the transformation matrix
        T_IM[0:3, 0:3] = VisionBasedLocalisation.get_R(self.markers[marker_id][THETA])
        T_IM[0:3, 3] = self.markers[marker_id][X:Z+1]

        return T_IM

    @staticmethod
    def get_R(theta):
        R = np.zeros((3, 3))
        if theta == 0:
            R = np.array([
                [0, -1,  0],
                [0,  0, -1],
                [1,  0,  0]
            ])
        elif theta == 180:
            R = np.array([
                [0, -1,  0],
                [0,  0, -1],
                [-1, 0,  0]
            ])
        elif theta == -90:
            R = np.array([
                [-1, 0,  0],
                [0,  0, -1],
                [0, -1,  0]
            ])

        return R

    @staticmethod
    def get_T_MC(R, T):
        T_MC = np.zeros((4, 4))
        T_MC[0:3, 0:3] = np.transpose(R)
        T_MC[0:3, 3] = T #np.dot(-np.transpose(R), T)

        return T_MC

    @staticmethod
    def get_T_CR():
        R = np.array([
                [0,  0,  1],
                [-1, 0,  0],
                [0, -1,  0]
            ])
        T = np.array([0, -0.3, 0])
        T_CR = np.zeros((4, 4))
        T_CR[0:3, 0:3] = R
        T_CR[0:3, 3] = T

        return T_CR

    @staticmethod
    def get_save_path():
        rospack = rospkg.RosPack()
        return rospack.get_path("localisation") + '/src/data/'
    

if __name__ == '__main__':
    vbl = VisionBasedLocalisation()
    rospy.spin()
