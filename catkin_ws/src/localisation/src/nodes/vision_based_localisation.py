
#! /usr/bin/env python

'''
This node provides the location esitmaion of the robot based on the camera

Subscribed topics:
    /asc/aruco_detections  [asclinic_pkg/FiducialMarkerArray]
Published topics:
    vodom                  [nav_msgs/Odometry]
'''

import rospy
from asclinic_pkg.msg import FiducialMarkerArray
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler
import cv2

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

        # Publisher
        self.vodom_pub = rospy.Publisher("vodom", Odometry, queue_size=10)

        # Initialise parameters
        self.x, self.y = 0.0, 0.0

        # Timer: Calls the timer_callback function at frequency in Hz
        # The vodom will be published at 10Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def marker_callback(self, data):
        id_num = data.num_markers
        n = 0
        results_1 = 0
        ini=np.zeros((3,50))
        ini[0][19]=0
        ini[1][19]=-0.3
        ini[2][19]=2.1
        ini[1][28]=-0.3
        ini[2][28]=1.5
        ini[0][28]=-0.6
        ini[1][25]=-0.3
        ini[2][25]=1.2
        ini[0][25]=0.6
        for marker in data.markers:
            distance_vec = marker.tvec
            rotation_vec = marker.rvec
            Rmat, _ =cv2.Rodrigues(rotation_vec)
            #Rinv=np.transpose(Rmat)
            #camera_ini=np.array([0,0,0])
            marker_ini=ini[:,marker.id]
            print('this is initial of marker',marker_ini,'and id',marker.id)
            R_Marker=np.dot(Rmat,marker_ini)
            results=R_Marker+distance_vec
            print('results is',results)
            results_1=(results_1*n+results)/(n+1)
            n=n+1
        print(results_1,n)

        # Map from camera frame to world frame
        self.x = -results_1[2]
        self.y = results_1[0]
        

    def timer_callback(self, event):
        # Logging
        if self.verbosity ==1:
            rospy.loginfo("-"*25 + "Vision Based Localisation" + "-"*25 +
                            f"\nself.x: {self.x:3.2f}, self.y: {self.y:3.2f}")
        
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

if __name__ == '__main__':
    vbl = VisionBasedLocalisation()
    rospy.spin()
