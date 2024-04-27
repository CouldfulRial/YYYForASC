
#! /usr/bin/env python
##'''
##This node provides the location esitmaion of the robot based on the camera

##Subscribed topics:

##Published topics:

##'''
import rospy
from std_msgs.msg import String
from asclinic_pkg.msg import FiducialMarkerArray
import numpy as np
from math import sqrt
import cv2
class VisionBasedLocalisation:
    def __init__(self):
         # Initialise node
        self.node_name = 'vision_based_localisation'
        rospy.init_node(self.node_name)
        # Subscribers
        # rospy.Subscriber(topic_name, msg_type, callback_function)
        self.sub = rospy.Subscriber('/asc/aruco_detections', FiducialMarkerArray, self.ref_twist_callback)
       
        # Timer: Calls the timer_callback function at frequency in Hz
        # rospy.Timer(rospy.Duration(1/frequency), callback_function)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)

        # Publisher
        # rospy.Publisher(topic_name, msg_type, queue_size)

    def ref_twist_callback(self, data):
        id_num = data.num_markers
        n = 0
        results_1 = 0
        ini=np.zeros((3,50))
        ini[0][19]=-0.6
        ini[1][19]=-0.3
        ini[2][19]=1.5
        ini[2][28]=1.5
        ini[1][28]=-0.3
        ini[0][28]=0
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
            results_1=(results_1*n+results)/(n+1)
            n=n+1
        print(results_1,n)
        return results_1,n
        print(ini)
    def timer_callback(self, event):
        pass

if __name__ == '__main__':
    vbl = VisionBasedLocalisation()
    rospy.spin()
