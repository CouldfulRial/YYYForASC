#! /usr/bin/env python
'''
This node fuses the localisation data from odometry and vision odometry using a Kalman filter.

Subscribed topics:
    wodom      [nav_msgs/Odometry]
    vodom      [nav_msgs/Odometry]
Published topics:
    odom      [nav_msgs/Odometry]
'''
import rospy
# import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler
# from filterpy.kalman import KalmanFilter

# class RobotLocalisation:
#     def __init__(self):
#         rospy.init_node('robot_localisation')

#         # Kalman filter setup
#         self.kf = KalmanFilter(dim_x=2, dim_z=2)  # Assuming a 2D position state (x, y)
#         self.kf.F = np.eye(2)  # State transition matrix
#         self.kf.H = np.eye(2)  # Measurement matrix
#         self.kf.x = np.zeros(2)  # Initial state
#         self.kf.P *= 1000.  # Initial covariance
#         self.kf.Q = np.eye(2) * 0.1  # Process noise
#         self.R_odom = np.eye(2) * 0.6  # Measurement noise for odometry (40% confidence)
#         self.R_vodom = np.eye(2) * 0.4  # Measurement noise for vision odometry (60% confidence)
#         self.kf.R = self.R_odom  # Start with odometry measurement noise

#         # Tracking camera operational status
#         self.camera_operational = True

#         # Subscribers
#         self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
#         self.vodom_sub = rospy.Subscriber('vodom', Odometry, self.vodom_callback)

#     def odom_callback(self, msg):
#         current_odom = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
#         self.kf.predict()  # Predict the next state based on the previous state
#         self.kf.R = self.R_odom  # Use odometry covariance
#         self.kf.update(current_odom)  # Update the state with odometry data
#         self.publish_state()

#     def vodom_callback(self, msg):
#         if msg.header.frame_id:  # Check if the vision system is operational
#             abs_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
#             self.kf.predict()
#             self.kf.R = self.R_vodom  # Use vision odometry covariance
#             self.kf.update(abs_position)
#             self.camera_operational = True
#         else:
#             self.camera_operational = False
#             rospy.logwarn("Vision system failure detected. Relying on odometry.")
#         self.publish_state()

#     def publish_state(self):
#         # Here, you would publish the fused state to a ROS topic
#         # For simplicity, we're just logging it
#         rospy.loginfo(f"Fused position: {self.kf.x}")

#     def run(self):
#         rospy.spin()

class Localisation:
    def __init__(self):
        # Initialise node
        self.node_name = 'localisation'
        rospy.init_node(self.node_name)

        # Get user parameter
        self.parm  = rospy.get_param(self.node_name)
        self.verbosity = self.parm["verbosity"]
        
        # Subscribers
        # rospy.Subscriber(topic_name, msg_type, callback_function)
        self.sub = rospy.Subscriber('wodom', Odometry, self.wodom_callback)
        self.sub = rospy.Subscriber('vodom', Odometry, self.vodom_callback)

        # Timer: Calls the timer_callback function at frequency in Hz
        # The vodom will be published at 10Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        # Publisher
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)

        # Initialise parameters
        self.x, self.y, self.psi = 0.0, 0.0, 0.0

    def wodom_callback(self, data):
        # Extract the current pose from the odometry message
        # self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.psi = data.pose.pose.orientation.z

    def vodom_callback(self, data):
        # Extract the current pose from the odometry message
        self.x = data.pose.pose.position.x
        # self.y = data.pose.pose.position.y
        # self.psi = data.pose.pose.orientation.z

    def timer_callback(self, event):
        # Logging
        if self.verbosity == 1:
            rospy.loginfo("-"*25 + "Localisation" + "-"*25 +
                            f"\nself.x: {self.x:3.2f}, self.y: {self.y:3.2f}, self.psi: {self.psi:3.2f}")
        
        # Get current time
        self.current_time = rospy.Time.now()
        
        # Publish odometry
        self.publish_odom()

    def publish_odom(self):
        # Create the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set the position in the odometry message
        odom_msg.pose.pose.position = Point(self.x, self.y, 0)
        odom_quat = quaternion_from_euler(0, 0, self.psi)
        odom_msg.pose.pose.orientation = Quaternion(*odom_quat)  # * is used to unpack the tuple

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

if __name__ == '__main__':
    localisation_node = Localisation()
    rospy.spin()
