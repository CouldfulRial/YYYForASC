#! /usr/bin/env python
'''
This node 

Subscribed topics:

Published topics:

'''
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from filterpy.kalman import KalmanFilter

class RobotLocalisation:
    def __init__(self):
        rospy.init_node('robot_localisation')

        # Kalman filter setup
        self.kf = KalmanFilter(dim_x=2, dim_z=2)  # Assuming a 2D position state (x, y)
        self.kf.F = np.eye(2)  # State transition matrix
        self.kf.H = np.eye(2)  # Measurement matrix
        self.kf.x = np.zeros(2)  # Initial state
        self.kf.P *= 1000.  # Initial covariance
        self.kf.Q = np.eye(2) * 0.1  # Process noise
        self.R_odom = np.eye(2) * 0.6  # Measurement noise for odometry (40% confidence)
        self.R_vodom = np.eye(2) * 0.4  # Measurement noise for vision odometry (60% confidence)
        self.kf.R = self.R_odom  # Start with odometry measurement noise

        # Tracking camera operational status
        self.camera_operational = True

        # Subscribers
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.vodom_sub = rospy.Subscriber('vodom', Odometry, self.vodom_callback)

    def odom_callback(self, msg):
        current_odom = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.kf.predict()  # Predict the next state based on the previous state
        self.kf.R = self.R_odom  # Use odometry covariance
        self.kf.update(current_odom)  # Update the state with odometry data
        self.publish_state()

    def vodom_callback(self, msg):
        if msg.header.frame_id:  # Check if the vision system is operational
            abs_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
            self.kf.predict()
            self.kf.R = self.R_vodom  # Use vision odometry covariance
            self.kf.update(abs_position)
            self.camera_operational = True
        else:
            self.camera_operational = False
            rospy.logwarn("Vision system failure detected. Relying on odometry.")
        self.publish_state()

    def publish_state(self):
        # Here, you would publish the fused state to a ROS topic
        # For simplicity, we're just logging it
        rospy.loginfo(f"Fused position: {self.kf.x}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    localisation_node = RobotLocalisation()
    try:
        localisation_node.run()
    except rospy.ROSInterruptException:
        pass
