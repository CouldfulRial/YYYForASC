#! /usr/bin/env python

'''
This node gives the estimation of the robot's position based on the wheel odometry.
Subscribed topics:
    mes_speeds [asclinic_pkg/LeftRightFloat32]
Published topics:
    wodom      [nav_msgs/Odometry]
'''

import rospy
import tf
from math import sin, cos, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import quaternion_from_euler
from asclinic_pkg.msg import LeftRightFloat32, LeftRightInt32

# Constants
WHEEL_DIAMETER = 0.144  # m
WHEEL_RADIUS = WHEEL_DIAMETER / 2
WHEEL_BASE = 0.218  # m
HALF_WHEEL_BASE = WHEEL_BASE / 2
TIME_STEP = 0.1

class WheelOdom:
    def __init__(self):
        # Initialise node
        self.node_name = 'wheel_odometry'
        rospy.init_node(self.node_name, anonymous=True)

        # Get user parameter
        self.parm  = rospy.get_param(self.node_name)
        self.verbosity = self.parm["verbosity"]

        # Initialise the subscribers
        self.sub_speeds = rospy.Subscriber('mes_speeds', LeftRightFloat32, self.mes_speeds_callback)

        # Initialise the publisher and tf broadcaster
        self.odom_pub = rospy.Publisher("wodom", Odometry, queue_size=50)  # idk why 50 here, but every single example uses 50
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        # Initialise the inertial pose
        self.x, self.y, self.psi = 0.0, 0.0, 0.0  # note that psi is in radian

    def mes_speeds_callback(self, data):
        # Update both the speeds
        self.delta_theta_l = data.left * TIME_STEP * 1
        self.delta_theta_r = data.right * TIME_STEP

        # Map to v and omega
        self.v     = WHEEL_RADIUS * (self.delta_theta_l + self.delta_theta_r) / 2  # m/s
        self.omega = WHEEL_RADIUS * (self.delta_theta_r - self.delta_theta_l) / WHEEL_BASE  # rad/s

        # Map to movement in the inertial frame
        self.delta_x_p = self.v * cos(self.psi)
        self.delta_y_p = self.v * sin(self.psi)
        self.delta_psi = self.omega

        # Update the inertial pose
        self.x   += self.delta_x_p
        self.y   += self.delta_y_p
        self.psi += self.delta_psi

        # Get current time
        self.current_time = rospy.Time.now()

        # Publish the odometry and tf
        self.publish_odom_tf()
        self.publish_odom()

    def publish_odom(self):
        # Create the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.current_time
        odom_msg.header.frame_id = "wodom"
        odom_msg.child_frame_id = "base_link"

        # Set the position in the odometry message
        odom_msg.pose.pose.position = Point(self.x, self.y, 0)
        odom_quat = quaternion_from_euler(0, 0, self.psi)
        odom_msg.pose.pose.orientation = Quaternion(*odom_quat)  # * is used to unpack the tuple

        # Set the velocity in the odometry message
        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.angular.z = self.omega

        # Log
        if self.verbosity == 1:
            rospy.loginfo("-"*25 + "Wheel Odometry" + "-"*25 + 
                          f"\ndelta_theta_l: {self.delta_theta_l:3.2f}, delta_theta_r: {self.delta_theta_r:3.2f}" + 
                          f"\nv:             {self.v:3.2f}, omega: {self.omega:3.2f}" + 
                          f"\ndelta_x_p:     {self.delta_x_p:3.2f}, delta_y_p: {self.delta_y_p:3.2f}, delta_psi: {self.delta_psi:3.2f}" + 
                          f"\nself.x:        {self.x:3.2f}, self.y: {self.y:3.2f}, self.psi: {self.psi:3.2f}")

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

    def publish_odom_tf(self):
        # Create the transform broadcaster
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            quaternion_from_euler(0, 0, self.psi),
            self.current_time,
            "base_link",
            "wodom"
        )

    @staticmethod
    def quat_to_euler(quat):
        if quat is not None:
            euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            return euler[2]  # return the yaw, which is the orientation around z-axis
        else:
            return 0  # If intialised to None, return 0
        
if __name__ == '__main__':
    wheel_odometry = WheelOdom()
    rospy.spin()
