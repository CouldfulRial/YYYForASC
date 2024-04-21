#! /usr/bin/env python

'''
This node gives the estimation of the robot's position based on the wheel odometry.
Subscribed topics:
    /asc/current_motor_duty_cycle  ==> for wheel direction
    /asc/encoder_counts            ==> for encoder counts
Published topics:
    /odom                          ==> for the odometry
'''
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler

# Subscriber messages
from asclinic_pkg.msg import LeftRightFloat32, LeftRightInt32

# Constants
WHEEL_DIAMETER = 0.144  # m
WHEEL_RADIUS = WHEEL_DIAMETER / 2
WHEEL_BASE = 0.218  # m
HALF_WHEEL_BASE = WHEEL_BASE / 2
CPR = 1120
RADIAN_PER_COUNT = 2 * pi / CPR


class WheelOdom:
    def __init__(self):
        rospy.init_node('wheel_odometry', anonymous=True)

        # Initialise the subscribers
        self.sub_encoder = rospy.Subscriber('/asc/encoder_counts', LeftRightInt32, self.update_encoder_callback)
        self.sub_dir = rospy.Subscriber('/asc/current_motor_duty_cycle', LeftRightFloat32, self.update_dir_callback)

        # Initialise the publisher and tf broadcaster
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)  # idk why 50 here, but every single example uses 50
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        # Initialise the inertial pose
        self.x, self.y, self.psi = 0.0, 0.0, 0.0  # note that psi is in radian

        # Initialise the directions
        self.left_dir, self.right_dir = 1, 1  # 1 for forward, -1 for backward

        # Variables to manage time stamps
        self.current_time = rospy.Time.now()
        self.last_logged_time = rospy.Time.now()

    def update_encoder_callback(self, data):
        # Update the encoder counts to delta theta for both wheels in radian
        delta_theta_l = self.left_dir * data.left * RADIAN_PER_COUNT
        delta_theta_r = self.right_dir * data.right * RADIAN_PER_COUNT

        # Map to:
        #   forward movement in the body frame (delta_s) (drive: v)
        #   rotation of the robot (delta_psi)            (steer: omega)
        self.v     = WHEEL_RADIUS * (delta_theta_l + delta_theta_r) / 2
        self.omega = WHEEL_RADIUS * (delta_theta_r - delta_theta_l) / WHEEL_BASE

        # Map to movement in the inertial frame
        delta_x_p = self.v * cos(self.psi)
        delta_y_p = self.v * sin(self.psi)
        delta_psi = self.omega

        # Update the inertial pose
        self.x += delta_x_p
        self.y += delta_y_p
        self.psi += delta_psi
        self.psi %= 2 * pi  # keep psi within 0, 2pi

        # Get current time
        self.current_time = rospy.Time.now()

        # Publish the odometry and tf
        self.publish_odom_tf()
        self.publish_odom()

        # Logging at a slower rate
        if self.current_time - self.last_logged_time >= 2.0:  # Check if 1 second has passed
            #Display in the console
            # rospy.loginfo(f"Seq_Num: {data.seq_num:05d}, left count: {data.left}, right count: {data.right}")
            # rospy.loginfo(f"Seq_Num: {data.seq_num:05d}, delta_theta_l: {delta_theta_l:.2f}, delta_theta_r: {delta_theta_r:.2f}")
            # rospy.loginfo(f"Seq_Num: {data.seq_num:05d}, v: {v:.2f}, omega: {omega:.2f}")
            # rospy.loginfo(f"Seq_Num: {data.seq_num:05d}, delta_x_p: {delta_x_p:.2f}, delta_y_p: {delta_y_p:.2f}")
            rospy.loginfo(f"Seq_Num: {data.seq_num:05d}, x: {self.x:.2f}, y: {self.y:.2f}, psi: {self.psi:.2f}")

            self.last_logged_time = self.current_time



    def update_dir_callback(self, data):
        # Update the directions
        self.left_dir = 1 if data.left >= 0 else -1
        self.right_dir = 1 if data.right >= 0 else -1

        left_dir = "forward" if self.left_dir == 1 else "backward"
        right_dir = "forward" if self.right_dir == 1 else "backward"

        # Display in the console
        rospy.loginfo(f"Seq_Num: {data.seq_num:05d}, left: {left_dir}, right: {right_dir}")

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

        # Set the velocity in the odometry message
        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.angular.z = self.omega

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

    def publish_odom_tf(self):
        # Create the transform broadcaster
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            quaternion_from_euler(0, 0, self.psi),
            self.current_time,
            "base_link",
            "odom"
        )
        


if __name__ == '__main__':
    try:
        wheel_odometry = WheelOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass