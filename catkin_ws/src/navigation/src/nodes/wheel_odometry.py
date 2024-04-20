#! /usr/bin/env python

'''
This node gives the estimation of the robot's position based on the wheel odometry.
Subscribed topics:
    /asc/current_motor_duty_cycle  ==> for wheel direction
    /asc/encoder_counts            ==> for encoder counts
Published topics:
    /odom                          ==> for the odometry
'''
import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# Subscriber messages
from asclinic_pkg.msg import LeftRightFloat32, LeftRightInt32

# Constants
WHEEL_DIAMETER = 0.144  # m
WHEEL_RADIUS = WHEEL_DIAMETER / 2
WHEEL_BASE = 0.218  # m
HALF_WHEEL_BASE = WHEEL_BASE / 2
CPR = 1120
RADIAN_PER_COUNT = 2 * pi / CPR

# x = 0.0
# y = 0.0
# th = 0.0

# vx = 0.1
# vy = -0.1
# vth = 0.1

# current_time = rospy.Time.now()
# last_time = rospy.Time.now()

# r = rospy.Rate(10)
# while not rospy.is_shutdown():
#     current_time = rospy.Time.now()

#     # compute odometry in a typical way given the velocities of the robot
#     dt = (current_time - last_time).to_sec()
#     delta_x = (vx * cos(th) - vy * sin(th)) * dt
#     delta_y = (vx * sin(th) + vy * cos(th)) * dt
#     delta_th = vth * dt

#     x += delta_x
#     y += delta_y
#     th += delta_th

#     # since all odometry is 6DOF we'll need a quaternion created from yaw
#     odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

#     # first, we'll publish the transform over tf
#     odom_broadcaster.sendTransform(
#         (x, y, 0.),
#         odom_quat,
#         current_time,
#         "base_link",
#         "odom"
#     )

#     # next, we'll publish the odometry message over ROS
#     odom = Odometry()
#     odom.header.stamp = current_time
#     odom.header.frame_id = "odom"

#     # set the position
#     odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

#     # set the velocity
#     odom.child_frame_id = "base_link"
#     odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

#     # publish the message
#     odom_pub.publish(odom)

#     last_time = current_time
#     r.sleep()

class WheelOdom:
    def __init__(self):
        rospy.init_node('wheel_odometry', anonymous=True)

        # Initialise the subscribers
        self.sub_encoder = rospy.Subscriber('/asc/encoder_counts', LeftRightInt32, self.update_encoder_callback)
        self.sub_dir = rospy.Subscriber('/asc/current_motor_duty_cycle', LeftRightFloat32, self.update_dir_callback)

        # Initialise the publisher and tf broadcaster
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        # Initialise the inertial pose
        self.x, self.y, self.psi = 0.0, 0.0, 0.0  # note that psi is in radian

        # Initialise the directions
        self.left_dir, self.right_dir = 1, 1  # 1 for forward, -1 for backward

        # Variables to manage logging rate
        self.last_log_time = rospy.get_time()


    def update_encoder_callback(self, data):
        # Update the encoder counts
        delta_theta_l = self.left_dir * data.left * RADIAN_PER_COUNT
        delta_theta_r = self.right_dir * data.right * RADIAN_PER_COUNT

        # Map to:
        #   forward movement in the body frame (delta_s)
        #   rotation of the robot (delta_psi)
        delta_s = WHEEL_RADIUS * (delta_theta_l + delta_theta_r) / 2
        delta_psi = WHEEL_RADIUS * (delta_theta_r - delta_theta_l) / WHEEL_BASE

        # Map to movement in the inertial frame
        delta_x_p = delta_s * cos(self.psi + delta_psi / 2)
        delta_y_p = delta_s * sin(self.psi + delta_psi / 2)

        # Update the inertial pose
        self.x += delta_x_p
        self.y += delta_y_p
        self.psi += delta_psi
        self.psi %= 2 * pi  # keep psi within 0, 2pi

        # Logging at a slower rate
        current_time = rospy.get_time()
        if current_time - self.last_log_time >= 1.0:  # Check if 1 second has passed
            # Display in the console
            rospy.loginfo(f"Seq_Num: {data.seq_num:05d}, delta_x_p: {delta_x_p:.2f}, delta_y_p: {delta_y_p:.2f}")
            rospy.loginfo(f"Seq_Num: {data.seq_num:05d}, x: {self.x:.2f}, y: {self.y:.2f}, psi: {self.psi:.2f}")

            self.last_log_time = current_time  # Reset log time


    def update_dir_callback(self, data):
        # Update the directions
        self.left_dir = 1 if data.left >= 0 else -1
        self.right_dir = 1 if data.right >= 0 else -1

        left_dir = "forward" if self.left_dir == 1 else "backward"
        right_dir = "forward" if self.right_dir == 1 else "backward"

        # Display in the console
        rospy.loginfo(f"Seq_Num: {data.seq_num:05d}, left: {left_dir}, right: {right_dir}")
        



if __name__ == '__main__':
    try:
        wheel_odometry = WheelOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass