#! /usr/bin/env python
'''
This node fuses the localisation data from wodom and vodom using a Kalman filter.

Subscribed topics:
    mes_speeds     [asclinic_pkg/LeftRightFloat32]
    vodom          [nav_msgs/Odometry]
    vodom_failure  [std_msgs/Bool]
Published topics:
    odom           [nav_msgs/Odometry]
'''
# Core
import rospy

# Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from asclinic_pkg.msg import LeftRightFloat32
from std_msgs.msg import Bool

# Algorithms
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
# from filterpy.kalman import KalmanFilter

X = 0
Y = 1
PSI = 2
WHEEL_DIAMETER = 0.144  # m
WHEEL_RADIUS = WHEEL_DIAMETER / 2
WHEEL_BASE = 0.218  # m
HALF_WHEEL_BASE = WHEEL_BASE / 2
TIME_STEP = 0.1
KL = 1
KR = 1

class Localisation:
    def __init__(self): 
        # Initialise node
        self.node_name = 'localisation'
        rospy.init_node(self.node_name)

        # Get user parameters
        try:
            self.parm  = rospy.get_param(self.node_name)
            self.verbosity = self.parm["verbosity"]
        except KeyError:
            self.verbosity = 1
        
        # Subscribers
        # rospy.Subscriber(topic_name, msg_type, callback_function)
        self.wsub = rospy.Subscriber('mes_speeds', LeftRightFloat32, self.wodom_callback)
        self.vsub = rospy.Subscriber('vodom', Odometry, self.vodom_callback)
        self.vfail_sub = rospy.Subscriber('vodom_failure', Bool, self.vfail_callback)

        # Publisher
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)

        # Initialise parameters
        self.Delta_s_t   = 0.0
        self.Delta_psi_t = 0.0
        self.z_t_1 = np.array([0.0, 0.0, 0.0])  # pose estimate from computer vision at time t-1
        self.z_t = np.array([0.0, 0.0, 0.0])  # pose estimate from computer vision at time t
        self.phat_t_given_t = np.array([0.0, 0.0, 0.0])  # pose estimate after fusion of odometry with CV update t
        self.phat_t_given_t_1 = np.array([0.0, 0.0, 0.0])  # pose estimate from odometry at time t given the previous CV update t-1
        self.phat_t_1_given_t_1 = np.array([0.0, 0.0, 0.0])  # pose estimate from odometry at time t-1 given the previous CV update t-1

        # Covariance matrices
        self.pose_cov_t_given_t = np.zeros((3, 3))  # Covariance of the pose estimate at time t given t
        self.pose_cov_t_given_t_1 = np.zeros((3, 3))  # Covariance of the pose estimate at time t given t-1
        self.pose_cov_t_1_given_t_1 = np.zeros((3, 3))  # Covariance of the pose estimate at time t-1 given t-1
        # Update the following covariance matrix after measurements
        self.cv_cov_t = np.eye(3) * 0.01  # Covariance of the computer vision estimate at time t

        # vodom failure flag
        self.vodom_failure = True

        # Timer: Calls the timer_callback function at frequency in Hz
        # The vodom will be published at 10Hz
        self.current_time = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def wodom_callback(self, data:LeftRightFloat32):
        # Implement lecture L07 P10
        # Extract the updated change in both wheels at t
        # Update the pose estimate at t given the previous vodom z_t-1
        delta_theta_l = data.left
        delta_theta_r = data.right

        # Map to Delta_s, Delta_psi
        self.Delta_s_t   = WHEEL_RADIUS / 2          * (delta_theta_l + delta_theta_r)
        self.Delta_psi_t = WHEEL_RADIUS / WHEEL_BASE * (delta_theta_r - delta_theta_l)
        inside = self.z_t_1[PSI] + self.Delta_psi_t / 2

        # Update the predicted pose according to f_up
        self.phat_t_given_t_1[X]   = self.phat_t_1_given_t_1[X]   + self.Delta_s_t * np.cos(inside)
        self.phat_t_given_t_1[Y]   = self.phat_t_1_given_t_1[Y]   + self.Delta_s_t * np.sin(inside)
        self.phat_t_given_t_1[PSI] = self.phat_t_1_given_t_1[PSI] + self.Delta_psi_t

        # Update the predicted covariance
        self.update_pose_cov(delta_theta_l, delta_theta_r, inside, self.Delta_s_t)

    def update_pose_cov(self, delta_theta_l, delta_theta_r, inside, Delta_s_t):
        # This function implements lecture L05 P17 and P33
        # Put the covariance of the wheel measurements into the return variable
        theta_covariance = np.zeros((2, 2))  # Initialise the theta covariance matrix
        theta_covariance[0, 0] = KL * abs(delta_theta_l)
        theta_covariance[1, 1] = KR * abs(delta_theta_r)

        # Put the covariance of the pose into the return variable
        # Linearisation w.r.t. states
        pose_jacobians = [  # 3x3
            [1, 0, -Delta_s_t * np.sin(inside)],
            [0, 1,  Delta_s_t * np.cos(inside)],
            [0, 0,  1]
        ]

        # Linearisation w.r.t. inputs
        r = WHEEL_RADIUS
        b = HALF_WHEEL_BASE
        theta_jacobians = [  # 3x2
            [ r/2 * np.cos(inside) + r/(4*b) * Delta_s_t * np.sin(inside), r/2 * np.cos(inside) - r/(4*b) * Delta_s_t * np.sin(inside)],
            [ r/2 * np.sin(inside) - r/(4*b) * Delta_s_t * np.cos(inside), r/2 * np.sin(inside) + r/(4*b) * Delta_s_t * np.cos(inside)],
            [-r / (2*b),                                                   r / (2*b)]
        ]

        # Update the pose covariance matrix (3x3)
        if not self.vodom_failure:
            # If the vision-based localisation is working
            self.pose_cov_t_given_t_1 = pose_jacobians @ self.pose_cov_t_1_given_t_1 @ np.transpose(pose_jacobians) + theta_jacobians @ theta_covariance @ np.transpose(theta_jacobians)
        else:
            # If the vision-based localisation fails, then we assume wodom is the only source of information
            self.pose_cov_t_given_t_1 = np.zeros((3, 3))

    def vodom_callback(self, data:Odometry):
        # Based on the vision-based localisation
        # Extract the current pose z_t
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        psi = self.quat_to_euler(data.pose.pose.orientation)

        # Update the current pose z_t, and z_t-1
        self.z_t_1 = self.z_t
        self.z_t = np.array([x, y, psi])

    def vfail_callback(self, data:Bool):
        if data.data:
            self.vodom_failure = True
        else:
            self.vodom_failure = False

    def timer_callback(self, event):
        # Update Kalman gain Kt
        Kt = self.pose_cov_t_given_t_1 @ np.linalg.inv(self.pose_cov_t_given_t_1 + self.cv_cov_t)

        # Update the pose estimate
        self.phat_t_1_given_t_1 = self.phat_t_given_t
        self.phat_t_given_t = self.phat_t_given_t_1 + Kt @ (self.z_t - self.phat_t_given_t_1)

        # Update the covariance matrix
        self.pose_cov_t_1_given_t_1 = self.pose_cov_t_given_t
        self.pose_cov_t_given_t = self.pose_cov_t_given_t_1 - Kt @ (self.pose_cov_t_given_t_1 + self.cv_cov_t) @ np.transpose(Kt)

        # Publish the odom at a fixed rate
        self.current_time = rospy.Time.now()
        self.publish_odom()

    def publish_odom(self):
        # Create the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Get the current pose update
        x = self.phat_t_given_t[X]
        y = self.phat_t_given_t[Y]
        psi = self.phat_t_given_t[PSI]

        if self.verbosity == 1:
            rospy.loginfo("-"*25 + "Localisation" + "-"*25 +
                          f"\n vodom fails: {self.vodom_failure}" +
                          f"\n x: {x:3.5f}, y: {y:3.5f}, psi: {psi/np.pi:3.5f}pi" + 
                          f"\n pose covariance: \n{self.pose_cov_t_given_t}")

        # Set the position in the odometry message
        odom_msg.pose.pose.position = Point(x, y, 0)
        odom_quat = quaternion_from_euler(0, 0, psi)
        odom_msg.pose.pose.orientation = Quaternion(*odom_quat)  # * is used to unpack the tuple

        # Set the velocity in the odometry message
        odom_msg.twist.twist.linear.x = self.Delta_s_t / TIME_STEP
        odom_msg.twist.twist.angular.z = self.Delta_psi_t / TIME_STEP
        
        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

    @staticmethod
    def quat_to_euler(quat):
        if quat is not None:
            euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            return euler[2]  # return the yaw, which is the orientation around z-axis
        else:
            return 0  # If intialised to None, return 0

if __name__ == '__main__':
    localisation_node = Localisation()
    rospy.spin()
