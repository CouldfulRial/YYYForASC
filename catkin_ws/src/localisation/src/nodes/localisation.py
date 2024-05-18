#! /usr/bin/env python
'''
This node fuses the localisation data from wodom and vodom using a Kalman filter.

Subscribed topics:
    mes_speeds     [asclinic_pkg/LeftRightFloat32]
    vodom          [nav_msgs/Odometry]
    vodom_failure  [std_msgs/Bool]  (ARCHIVED)
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
import tf

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
        self.wsub = rospy.Subscriber('mes_speeds', LeftRightFloat32, self.wodom_callback)  # 10Hz
        # self.vsub = rospy.Subscriber('vodom', Odometry, self.vodom_callback)               # 5 Hz
        # self.vfail_sub = rospy.Subscriber('vodom_failure', Bool, self.vfail_callback)

        # Publisher
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        # Initialise parameters
        self.delta_theta_l, self.delta_theta_r = 0.0, 0.0
        self.Delta_s_t, self.Delta_psi_t = 0.0, 0.0
        self.x = self.y = self.psi = 0.0
        self.z_t = np.array([0.0, 0.0, 0.0])  # pose estimate from computer vision at time t
        self.phat_t_given_t = np.array([0.0, 0.0, 0.0])  # pose estimate after fusion of odometry with CV update t
        self.phat_t_given_t_1 = np.array([0.0, 0.0, 0.0])  # pose estimate from odometry at time t given the previous CV update t-1
        self.phat_t_1_given_t_1 = np.array([0.0, 0.0, 0.0])  # pose estimate from odometry at time t-1 given the previous CV update t-1
        self.Kt = np.zeros((3, 3))  # Kalman gain

        # Covariance matrices
        self.pose_cov_t_given_t = np.zeros((3, 3))  # Covariance of the pose estimate at time t given t
        self.pose_cov_t_given_t_1 = np.zeros((3, 3))  # Covariance of the pose estimate at time t given t-1
        self.pose_cov_t_1_given_t_1 = np.eye(3) * 0.01 # Covariance of the pose estimate at time t-1 given t-1
        # Update the following covariance matrix after measurements
        self.cv_cov_t = np.eye(3) * 10**-6  # Covariance of the computer vision estimate at time t

        # vodom failure flag
        self.vodom_failure = False

        # Timer: Calls the timer_callback function at frequency in Hz
        # The vodom will be published at 10Hz
        self.current_time = rospy.Time.now()
        self.last_wodom = self.current_time.to_sec()
        self.last_vodom = self.current_time.to_sec()
        self.timer = rospy.Timer(rospy.Duration(0.5), self.step_forward)

    def wodom_callback(self, data:LeftRightFloat32):  #10Hz
        self.last_wodom = rospy.Time.now().to_sec()
        # Implement lecture L07 P10
        # Extract the updated change in both wheels for the past 0.1s
        # Update the pose estimate at t given the previous vodom z_t-1
        self.delta_theta_l = data.left
        self.delta_theta_r = data.right

        self.Delta_s_t   = WHEEL_RADIUS / 2          * (self.delta_theta_l + self.delta_theta_r)
        self.Delta_psi_t = WHEEL_RADIUS / WHEEL_BASE * (self.delta_theta_r - self.delta_theta_l)

        ## Prediction Step
        # Based on the previous fused pose, update the pose given the wheel measurements
        inside = self.phat_t_1_given_t_1[PSI] + self.Delta_psi_t / 2
        self.phat_t_given_t_1[X]   = self.phat_t_1_given_t_1[X]   + self.Delta_s_t * np.cos(inside)
        self.phat_t_given_t_1[Y]   = self.phat_t_1_given_t_1[Y]   + self.Delta_s_t * np.sin(inside)
        self.phat_t_given_t_1[PSI] = self.phat_t_1_given_t_1[PSI] + self.Delta_psi_t

        # Update the pose covariance matrix
        self.update_pose_cov(self.delta_theta_l, self.delta_theta_r, inside, self.Delta_s_t)

        # Step forward
        self.phat_t_1_given_t_1 = self.phat_t_given_t_1
        self.pose_cov_t_1_given_t_1 = self.pose_cov_t_given_t_1

        self.phat_t_given_t = self.phat_t_given_t_1  # Temp

    def vodom_callback(self, data:Odometry):  # 5Hz
        self.last_vodom = rospy.Time.now().to_sec()
        # Based on the vision-based localisation, update the current pose z_t
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        psi = self.quat_to_euler(data.pose.pose.orientation)
        self.z_t = np.array([x, y, psi])

        ## Update Step
        # Update Kalman gain Kt
        self.Kt = self.pose_cov_t_given_t_1 @ np.linalg.inv(self.pose_cov_t_given_t_1 + self.cv_cov_t)

        # Update the pose estimate
        self.phat_t_given_t = self.phat_t_given_t_1 + self.Kt @ (self.z_t - self.phat_t_given_t_1)

        # Update the covariance matrix
        self.pose_cov_t_given_t = self.pose_cov_t_given_t_1 - self.Kt @ (self.pose_cov_t_given_t_1 + self.cv_cov_t) @ np.transpose(self.Kt)

        # Step forward
        self.phat_t_1_given_t_1 = self.phat_t_given_t
        self.pose_cov_t_1_given_t_1 = self.pose_cov_t_given_t

    def step_forward(self, event):
        # Publish the odom
        self.current_time = rospy.Time.now()
        self.publish_odom()

    def publish_odom(self):
        # Create the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Get the current pose update
        self.x = self.phat_t_given_t[X]
        self.y = self.phat_t_given_t[Y]
        self.psi = self.phat_t_given_t[PSI]

        if self.verbosity == 1:
            rospy.loginfo("-"*20 + "Localisation" + "-"*20 +
                        #  f"\n camera fails: {self.vodom_failure}" +
                        f"\n prev pose (self.phat_t_1_given_t_1): \n{self.phat_t_1_given_t_1}" +
                        f"\n\n##PREDICTION: {self.last_wodom:.2f}s" + 
                        f"\n delta_theta_left: {self.delta_theta_l:3.5f}, delta_theta_right: {self.delta_theta_r:3.5f}" + 
                        f"\n wodom predicted pose (self.phat_t_given_t_1): \n{self.phat_t_given_t_1}" +
                        f"\n predicted pose covariance (self.pose_cov_t_given_t_1): \n{self.pose_cov_t_given_t_1}" +
                        f"\n\n##FUSION: {self.last_vodom:.2f}s" + 
                        f"\n vbl (z_t): \n{self.z_t}" +
                        f"\n kalman gain (self.Kt): \n{self.Kt}" +
                        f"\n updated pose (self.phat_t_given_t): \n{self.phat_t_given_t}" +
                        f"\n pose covariance (self.pose_cov_t_given_t): \n{self.pose_cov_t_given_t}" + 
                        f"\n\n##RESULT: {self.current_time.to_sec():.2f}s" + 
                        f"\n x: {self.x:3.5f}, y: {self.y:3.5f}, psi: {self.psi/np.pi:3.5f}pi")
                        

        # Set the position in the odometry message
        odom_msg.pose.pose.position = Point(self.x, self.y, 0)
        odom_quat = quaternion_from_euler(0, 0, self.psi)
        odom_msg.pose.pose.orientation = Quaternion(*odom_quat)  # * is used to unpack the tuple

        # Set the velocity in the odometry message
        odom_msg.twist.twist.linear.x = self.Delta_s_t / TIME_STEP
        odom_msg.twist.twist.angular.z = self.Delta_psi_t / TIME_STEP
        
        # Publish the odometry message and tf
        self.odom_pub.publish(odom_msg)
        self.publish_odom_tf()

    ##############################################################################################################
    ############################## Untilities ####################################################################
    ##############################################################################################################

    def publish_odom_tf(self):
        # Create the transform broadcaster
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0),
            quaternion_from_euler(0, 0, self.psi),
            self.current_time,
            "base_link",
            "odom"
        )

    def vfail_callback(self, data:Bool):
        if data.data:
            self.vodom_failure = True
        else:
            self.vodom_failure = False

    def update_pose_cov(self, delta_theta_l, delta_theta_r, inside, Delta_s_t):
        # This function implements lecture L05 P17 and P33
        # Put the covariance of the wheel measurements into the return variable
        theta_covariance = np.zeros((2, 2))  # Initialise the theta covariance matrix
        theta_covariance[0, 0] = KL * abs(delta_theta_l)
        theta_covariance[1, 1] = KR * abs(delta_theta_r)

        # Put the covariance of the pose into the return variable
        # Linearisation w.r.t. states
        pose_jacobians = np.array([  # 3x3
            [1, 0, -Delta_s_t * np.sin(inside)],
            [0, 1,  Delta_s_t * np.cos(inside)],
            [0, 0,  1]
        ])

        # Linearisation w.r.t. inputs
        r = WHEEL_RADIUS
        b = HALF_WHEEL_BASE
        theta_jacobians = np.array([  # 3x2
            [ r/2 * np.cos(inside) + r/(4*b) * Delta_s_t * np.sin(inside), r/2 * np.cos(inside) - r/(4*b) * Delta_s_t * np.sin(inside)],
            [ r/2 * np.sin(inside) - r/(4*b) * Delta_s_t * np.cos(inside), r/2 * np.sin(inside) + r/(4*b) * Delta_s_t * np.cos(inside)],
            [-r / (2*b),                                                   r / (2*b)]
        ])

        # Update the pose covariance matrix (3x3)
        self.pose_cov_t_given_t_1 = pose_jacobians  @ self.pose_cov_t_1_given_t_1 @ np.transpose(pose_jacobians) + \
                                    theta_jacobians @ theta_covariance            @ np.transpose(theta_jacobians)


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
