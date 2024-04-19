#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This file is part of ASClinic-System.
#    
# See the root of the repository for license details.
#
# ----------------------------------------------------------------------------
#     _    ____   ____ _ _       _          ____            _                 
#    / \  / ___| / ___| (_)____ (_) ___    / ___| _   _ ___| |_ ___ ________  
#   / _ \ \___ \| |   | | |  _ \| |/ __|___\___ \| | | / __| __/ _ \  _   _ \ 
#  / ___ \ ___) | |___| | | | | | | (_|_____|__) | |_| \__ \ ||  __/ | | | | |
# /_/   \_\____/ \____|_|_|_| |_|_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
#                                                 |___/                       
#
# DESCRIPTION:
# Python node as a skeleton from implementing a control policy.
# This node is provided to exemplify bringing together the
# encoder counts and ArUco detection measurements into one node,
# and using those measurement to set a motor duty cycle action.
#
# ----------------------------------------------------------------------------



# Import the ROS-Python package
import rospy

# Import the standard message types
#from std_msgs.msg import UInt32

# Import numpy
import numpy as np

# Import package-specific messages
from asclinic_pkg.msg import LeftRightFloat32
from asclinic_pkg.msg import LeftRightInt32
from asclinic_pkg.msg import FiducialMarkerArray


# DEFINE THE PARAMETERS
# > For the verbosity level of displaying info
DEFAULT_CONTROL_POLICY_VERBOSITY = 1
# Note: the levels of increasing verbosity are defined as:
# 0 : Info is not displayed. Warnings and errors are still displayed.
# 1 : Startup info is displayed.
# 2 : Info each control action and state estimate update is displayed.

# > For the wheel base of the robot [in meters]
DEFAULT_ROBOT_WHEEL_BASE = 0.22

# > For the wheel radius of the robot [in meters]
DEFAULT_ROBOT_WHEEL_RADIUS = 0.072

# > For the number of encoder counts per revolution of a wheel
DEFAULT_ENCODER_COUNTS_PER_WHEEL_REVOLUTION = 1680



class ControlPolicySkeleton:

    def __init__(self):

        # GET THE PARAMETERS VALUES:
        # > For the verbosity level of displaying info
        if (rospy.has_param(node_namespace + node_name + "/" + "control_policy_verbosity")):
            self.control_policy_verbosity = rospy.get_param(node_namespace + node_name + "/" + "control_policy_verbosity")
        else:
            rospy.logwarn("[CONTROL POLICY SKELETON] FAILED to get \"control_policy_verbosity\" parameter. Using default value instead.")
            self.control_policy_verbosity = DEFAULT_CONTROL_POLICY_VERBOSITY

        # > For the wheel base dimension of the robot
        if (rospy.has_param(node_namespace + node_name + "/" + "robot_wheel_base")):
            self.robot_wheel_base = rospy.get_param(node_namespace + node_name + "/" + "robot_wheel_base")
        else:
            rospy.logwarn("[CONTROL POLICY SKELETON] FAILED to get \"robot_wheel_base\" parameter. Using default value instead.")
            self.robot_wheel_base = DEFAULT_ROBOT_WHEEL_BASE

        # > For the wheel radius dimension of the robot
        if (rospy.has_param(node_namespace + node_name + "/" + "robot_wheel_radius")):
            self.robot_wheel_radius = rospy.get_param(node_namespace + node_name + "/" + "robot_wheel_radius")
        else:
            rospy.logwarn("[CONTROL POLICY SKELETON] FAILED to get \"robot_wheel_radius\" parameter. Using default value instead.")
            self.robot_wheel_radius = DEFAULT_ROBOT_WHEEL_RADIUS

        # > For the number of encoder counts per revolution of a wheel
        if (rospy.has_param(node_namespace + node_name + "/" + "encoder_counts_per_wheel_revolution")):
            self.encoder_counts_per_wheel_revolution = rospy.get_param(node_namespace + node_name + "/" + "encoder_counts_per_wheel_revolution")
        else:
            rospy.logwarn("[CONTROL POLICY SKELETON] FAILED to get \"encoder_counts_per_wheel_revolution\" parameter. Using default value instead.")
            self.encoder_counts_per_wheel_revolution = DEFAULT_ENCODER_COUNTS_PER_WHEEL_REVOLUTION



        # PUBLISHERS AND SUBSCRIBERS:
        # > Initialise a publisher for the motor duty cycle requests
        self.motor_duty_cycle_request_publisher = rospy.Publisher(node_namespace+"set_motor_duty_cycle", LeftRightFloat32, queue_size=1)

        # > Initialise a subscriber for the current motor duty cycle
        rospy.Subscriber(node_namespace+"current_motor_duty_cycle", LeftRightFloat32, self.currentMotorDutyCycleSubscriberCallback, queue_size=1)

        # > Initialise a subscriber for the encoder counts sensor measurements
        rospy.Subscriber(node_namespace+"encoder_counts", LeftRightInt32, self.encoderCountsSubscriberCallback, queue_size=10)

        # > Initialise a subscriber for the ArUco marker vision-detection measurements
        rospy.Subscriber(node_namespace +"aruco_detections", FiducialMarkerArray, self.arucoDetectionsSubscriberCallback, queue_size=10)



        # CLASS VARIABLES:
        # > For world frame state estimate [xW, yW, phiW2R]
        self.xW_estimate = 0.0
        self.yW_estimate = 0.0
        self.phiW2R_estimate = 0.0

        # > Compute the rotation of the wheel per encoder count
        self.wheel_rotation_per_count = (2.0 * np.pi / self.encoder_counts_per_wheel_revolution) * self.robot_wheel_radius

        # > Initialize a sequence number for the motor duty cycle request actions
        self.motor_action_sequence_number = 1

        # > For the current direction of motor drive
        self.motor_left_drive_direction  = 0
        self.motor_right_drive_direction = 0



        # Display the status
        if (self.control_policy_verbosity >= 1):
            rospy.loginfo("[CONTROL POLICY SKELETON] Node initialisation complete." )



    # Function that computes and publishes motor duty-cycle request actions
    # baesd on the most recent state estimate
    def execute_control_policy(self):

        # NOTE: this skeleton does not provide any meaningful
        #       control policy implementation.

        # Prepare a message to send the motor duty-cycle request action
        msg_for_motors = LeftRightFloat32()
        msg_for_motors.left    = 0.0
        msg_for_motors.right   = 0.0
        msg_for_motors.seq_num = self.motor_action_sequence_number

        # Publish the message
        self.motor_duty_cycle_request_publisher.publish(msg_for_motors)

        # Increment the sequence number
        self.motor_action_sequence_number = self.motor_action_sequence_number + 1

    # Current motor duty cycle subscriber callback
    def currentMotorDutyCycleSubscriberCallback(self, msg):

        # Display the data received
        if (self.control_policy_verbosity >= 2):
            rospy.loginfo("[CONTROL POLICY SKELETON] Received current motor duty cycle (left,right,seq_num) = ( " + "{:6.1}".format(msg.left) + " , " + "{:6.1}".format(msg.right) + " , " + msg.seq_num + " )" )

        # Update the member variables to match the direction
        # of the current motor duty cycle.
        # > For the left wheel
        if (msg.left < -0.1):
            self.motor_left_drive_direction = -1.0
        elif(msg.left > 0.1):
            self.motor_left_drive_direction = 1.0
        else:
            self.motor_left_drive_direction = 0.0
        # > For the right wheel
        if (msg.right < -0.1):
            self.motor_right_drive_direction = -1.0
        elif(msg.right > 0.1):
            self.motor_right_drive_direction = 1.0
        else:
            self.motor_right_drive_direction = 0.0


    # Encoder counts subscriber callback
    # NOTE: this skeleton lets the receiving of the encoder counts
    #       messages determine the frequency at which the control
    #       policy runs.
    def encoderCountsSubscriberCallback(self, msg):

        # Display the data received
        if (self.control_policy_verbosity >= 2):
            rospy.loginfo("[CONTROL POLICY SKELETON] Received encoder counts (left,right,seq_num) = ( " + "{:6}".format(msg.left) + " , " + "{:6}".format(msg.right) + " , " + msg.seq_num + " )" )

        # Compute the angular change of the wheels
        delta_theta_left  = msg.left  * self.motor_left_drive_direction  * self.wheel_rotation_per_count
        delta_theta_right = msg.right * self.motor_right_drive_direction * self.wheel_rotation_per_count

        # Compute the change in displacement (delta s) and change in rotation (delta phi) resulting from the wheel rotations
        delta_s   = (delta_theta_right + delta_theta_left) * 0.5 * self.robot_wheel_radius
        delta_phi = (delta_theta_right - delta_theta_left) * 0.5 * self.robot_wheel_radius / self.robot_wheel_base

        # Get the current state estimate in local variables to avoid errors/misinterpretation related to the sequence of updates
        xW_current = self.xW_estimate
        yW_current = self.yW_estimate
        phiW2R_current = self.phiW2R_estimate

        # Compute the sine and cosine of the "halfway" angle for the wheel odometry mean and covariance formulas
        sin_phi_plus_half = np.sin(phiW2R_current + 0.5 * delta_phi)
        cos_phi_plus_half = np.cos(phiW2R_current + 0.5 * delta_phi)

        # Update the mean of the state estimate
        self.xW_estimate = xW_current + delta_s * cos_phi_plus_half
        self.yW_estimate = yW_current + delta_s * sin_phi_plus_half
        self.phiW2R_estimate = phiW2R_current + delta_phi

        # Update the covariance of the state estimate
        # > NOTE: this skeleton does not include co-variance

        # Call the function to execute the control policy
        self.execute_control_policy()



    # ArUco Detections subscriber callback
    # Details about the ArUco detection data:
    # > The properties "rvec" and "tvec" respectively
    #   describe the rotation and translation of the
    #   marker frame relative to the camera frame, i.e.:
    #   tvec - is a vector of length 3 expressing the
    #          (x,y,z)-coordinates of the marker's center
    #          in the coordinate frame of the camera.
    #   rvec - is a vector of length 3 expressing the
    #          rotation of the marker's frame relative to
    #          the frame of the camera. This vector is an
    #          "axis angle" representation of the rotation.
    # > Hence, a vector expressed in maker-frame coordinates
    #   can be transformed to camera-frame coordinates as:
    #   - Rmat = cv2.Rodrigues(rvec)
    #   - [x,y,z]_{in camera frame} = tvec + Rmat * [x,y,z]_{in marker frame}
    # > Note: the camera frame convention is:
    #   - z-axis points along the optical axis, i.e., straight out of the lens
    #   - x-axis points to the right when looking out of the lens along the z-axis
    #   - y-axis points to the down  when looking out of the lens along the z-axis
    def arucoDetectionsSubscriberCallback(self, msg):

        # Display the data received
        if (self.control_policy_verbosity >= 2):
            rospy.loginfo("[CONTROL POLICY SKELETON] Received aruco detections data for " + str(msg.num_markers) + " markers." )

        # Iterate through the array of detected markers
        for i_marker in range(msg.num_markers):
            # Get the ID, tvec, and rvec for this marker
            this_id   = msg.markers[i_marker].id
            this_tvec = msg.markers[i_marker].tvec
            this_rvec = msg.markers[i_marker].rvec

            # Convert to a World frame estimate
            # NOTE: this skeleton does not include this conversation

        # Fuse with the current state estimate
        # NOTE: this skeleton does not include sensor fusion



if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "control_policy_skeleton"
    rospy.init_node(node_name)

    # Get the namespace of the node
    global node_namespace
    node_namespace = rospy.get_namespace()

    # Initialise an object of the "control policy skeleton" class
    control_policy_object = ControlPolicySkeleton()

    # Spin as a single-threaded node
    rospy.spin()

    # Release any hardware resources that were opened
    # > Nothing to release
