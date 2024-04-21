#!/usr/bin/env python

'''
This node controls the motion (outer controller)
Subscribed topics:
    measured_wheel_speeds     [asclinc_pkg/LeftRightFloat32]
    reference_wheel_speeds    [asclinc_pkg/LeftRightFloat32]
Published topics:
    /asc/set_motor_duty_cycle [asclinc_pkg/LeftRightFloat32]
'''

import rospy
import math
from geometry_msgs.msg import Pose2D
from asclinic_pkg.msg import MotorSpeeds

class KinematicController:
    def __init__(self):
        rospy.init_node('kinematic_controller')

        # Subscriber
        self.pose_sub = rospy.Subscriber('/robot_pose', Pose2D, self.pose_callback)

        # Publisher
        self.speed_pub = rospy.Publisher('/motor_speeds', MotorSpeeds, queue_size=10)

        # Desired pose
        self.desired_pose = Pose2D(x=5.0, y=5.0, theta=math.pi/2)  # Example target

        # Controller gains
        self.Kp_pos = 0.5  # Position gain
        self.Kp_ang = 0.3  # Angular gain

    def pose_callback(self, msg):
        # Compute pose error
        error_x = self.desired_pose.x - msg.x
        error_y = self.desired_pose.y - msg.y
        error_theta = self.desired_pose.theta - msg.theta

        # Compute control signals (simple proportional controller for demonstration)
        v = self.Kp_pos * math.sqrt(error_x**2 + error_y**2)
        omega = self.Kp_ang * error_theta

        # Assuming differential
