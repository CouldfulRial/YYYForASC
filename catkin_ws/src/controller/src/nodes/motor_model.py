#! /usr/bin/env python

'''
This node measures the speed in v and omega
Subscribed topics:
    /asc/current_motor_duty_cycle  [asclinc_pkg/LeftRightFloat32]
    /asc/encoder_counts            [asclinc_pkg/LeftRightInt32]
Published topics:
    /measured_vel                  [geometry_msgs/Twist]
'''
from math import pi
import rospy

# Subscriber messages
from asclinic_pkg.msg import LeftRightFloat32, LeftRightInt32
from geometry_msgs.msg import Twist, Vector3

# Constants
WHEEL_DIAMETER = 0.144  # m
WHEEL_RADIUS = WHEEL_DIAMETER / 2
WHEEL_BASE = 0.218  # m
HALF_WHEEL_BASE = WHEEL_BASE / 2
CPR = 1120
RADIAN_PER_COUNT = 2 * pi / CPR

TIME_STEP = 0.1  # s, which is the interval encoder publishes data


class MotorModel:
    def __init__(self):
        # Get user parameter
        self.verbosity = rospy.get_param('~verbosity', 1)

        # Initialise node
        rospy.init_node('motor_model', anonymous=True)

        # Initialise the subscribers
        self.sub_encoder = rospy.Subscriber('/asc/encoder_counts', LeftRightInt32, self.update_encoder_callback)
        self.sub_dir = rospy.Subscriber('/asc/current_motor_duty_cycle', LeftRightFloat32, self.update_dir_callback)

        # Initialise the publisher and tf broadcaster
        self.twist_pub = rospy.Publisher("measured_vel", Twist, queue_size=10)

        # Initialise the variables
        self.left_dir, self.right_dir = 1, 1  # 1 for forward, -1 for backward
        self.delta_theta_l, self.delta_theta_r = 0, 0
        self.seq = 0


    def update_encoder_callback(self, data):
        # Update the encoder counts to delta theta for both wheels in RADIAN
        self.delta_theta_l = self.left_dir * data.left * RADIAN_PER_COUNT
        self.delta_theta_r = self.right_dir * data.right * RADIAN_PER_COUNT
        self.seq = data.seq_num

        # Convert the delta theta to v and omega
        v     = WHEEL_RADIUS * (self.delta_theta_l + self.delta_theta_r) / 2           / TIME_STEP # m/s
        omega = WHEEL_RADIUS * (self.delta_theta_r - self.delta_theta_l) / WHEEL_BASE  / TIME_STEP # rad/s

        # log info
        if self.verbosity == 1:
            rospy.loginfo("-"*25 + "Motor Model" + "-"*25 + 
                          f"\nseq: {self.seq}, v: {v:3.5f}, omega: {omega:3.5f}")

        # Publish the speeds
        self.twist_pub.publish(
            Twist(
                linear =Vector3(x=v, y=0, z=0),
                angular=Vector3(x=0, y=0, z=omega)
                  ))

    def update_dir_callback(self, data):
        # Update the directions
        self.left_dir = 1 if data.left >= 0 else -1
        self.right_dir = 1 if data.right >= 0 else -1


if __name__ == '__main__':
    try:
        motor_model = MotorModel()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass