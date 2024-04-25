#! /usr/bin/env python

'''
This node measures the speed in v and omega
Subscribed topics:
    /asc/current_motor_duty_cycle  [asclinc_pkg/LeftRightFloat32]
    /asc/encoder_counts            [asclinc_pkg/LeftRightInt32]
Published topics:
    /mes_speeds                 [asclinic_pkg/LeftRightFloat32]
'''
from math import pi
import rospy

# Subscriber messages
from asclinic_pkg.msg import LeftRightFloat32, LeftRightInt32
from geometry_msgs.msg import Twist, Vector3

# Constants
CPR = 1120
RADIAN_PER_COUNT = 2 * pi / CPR

TIME_STEP = 0.1  # s, which is the interval encoder publishes data


class MotorModel:
    def __init__(self):
        # Initialise node
        self.node_name = 'motor_model'
        rospy.init_node(self.node_name, anonymous=True)

        # Get user parameters
        self.parms = rospy.get_param(self.node_name)
        self.verbosity = self.parms['verbosity']

        # Initialise the subscribers
        self.sub_encoder = rospy.Subscriber('/asc/encoder_counts', LeftRightInt32, self.update_encoder_callback)
        self.sub_dir = rospy.Subscriber('/asc/current_motor_duty_cycle', LeftRightFloat32, self.update_dir_callback)

        # Initialise the publisher
        self.speeds_pub = rospy.Publisher("mes_speeds", LeftRightFloat32, queue_size=10)

        # Initialise the variables
        self.left_dir, self.right_dir = 1, 1  # 1 for forward, -1 for backward
        self.delta_theta_l, self.delta_theta_r = 0, 0
        self.seq = 0

    def update_encoder_callback(self, data):
        # Update the encoder counts to delta theta for both wheels in RADIAN
        self.delta_theta_l = self.left_dir * data.left * RADIAN_PER_COUNT / TIME_STEP  # rad/s
        self.delta_theta_r = self.right_dir * data.right * RADIAN_PER_COUNT / TIME_STEP  # rad/s
        self.seq = data.seq_num

        # log info
        if self.verbosity == 1:
            rospy.loginfo("-"*25 + "Motor Model" + "-"*25 + 
                          f"\nseq: {self.seq}:" + 
                          f"\n\tleft wheel speed: {self.delta_theta_l} rad/s" +
                          f"\n\tright wheel speed: {self.delta_theta_r} rad/s")

        # Publish the speeds
        self.speeds_pub.publish(
            LeftRightFloat32(
                seq_num=self.seq,
                left=self.delta_theta_l,
                right=self.delta_theta_r
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