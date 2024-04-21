#! /usr/bin/env python

'''
This node publishes the speeds for both wheels
Subscribed topics:
    /asc/current_motor_duty_cycle  [asclinc_pkg/LeftRightFloat32]
    /asc/encoder_counts            [asclinc_pkg/LeftRightInt32]
Published topics:
    /measured_wheel_speeds         [asclinc_pkg/LeftRightFloat32]
'''
from math import pi
import rospy

# Subscriber messages
from asclinic_pkg.msg import LeftRightFloat32, LeftRightInt32

# Constants
CPR = 1120
RADIAN_PER_COUNT = 2 * pi / CPR


class MotorModel:
    def __init__(self):
        rospy.init_node('motor_model', anonymous=True)

        # Initialise the subscribers
        self.sub_encoder = rospy.Subscriber('/asc/encoder_counts', LeftRightInt32, self.update_encoder_callback)
        self.sub_dir = rospy.Subscriber('/asc/current_motor_duty_cycle', LeftRightFloat32, self.update_dir_callback)

        # Initialise the publisher and tf broadcaster
        self.speeds_pub = rospy.Publisher("measured_wheel_speeds", LeftRightFloat32, queue_size=10)

        # Initialise the directions
        self.left_dir, self.right_dir = 1, 1  # 1 for forward, -1 for backward


    def update_encoder_callback(self, data):
        # Update the encoder counts to delta theta for both wheels in radian
        self.delta_theta_l = self.left_dir * data.left * RADIAN_PER_COUNT
        self.delta_theta_r = self.right_dir * data.right * RADIAN_PER_COUNT
        self.seq = data.seq_num

        # Publish the speeds
        self.publish_speeds()

        # log info
        # rospy.loginfo(f"Seq_Num: {data.seq_num:05d}, left_speed: {self.delta_theta_l}, right_speed: {self.delta_theta_r}")


    def update_dir_callback(self, data):
        # Update the directions
        self.left_dir = 1 if data.left >= 0 else -1
        self.right_dir = 1 if data.right >= 0 else -1

        left_dir = "forward" if self.left_dir == 1 else "backward"
        right_dir = "forward" if self.right_dir == 1 else "backward"

        # Display in the console
        # rospy.loginfo(f"Seq_Num: {data.seq_num:05d}, left: {left_dir}, right: {right_dir}")


    def publish_speeds(self):
        speed_msg = LeftRightFloat32()
        speed_msg.left = self.delta_theta_l
        speed_msg.right = self.delta_theta_r
        speed_msg.seq_num = self.seq

        self.speeds_pub.publish(speed_msg)


if __name__ == '__main__':
    try:
        motor_model = MotorModel()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass