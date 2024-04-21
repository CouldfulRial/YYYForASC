#!/usr/bin/env python

import rospy
from asclinic_pkg import LeftRightInt32, LeftRightFloat32

# Define parameters
DUTY_CYCLE = 25
COUNT_PER_REVOLUTION = 1120
ANGLE_PER_COUNT = 360 / COUNT_PER_REVOLUTION
TARGET_ANGLE = 15


class RotateByAngle:
    def __init__(self):
        rospy.init_node('encoder_counter', anonymous=True)
        self.subscriber = rospy.Subscriber('encoder_counts', LeftRightInt32, self.callback)
        
        # Initialize accumulated counts
        self.accumulated_left = 0
        self.accumulated_right = 0

    def callback(self, data):
        # Update accumulated counts
        self.accumulated_left += data.left
        self.accumulated_right += data.right

        # Display the accumulated counts and sequence number in the console
        rospy.loginfo(f"Seq_Num: {data.seq_num}, Accumulated Left: {self.accumulated_left}, Accumulated Right: {self.accumulated_right}")
        

if __name__ == '__main__':
    try:
        RBA = RotateByAngle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
