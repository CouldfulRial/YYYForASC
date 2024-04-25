#! /usr/bin/env python

'''
This node stops the motors
Published topics:
    /asc/set_motor_duty_cycle [asclinc_pkg/LeftRightFloat32]
'''

import rospy
from asclinic_pkg.msg import LeftRightFloat32, LeftRightFloat32

class MotorStopper:
    def __init__(self):
        rospy.init_node('stop_motors')
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

        # Publisher
        self.duty_cycle_pub = rospy.Publisher('/asc/set_motor_duty_cycle', LeftRightFloat32, queue_size=10)

    def timer_callback(self, event):
        # Publish duty cycles
        self.duty_cycle_pub.publish(LeftRightFloat32(left=0, right=0))

if __name__ == '__main__':
    controller = MotorStopper()
    rospy.spin()
