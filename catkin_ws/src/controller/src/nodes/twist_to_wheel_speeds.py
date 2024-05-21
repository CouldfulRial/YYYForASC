#! /usr/bin/env python
'''
This node converts twist speeds to wheel speeds
Subscribed topics:
    cmd_vel    [geometry_msgs/Twist]
Published topics:
    cmd_speeds [asclinic_pkg/LeftRightFloat32]
'''

import rospy
from math import pi
from geometry_msgs.msg import Twist
from asclinic_pkg.msg import LeftRightFloat32, LeftRightFloat32

# CONSTANTS
WHEEL_DIAMETER = 0.144  # m
WHEEL_RADIUS = WHEEL_DIAMETER / 2
WHEEL_BASE = 0.218  # m
HALF_WHEEL_BASE = WHEEL_BASE / 2
TIME_STEP = 0.1

class Twist2WheelSpeeds:
    def __init__(self):
        # Initialise node
        self.node_name = 'twist_to_wheel_speeds'
        rospy.init_node(self.node_name, anonymous=True)

        # Subscribers
        self.mes_speed_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)

        # Timer: Limit the inner loop reference to 1Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        # Publisher
        self.duty_cycle_pub = rospy.Publisher('cmd_speeds', LeftRightFloat32, queue_size=10)

        # Initialise the variables
        self.v = 0
        self.omega = 0

    def cmd_vel_callback(self, data):  # Topic at 10Hz
        self.v     = data.linear.x  # m/s
        self.omega = data.angular.z # rad/s

    def timer_callback(self, event):  # To publish the speeds at 1Hz
        left_speed = (self.v - self.omega * HALF_WHEEL_BASE) / WHEEL_RADIUS  * TIME_STEP
        right_speed = (self.v + self.omega * HALF_WHEEL_BASE) / WHEEL_RADIUS * TIME_STEP

        # Publish
        self.duty_cycle_pub.publish(LeftRightFloat32(
            left=left_speed,
            right=right_speed
            )
        )

if __name__ == '__main__':
    t2w = Twist2WheelSpeeds()
    rospy.spin()
