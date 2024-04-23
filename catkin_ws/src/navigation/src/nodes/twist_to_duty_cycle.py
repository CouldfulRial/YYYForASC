#! /usr/bin/env python

'''
This node converts the input twist to duty cycle
Subscribed topics:
    cmd_vel                   [geometry_msgs/Twist]
Published topics:
    /asc/set_motor_duty_cycle [asclinic_pkg/LeftRightFloat32]
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


class Twist2DC:
    def __init__(self):
        # Get user parameter
        self.verbosity = rospy.get_param('~verbosity', 0)

        # Initialise node
        rospy.init_node('twist_to_duty_cycle', anonymous=True)

        # Initialise the subscribers
        self.sub_encoder = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)

        # Initialise the publisher and tf broadcaster
        self.dc_pub = rospy.Publisher("/asc/set_motor_duty_cycle", LeftRightFloat32, queue_size=10)

    def cmd_vel_callback(self, data):
        # Get v, omega
        v     = data.linear.x
        omega = data.angular.z

        # Convert the delta theta to delta_theta_l and r
        delta_theta_l = (v - omega * HALF_WHEEL_BASE) / WHEEL_RADIUS
        delta_theta_r = (v + omega * HALF_WHEEL_BASE) / WHEEL_RADIUS

        # log info
        if self.verbosity == 1:
            rospy.loginfo("-"*25 + "Twist To DC" + "-"*25 + 
                          f"\ndelta_theta_l: {delta_theta_l:3.5f}, delta_theta_r: {delta_theta_r:3.5f}")

        # Publish the dc
        self.dc_pub.publish(
            LeftRightFloat32(
                left = delta_theta_l,
                right= delta_theta_r
            )
        )

if __name__ == '__main__':
    try:
        motor_model = Twist2DC()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass