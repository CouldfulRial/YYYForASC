#! /usr/bin/env python

import rospy

class WheelOdom:
    def __init__(self):
        # Get user parameter
        self.args = rospy.get_param('parameter') 
        self.verbosity = self.args['verbosity']

        rospy.init_node('parameter', anonymous=True)

        # Timer: Calls the timer_callback function at 2 Hz
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)


    def timer_callback(self, data):
        rospy.loginfo(f"parameter: {self.verbosity}")

if __name__ == '__main__':
    try:
        wheel_odometry = WheelOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    try:
        motor_model = WheelOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass