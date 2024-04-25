#! /usr/bin/env python
'''
This node provides the location esitmaion of the robot based on the camera

Subscribed topics:

Published topics:

'''
import rospy
from std_msgs.msg import String
from asclinic_pkg.msg import LeftRightFloat32, LeftRightFloat32
import matplotlib.pyplot as plt


class VisionBasedLocalisation:
    def __init__(self):
         # Initialise node
        self.node_name = 'motor_controller'
        rospy.init_node(self.node_name)

        # Get user parameter
        self.parm  = rospy.get_param(self.node_name)
        self.verbosity = self.parm["verbosity"]
        self.plot      = self.parm["plot"]

        # Subscribers
        # rospy.Subscriber(topic_name, msg_type, callback_function)
        self.sub = rospy.Subscriber('SUBSCRIBED TOPIC NAME', String, self.ref_twist_callback)

        # Timer: Calls the timer_callback function at frequency in Hz
        # rospy.Timer(rospy.Duration(1/frequency), callback_function)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)

        # Publisher
        # rospy.Publisher(topic_name, msg_type, queue_size)
        self.pub = rospy.Publisher('PUBLISH TO TOPIC NAME', String, queue_size=10)

        # Register the shutdown callback
        rospy.on_shutdown(self.shutdown_callback)

    def timer_callback(self, event):
        pass

    def sub_callback(self, data):
        pass

    def shutdown_callback(self):
        pass

    def plot_callback(self):
        plt.plot(self.left_speed_list, label='Left Wheel Speed')
        plt.xlabel('steps')
        plt.ylabel('speeds')
        plt.title("motor controller")
        plt.show()

if __name__ == '__main__':
    vbl = VisionBasedLocalisation()
    rospy.spin()
