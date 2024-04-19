#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import UInt32

class TemplatePyNodeMinimal:

    def __init__(self):
        # Initialise a publisher
        self.template_publisher = rospy.Publisher(node_name+"/template_topic", UInt32, queue_size=10)
        # Initialise a timer
        self.counter = 0
        rospy.Timer(rospy.Duration(1.0), self.timerCallbackForPublishing)
        # Initialise a subscriber
        rospy.Subscriber(node_name+"/template_topic", UInt32, self.templateSubscriberCallback)

    # Respond to timer callback
    def timerCallbackForPublishing(self, event):
        self.counter += 1
        self.template_publisher.publish(self.counter)

    # Respond to subscriber receiving a message
    def templateSubscriberCallback(self, msg):
        rospy.loginfo("[TEMPLATE PY NODE MINIMAL] Message receieved with data = " + str(msg.data) )

if __name__ == '__main__':
    # Initialise the node
    global node_name
    node_name = "template_py_node_minimal"
    rospy.init_node(node_name)
    template_py_node = TemplatePyNodeMinimal()
    # Spin as a single-threaded node
    rospy.spin()
