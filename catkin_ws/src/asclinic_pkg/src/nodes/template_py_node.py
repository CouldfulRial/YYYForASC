#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (C) 2021, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
#
# This file is part of ASClinic-System.
#    
# See the root of the repository for license details.
#
# ----------------------------------------------------------------------------
#     _    ____   ____ _ _       _          ____            _                 
#    / \  / ___| / ___| (_)____ (_) ___    / ___| _   _ ___| |_ ___ ________  
#   / _ \ \___ \| |   | | |  _ \| |/ __|___\___ \| | | / __| __/ _ \  _   _ \ 
#  / ___ \ ___) | |___| | | | | | | (_|_____|__) | |_| \__ \ ||  __/ | | | | |
# /_/   \_\____/ \____|_|_|_| |_|_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
#                                                 |___/                       
#
# DESCRIPTION:
# Template Python node with a publisher and subscriber
#
# ----------------------------------------------------------------------------





# Import the ROS-Python package
import rospy

# Import the standard message types
from std_msgs.msg import UInt32

# Import the asclinic message types
from asclinic_pkg.msg import TemplateMessage





# NOTE:
# A number of the function description given in this file are
# taken directly from the following ROS tutorials:
# http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
# http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
# http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
# http://wiki.ros.org/rospy/Overview/Time





class TemplatePyNode:

    def __init__(self):
        # INITIALISE THE PUBLISHER
        # Initialise the member variable "self.template_publisher" to
        # be a "Publisher" type variable. The publisher needs to be
        # a member variable because it is used in other functions when
        # publishing a message. The syntax for the call to initialise
        # the publisher is:
        #   pub = rospy.Publisher("topic_name", std_msgs.msg.String, queue_size=10, latch=False)
        # where:
        #   "topic_name"
        #     String that defines the name under which the topic is
        #     advertised for publication.
        #   std_msgs.msg.String
        #     Specifies the type that defines the structure of the
        #     message to be communicated on this topic. The commonly
        #     used standard message types, i.e., "std_msgs", are:
        #       std_msgs.msg.Bool
        #       std_msgs.msg.UInt32
        #       std_msgs.msg.Int32
        #       std_msgs.msg.Float32
        #       std_msgs.msg.String
        #       std_msgs.msg.Time
        #       std_msgs.msg.Duration
        #     Custom message type have the structure defined by
        #     files in the "msg" folder of the package, i.e., look
        #     for the "TemplateMessage.msg" file for the example
        #     used below.
        #   "topic_name"
        #     String that defines the name under which the topic is
        #     advertised for publication.
        #   queue_size=10
        #     Integer that defines the number of messages buffered.
        #   latch=False
        #     Enables "latching" on a connection. When a connection
        #     is latched, the last message published is saved and
        #     automatically sent to any future subscribers that connect.
        #     This is useful for slow-changing to static data like a map.
        #     Default is false.
        #
        # 
        # The rospy.Publisher() function is how you tell ROS that you want
        # to publish on a given topic name. This invokes a call to the ROS
        # master node, which keeps a registry of who is publishing and who
        # is subscribing. After this rospy.Publisher() call is made, the master
        # node will notify anyone who is trying to subscribe to this topic name,
        # and they will in turn negotiate a peer-to-peer connection with this
        # node.  rospy.Publisher() returns a Publisher object which allows you to
        # publish messages on that topic through a call to publish().
        #
        # The queue_size parameters means that if messages are published more
        # quickly than we can send them, the number here specifies how many
        # messages to buffer up before throwing some away.
        #
        self.template_publisher = rospy.Publisher(node_name+"/template_topic", TemplateMessage, queue_size=10)
        
        # INITIALISE THE TIMER FOR RECURRENT PUBLISHING
        # Initialise a local variable to be "rospy.Timer" type variable.
        # The syntax for the call to initialise the timer is:
        #   rospy.Timer(period, callback, oneshot=False)
        # where:
        #   period
        #     This is the period between calls to the timer callback. For
        #     example, if this is rospy.Duration((0.1), the callback will be
        #     scheduled for every 1/10th of a second.
        #   callback
        #     This is the callback to be called. The function is passed a
        #     TimerEvent instance.
        #   oneshot=False
        #     Specifies whether or not the timer is a one-shot timer.
        #     If so, it will only fire once. Otherwise it will be re-scheduled
        #     continuously until it is stopped.
        #
        self.counter = 0
        rospy.Timer(rospy.Duration(1.0), self.timerCallbackForPublishing, oneshot=False)

        # INITIALISE THE SUBSCRIBER
        # Initialise a local variable to be a "rospy.Subscriber" type variable
        # that subscribes to the topic named "template_topic". The subscriber
        # can be a local variable because as long as this instance of the 
        # "TemplatePyNode" class exists, i.e., as long as this node is active
        # as per the "__main__" function, then the subsriber will trigger the
        # callback function when messages are received. The syntax for the call
        # to initialise the subscriber is:
        #   rospy.Subscriber("topic_name", std_msgs.msg.String, callback queue_size=10)
        # where:
        #   topic
        #     String that defines the name of which topic to subscribe to.
        #   callback
        #     This is the callback to be called, most commonly is a class method.
        #   queue_size=10
        #     Integer that defines the number of messages buffered.
        #
        #
        # The rospy.Subscriber() call is how you tell ROS that you want to receive 
        # messages on a given topic.  This invokes a call to the ROS
        # master node, which keeps a registry of who is publishing and who
        # is subscribing.  Messages are passed to a callback function, here
        # called chatterCallback.  rospy.Subscriber() returns a Subscriber object that you
        # must hold on to until you want to unsubscribe.
        #
        # The queue_size parameters means that if messages are arriving faster than they
        # are being processed, this is the number of messages that will be buffered up
        # before beginning to throw away the oldest ones.
        #
        rospy.Subscriber(node_name+"/template_topic", TemplateMessage, self.templateSubscriberCallback)

    # Respond to timer callback
    def timerCallbackForPublishing(self, event):
        # Increment the counter
        self.counter += 1
        # PUBLISH A MESSAGE
        # Initialise a "TemplateMessage" struct
        # > Note that all value are set to zero by default
        template_message = TemplateMessage();
        # Set the values
        template_message.temp_bool          = True;
        template_message.temp_uint32        = self.counter;
        template_message.temp_int32         = -1;
        template_message.temp_float32       = 1.23;
        template_message.temp_float64       = -1.23;
        template_message.temp_string        = "test";
        # Append elements into the array
        template_message.temp_float64_array.append(1.1);
        template_message.temp_float64_array.append(2.2);
        template_message.temp_float64_array.append(3.3);
        # Publish the message
        self.template_publisher.publish(template_message)

    # Respond to subscriber receiving a message
    def templateSubscriberCallback(self, msg):
        # Display that a message was received
        rospy.loginfo("[TEMPLATE PY NODE] Message receieved with data = " + str(msg.temp_uint32) )

if __name__ == '__main__':
    

    # STARTING THE ROS NODE
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global node_name
    node_name = "template_py_node"
    rospy.init_node(node_name, anonymous=False)
    template_py_node = TemplatePyNode()
    
    # SPIN AS A SINGLE-THREADED NODE
    #
    # rospy.spin() will enter a loop, pumping callbacks.  With this version, all
    # callbacks will be called from within this thread (the main one).  ros::spin()
    # will exit when Ctrl-C is pressed, or the node is shutdown by the master.
    #
    rospy.spin()
