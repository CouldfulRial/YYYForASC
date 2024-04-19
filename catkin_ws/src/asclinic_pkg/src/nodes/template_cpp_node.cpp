// Copyright (C) 2021, The University of Melbourne, Department of Electrical and Electronic Engineering (EEE)
//
// This file is part of ASClinic-System.
//    
// See the root of the repository for license details.
//
// ----------------------------------------------------------------------------
//     _    ____   ____ _ _       _          ____            _                 
//    / \  / ___| / ___| (_)____ (_) ___    / ___| _   _ ___| |_ ___ ________  
//   / _ \ \___ \| |   | | |  _ \| |/ __|___\___ \| | | / __| __/ _ \  _   _ \ 
//  / ___ \ ___) | |___| | | | | | | (_|_____|__) | |_| \__ \ ||  __/ | | | | |
// /_/   \_\____/ \____|_|_|_| |_|_|\___|   |____/ \__, |___/\__\___|_| |_| |_|
//                                                 |___/                       
//
// DESCRIPTION:
// Template C++ node with a publisher and subscriber
//
// ----------------------------------------------------------------------------





// ----------------------------------------------------
// III  N   N   CCCC  L      U   U  DDDD   EEEEE   SSSS
//  I   NN  N  C      L      U   U  D   D  E      S    
//  I   N N N  C      L      U   U  D   D  EEE     SSS 
//  I   N  NN  C      L      U   U  D   D  E          S
// III  N   N   CCCC  LLLLL   UUU   DDDD   EEEEE  SSSS 
// ----------------------------------------------------

// Include some useful libraries
#include <math.h>
#include <stdlib.h>

// Include the ROS libraries
#include "ros/ros.h"
#include <ros/package.h>

// Include the standard message types
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <std_msgs/String.h>

// Include the asclinic message types
#include "asclinic_pkg/TemplateMessage.h"

// Namespacing the package
using namespace asclinic_pkg;



// -----------------------------------------------------------
// V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
// V   V   A A   R   R   I    A A   B   B  L      E      S    
// V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS 
//  V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
//   V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS 
// -----------------------------------------------------------

// Variables declared here can be considered as "member variables"
// for this node. The naming convention used is that each variable
// name begins with "m_".
// Some notes about member variables:
// > They are available to all functions within this node.
// > The value stored persists across calls to functions.
// > If the node runs as a single thread, then you do NOT need
//   to worry about simultaneous access to the variable.
// > If the node runs multi-threaded, then you DO need
//   to worry about simultaneous access to the variable.

// Publisher variable:
ros::Publisher m_template_publisher;

// Timer variable used for recurrent publishing:
ros::Timer m_timer_for_publishing;





//    --------------------------------------------------------------------
//    FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N
//    F      U   U  NN  N  C        T     I   O   O  NN  N
//    FFF    U   U  N N N  C        T     I   O   O  N N N
//    F      U   U  N  NN  C        T     I   O   O  N  NN
//    F       UUU   N   N   CCCC    T    III   OOO   N   N
//
//    PPPP   RRRR    OOO   TTTTT   OOO   TTTTT  Y   Y  PPPP   EEEEE   SSSS
//    P   P  R   R  O   O    T    O   O    T     Y Y   P   P  E      S    
//    PPPP   RRRR   O   O    T    O   O    T      Y    PPPP   EEE     SSS 
//    P      R  R   O   O    T    O   O    T      Y    P      E          S
//    P      R   R   OOO     T     OOO     T      Y    P      EEEEE  SSSS 
//    --------------------------------------------------------------------

// Function prototypes are not strictly required for the code
// to complile, but adding the function prototypes here allows
// the functions to be implemented in any order in the .cpp file.
// If a function prototype is not included below, then that
// function needs to implemented n the .cpp file before every
// other function that it is called from.

// > Callback run when timer fires
void timerCallbackForPublishing(const ros::TimerEvent&);

// > Callback run when subscriber receives a message
void templateSubscriberCallback(const TemplateMessage& template_message);



// -------------------------------------------------------------------------------
// FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N
// F      U   U  NN  N  C        T     I   O   O  NN  N
// FFF    U   U  N N N  C        T     I   O   O  N N N
// F      U   U  N  NN  C        T     I   O   O  N  NN
// F       UUU   N   N   CCCC    T    III   OOO   N   N
//
// III M   M PPPP  L     EEEEE M   M EEEEE N   N TTTTT   A   TTTTT III  OOO  N   N
//  I  MM MM P   P L     E     MM MM E     NN  N   T    A A    T    I  O   O NN  N
//  I  M M M PPPP  L     EEE   M M M EEE   N N N   T   A   A   T    I  O   O N N N
//  I  M   M P     L     E     M   M E     N  NN   T   AAAAA   T    I  O   O N  NN
// III M   M P     LLLLL EEEEE M   M EEEEE N   N   T   A   A   T   III  OOO  N   N
// -------------------------------------------------------------------------------

// > Callback run when timer fires
void timerCallbackForPublishing(const ros::TimerEvent&)
{
	// Declare a static integer for publishing a different
	// number each time
	static uint counter = 0;

	// Increment the counter
	counter++;

	// PUBLISH A MESSAGE
	// Initialise a "TemplateMessage" struct
	// > Note that all value are set to zero by default
	TemplateMessage template_message = TemplateMessage();
	// Set the values
	template_message.temp_bool          = true;
	template_message.temp_uint32        = counter;
	template_message.temp_int32         = -1;
	template_message.temp_float32       = 1.23;
	template_message.temp_float64       = -1.23;
	template_message.temp_string        = "test";
	// Push back elements into the array
	template_message.temp_float64_array.push_back(1.1);
	template_message.temp_float64_array.push_back(2.2);
	template_message.temp_float64_array.push_back(3.3);
	// Publish the message
	m_template_publisher.publish(template_message);

	// START THE TIMER AGAIN
	// > Stop any previous instance that might still be running
	m_timer_for_publishing.stop();
	// > Set the period again (second argument is reset)
	m_timer_for_publishing.setPeriod( ros::Duration(1.0), true);
	// > Start the timer again
	m_timer_for_publishing.start();
}



// > Callback run when subscriber receives a message
void templateSubscriberCallback(const TemplateMessage& template_message)
{
	// Display that a message was received
	ROS_INFO_STREAM("[TEMPLATE CPP NODE] Message receieved with temp_uint32 = " << template_message.temp_uint32);
}




//    ------------------------
//    M   M    A    III  N   N
//    MM MM   A A    I   NN  N
//    M M M  A   A   I   N N N
//    M   M  AAAAA   I   N  NN
//    M   M  A   A  III  N   N
//    ------------------------

int main(int argc, char* argv[])
{
	// NOTE:
	// A number of the function description given in this file are
	// taken directly from the following ROS tutorials:
	// http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
	// http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
	// http://wiki.ros.org/roscpp/Overview/Timers

	// STARTING THE ROS NODE
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "template_cpp_node");

	// CONSTRUCT A NODE HANDLE
	// Construct a "ros::NodeHandle" type local variable "nodeHandle"
	// as the current node, the "~" indcates that "self" is the node
	// handle assigned to this variable.
	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle nodeHandle("~");

	// GET THE NAMESPACE OF THIS NODE
	// > This can be useful for debugging because you may be
	//   launching the same node under different namespaces.
	std::string m_namespace = ros::this_node::getNamespace();
	// Display the namespace
	ROS_INFO_STREAM("[TEMPLATE CPP NODE] ros::this_node::getNamespace() =  " << m_namespace);

	// INITIALISE THE PUBLISHER
	// Initialise the member variable "m_template_publisher" to be
	// a "ros::Publisher" type variable. The publisher needs to be
	// a member variable because it is used in other functions when
	// publishing a message. The syntax for the call to initialise
	// the publisher is:
	//   ros::Publisher advertise<<message_type>>(const std::string& topic, uint32_t queue_size, bool latch = false);
	// where:
	//   <message_type>
	//     Specifies the type that defines the structure of the
	//     message to be communicated on this topic. The commonly
	//     used standard message types, i.e., "std_msgs", are:
	//       std_msgs::Bool
	//       std_msgs::UInt32
	//       std_msgs::Int32
	//       std_msgs::Float32
	//       std_msgs::String
	//       std_msgs::Time
	//       std_msgs::Duration
	//     Custom message type have the structure defined by
	//     files in the "msg" folder of the package, i.e., look
	//     for the "TemplateMessage.msg" file for the example
	//     used below.
	//   topic
	//     String that defines the name under which the topic is
	//     advertised for publication.
	//   queue_size
	//     Integer that defines the number of messages buffered.
	//   latch
	//     Enables "latching" on a connection. When a connection
	//     is latched, the last message published is saved and
	//     automatically sent to any future subscribers that connect.
	//     This is useful for slow-changing to static data like a map.
	//     Default is false.
	//
	/**
	 *  The advertise() function is how you tell ROS that you want to
	 * publish on a given topic name. This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing. After this advertise() call is made, the master
	 * node will notify anyone who is trying to subscribe to this topic name,
	 * and they will in turn negotiate a peer-to-peer connection with this
	 * node.  advertise() returns a Publisher object which allows you to
	 * publish messages on that topic through a call to publish().  Once
	 * all copies of the returned Publisher object are destroyed, the topic
	 * will be automatically unadvertised.
	 *
	 * The second parameter to advertise() is the size of the message queue
	 * used for publishing messages.  If messages are published more quickly
	 * than we can send them, the number here specifies how many messages to
	 * buffer up before throwing some away.
	 */
	m_template_publisher = nodeHandle.advertise<TemplateMessage>("template_topic", 10, false);


	// INITIALISE THE TIMER FOR RECURRENT PUBLISHING
	// Initialise the member variable "m_timer_for_publishing" to be
	// a "ros::Timer" type variable. The syntax for the call to
	// initialise the timer is:
	//   ros::Timer ros::NodeHandle::createTimer(ros::Duration period, <callback>, bool oneshot = false);
	// where:
	//   period
	//     This is the period between calls to the timer callback. For
	//     example, if this is ros::Duration(0.1), the callback will be
	//     scheduled for every 1/10th of a second.
	//   <callback>
	//     This is the callback to be called -- it may be a function,
	//     class method or functor object.
	//   oneshot
	//     Specifies whether or not the timer is a one-shot timer.
	//     If so, it will only fire once. Otherwise it will be re-scheduled
	//     continuously until it is stopped.
	//     If a one-shot timer has fired it can be reused by calling stop()
	//     along with setPeriod(ros::Duration) and start() to be re-scheduled
	//     once again.
	//
	m_timer_for_publishing = nodeHandle.createTimer(ros::Duration(1.0), timerCallbackForPublishing, true);


	// INITIALISE THE SUBSCRIBER
	// Initialise a local variable "template_subscriber" to be a
	// "ros::Subscriber" type variable that subscribes to the
	// topic named "template_topic". The subscriber can be a local
	// variable because as long as the main function is running,
	// i.e., as long as this node is active, then the subsriber will
	// trigger the callback function when messages are received. The
	// syntax for the call to initialise the subscriber is:
	//   .subscribe((const std::string& topic, uint32_t queue_size, <callback>, const ros::TransportHints& transport_hints = ros::TransportHints());
	// where:
	//   topic
	//     String that defines the name of which topic to subscribe to
	//   queue_size
	//     Integer that defines the number of messages buffered.
	//   <callback>
	//     This is the callback to be called. Depending on the version
	//     of subscribe() you're using, this may be any of a few things.
	//     The most common is a class method pointer and a pointer to
	//     the instance of the class.
	//   transport_hints
	//     The transport hints allow you to specify hints to roscpp's
	//     transport layer. This lets you specify things like preferring
	//     a UDP transport, using tcp nodelay, etc.
	//     
	//
	/**
	 * The subscribe() call is how you tell ROS that you want to receive messages
	 * on a given topic.  This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing.  Messages are passed to a callback function, here
	 * called chatterCallback.  subscribe() returns a Subscriber object that you
	 * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	 * object go out of scope, this callback will automatically be unsubscribed from
	 * this topic.
	 *
	 * The second parameter to the subscribe() function is the size of the message
	 * queue.  If messages are arriving faster than they are being processed, this
	 * is the number of messages that will be buffered up before beginning to throw
	 * away the oldest ones.
	 */
	ros::Subscriber template_subscriber = nodeHandle.subscribe("template_topic", 1, templateSubscriberCallback);


	// Display that the node is ready
	ROS_INFO("[TEMPLATE CPP NODE] Ready :-)");

	// SPIN AS A SINGLE-THREADED NODE
	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
	ros::spin();

	return 0;
}
