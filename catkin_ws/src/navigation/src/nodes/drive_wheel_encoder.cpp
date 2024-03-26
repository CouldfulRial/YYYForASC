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
// Node for monitoring GPIO pins connected to wheel encoders
//
// ----------------------------------------------------------------------------




#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"

#include <string>
#include <gpiod.h>

// Includes required for threading
// > Mutex for handling shared variable
#include <mutex>
// > Include only one of the following two options:
#include <thread>
//#include <boost/thread/thread.hpp>

// Include the message types
#include "navigation/LRInt32.h"
// #include "asclinic_pkg/LeftRightFloat32.h"

// Include the asclinic constants
//#include "nodes/constant.h"

// Namespacing the package
//using namespace asclinic_pkg;

// MEMBER VARIABLES THAT ARE PARAMETERS FOR THIS NODE:
// > For the verbosity level of displaying info
//   Note: the levels of increasing verbosity are defined as:
//   0 : Info is not displayed. Warnings and errors are still displayed
//   1 : Startup info is displayed
//   2 : Info about messages published is displayed
int m_encoder_read_verbosity = 2;

// > The "gpiochip" number
int m_gpio_chip_number = 1;

// Flags indicating which encoder channels to minotor
bool m_should_monitor_left_side_encoder_channel_a  = true;
bool m_should_monitor_left_side_encoder_channel_b  = false;
bool m_should_monitor_right_side_encoder_channel_a = true;
bool m_should_monitor_right_side_encoder_channel_b = false;

// > The line numbers of the encoder channels
int m_line_number_for_left_side_encoder_channel_a  = 105;
int m_line_number_for_left_side_encoder_channel_b  = 106;
int m_line_number_for_right_side_encoder_channel_a = 84;
int m_line_number_for_right_side_encoder_channel_b = 130;

// > The events to monitor on the encoder channels
std::string m_encoder_events_to_monitor = "rising";

// > The "delta t" used for the frequency of publishing encoder counts
float m_delta_t_for_publishing_counts = 0.1;

// ALL OTHER MEMBER VARIABLES FOR THIS NODE:
// > Publisher and timer for the current
//   encoder counts
ros::Publisher m_encoder_counts_publisher;
ros::Timer m_timer_for_publishing;

// > For sharing the encoder counts between threads
int m_encoder_counts_for_left_a  = 0;
int m_encoder_counts_for_left_b  = 0;
int m_encoder_counts_for_right_a = 0;
int m_encoder_counts_for_right_b = 0;

// > Mutex for preventing multiple-access of shared variables
std::mutex m_counting_mutex;

// > Boolean flag for when to stop counting
//   (This is important for releasing the GPIO lines when the node is shut down)
bool encoder_thread_should_count = true;

// FUNCTION: Respond to timer callback
void timerCallbackForPublishing(const ros::TimerEvent&)
{
	// Sequnce number for tracking the order of published messages
	static unsigned long int s_sequence_number_for_msgs = 1;

	// Get the current counts into a local variable
	// > And reset the shared counts variable to zero
	int counts_left_a_local_copy;
	int counts_left_b_local_copy;
	int counts_right_a_local_copy;
	int counts_right_b_local_copy;
	m_counting_mutex.lock();
	counts_left_a_local_copy  = m_encoder_counts_for_left_a;
	counts_left_b_local_copy  = m_encoder_counts_for_left_b;
	counts_right_a_local_copy = m_encoder_counts_for_right_a;
	counts_right_b_local_copy = m_encoder_counts_for_right_b;
	m_encoder_counts_for_left_a  = 0;
	m_encoder_counts_for_left_b  = 0;
	m_encoder_counts_for_right_a = 0;
	m_encoder_counts_for_right_b = 0;
	m_counting_mutex.unlock();

	// Publish a message
	navigation::LRInt32 msg;
	msg.left  = counts_left_a_local_copy  + counts_left_b_local_copy;
	msg.right = counts_right_a_local_copy + counts_right_b_local_copy;
	msg.seq_num = s_sequence_number_for_msgs;
	m_encoder_counts_publisher.publish(msg);

	// Increment the sequence number
	s_sequence_number_for_msgs++;

	// Display the counts
	if (m_encoder_read_verbosity >= 2)
		ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] Published (seq_num,left,right) = ( " << std::setw(5) << msg.seq_num << " , " << std::setw(5) << msg.left << " , " << std::setw(5) << msg.right << " )");
}

// FUNCTION: perform counting on the events on the encoder channels
void encoderCountingThreadMain()
{
	// Specify the chip name of the GPIO interface
	// > Note: for the 40-pin header of the Jetson SBCs, this
	//   is "/dev/gpiochip1"
	std::stringstream temp_string_stream;
	temp_string_stream << "/dev/gpiochip" << m_gpio_chip_number;
	const char * gpio_chip_name = temp_string_stream.str().c_str();

	// Make a local copy of the line number member variables
	int line_number_left_a  = m_line_number_for_left_side_encoder_channel_a;
	int line_number_left_b  = m_line_number_for_left_side_encoder_channel_b;
	int line_number_right_a = m_line_number_for_right_side_encoder_channel_a;
	int line_number_right_b = m_line_number_for_right_side_encoder_channel_b;

	// Initialise a GPIO chip, line, and event objects
	struct gpiod_chip *chip;
	struct gpiod_line *line_left_a;
	struct gpiod_line *line_left_b;
	struct gpiod_line *line_right_a;
	struct gpiod_line *line_right_b;
	struct gpiod_line_bulk line_bulk;
	struct gpiod_line_event event;
	struct gpiod_line_bulk event_bulk;

	// Specify the timeout specifications
	// > The first entry is seconds
	// > The second entry is nano-seconds
	struct timespec timeout_spec = { 0, 10000000 };
	
	// Intialise a variable for the flags returned
	// by GPIO calls
	int returned_wait_flag;
	int returned_read_flag;

	// Get and print the value of the GPIO line
	// > Note: the third argument to "gpiod_ctxless_get_value"
	//   is an "active_low" boolean input argument.
	//   If true, this indicate to the function that active state
	//   of this line is low.
	int value;
	// > For left-side encoder channel A
	if (m_should_monitor_left_side_encoder_channel_a)
	{
		value = gpiod_ctxless_get_value(gpio_chip_name, line_number_left_a, false, "foobar");
		if (m_encoder_read_verbosity >= 1)
			ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_left_a << " returned value = " << value);
	}
	// > For left-side encoder channel B
	if (m_should_monitor_left_side_encoder_channel_b)
	{
		value = gpiod_ctxless_get_value(gpio_chip_name, line_number_left_b, false, "foobar");
		if (m_encoder_read_verbosity >= 1)
			ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_left_b << " returned value = " << value);
	}
	// > For right-side encoder channel A
	if (m_should_monitor_right_side_encoder_channel_a)
	{
		value = gpiod_ctxless_get_value(gpio_chip_name, line_number_right_a, false, "foobar");
		if (m_encoder_read_verbosity >= 1)
			ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_right_a << " returned value = " << value);
	}
	// > For right-side encoder channel B
	if (m_should_monitor_right_side_encoder_channel_b)
	{
		value = gpiod_ctxless_get_value(gpio_chip_name, line_number_right_b, false, "foobar");
		if (m_encoder_read_verbosity >= 1)
			ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] On startup of node, chip " << gpio_chip_name << " line " << line_number_right_b << " returned value = " << value);
	}

	// Open the GPIO chip
	chip = gpiod_chip_open(gpio_chip_name);
	// Retrieve the GPIO lines
	if (m_should_monitor_left_side_encoder_channel_a)
		line_left_a  = gpiod_chip_get_line(chip,line_number_left_a);
	if (m_should_monitor_left_side_encoder_channel_b)
		line_left_b  = gpiod_chip_get_line(chip,line_number_left_b);
	if (m_should_monitor_right_side_encoder_channel_a)
		line_right_a = gpiod_chip_get_line(chip,line_number_right_a);
	if (m_should_monitor_right_side_encoder_channel_b)
		line_right_b = gpiod_chip_get_line(chip,line_number_right_b);
	// Initialise the line bulk
	gpiod_line_bulk_init(&line_bulk);
	// Add the lines to the line bulk
	if (m_should_monitor_left_side_encoder_channel_a)
		gpiod_line_bulk_add(&line_bulk, line_left_a);
	if (m_should_monitor_left_side_encoder_channel_b)
		gpiod_line_bulk_add(&line_bulk, line_left_b);
	if (m_should_monitor_right_side_encoder_channel_a)
		gpiod_line_bulk_add(&line_bulk, line_right_a);
	if (m_should_monitor_right_side_encoder_channel_b)
		gpiod_line_bulk_add(&line_bulk, line_right_b);

	// Display the status
	if (m_encoder_read_verbosity >= 1)
	{
		ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] Chip " << gpio_chip_name << " opened");
		if (m_should_monitor_left_side_encoder_channel_a)
			ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] lines " << line_number_left_a << " retrieved");
		if (m_should_monitor_left_side_encoder_channel_b)
			ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] lines " << line_number_left_b << " retrieved");
		if (m_should_monitor_right_side_encoder_channel_a)
			ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] lines " << line_number_right_a << " retrieved");
		if (m_should_monitor_right_side_encoder_channel_b)
			ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] lines " << line_number_right_b << " retrieved");
	}

	// Request the line events to be mointored
	// > Note: only one of these should be uncommented
	if (m_encoder_events_to_monitor.compare(0,std::string::npos,"rising"))
		gpiod_line_request_bulk_rising_edge_events(&line_bulk, "foobar");
	else if (m_encoder_events_to_monitor.compare(0,std::string::npos,"falling"))
		gpiod_line_request_bulk_falling_edge_events(&line_bulk, "foobar");
	else if (m_encoder_events_to_monitor.compare(0,std::string::npos,"both"))
		gpiod_line_request_bulk_both_edges_events(&line_bulk, "foobar");
	else
	{
		ROS_WARN_STREAM("[ENCODER READ MULTI THREADED] The value of \"encoder_events_to_monitor\" (which is \"" << m_encoder_events_to_monitor << "\") is not recognised.");
		ROS_WARN_STREAM("[ENCODER READ MULTI THREADED] Defaulting to counting rising edges.");
		gpiod_line_request_bulk_rising_edge_events(&line_bulk, "foobar");
	}

	// Display the line event values for rising and falling
	//ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] The constants defined for distinguishing line events are:, GPIOD_LINE_EVENT_RISING_EDGE = " << GPIOD_LINE_EVENT_RISING_EDGE << ", and GPIOD_LINE_EVENT_FALLING_EDGE = " << GPIOD_LINE_EVENT_FALLING_EDGE);

	// Enter a loop that endlessly monitors the encoders
	while (encoder_thread_should_count)
	{

		// Monitor for the requested events on the GPIO line bulk
		// > Note: the function "gpiod_line_event_wait" returns:
		//    0  if wait timed out
		//   -1  if an error occurred
		//    1  if an event occurred.
		returned_wait_flag = gpiod_line_event_wait_bulk(&line_bulk, &timeout_spec, &event_bulk);

		// Respond based on the the return flag
		if (returned_wait_flag == 1)
		{
			// Get the number of events that occurred
			int num_events_during_wait = gpiod_line_bulk_num_lines(&event_bulk);

			// Lock the mutex while before counting the events
			m_counting_mutex.lock();

			// Iterate over the event
			for (int i_event = 0; i_event < num_events_during_wait; i_event++)
			{
				// Get the line handle for this event
				struct gpiod_line *line_handle = gpiod_line_bulk_get_line(&event_bulk, i_event);

				// Get the number of this line
				unsigned int this_line_number = gpiod_line_offset(line_handle);

				// Read the event on the GPIO line
				// > Note: the function "gpiod_line_event_read" returns:
				//    0  if the event was read correctly
				//   -1  if an error occurred
				returned_read_flag = gpiod_line_event_read(line_handle,&event);

				// Respond based on the the return flag
				if (returned_read_flag == 0)
				{
					// Increment the respective count
					if (this_line_number == line_number_left_a)
						m_encoder_counts_for_left_a++;
					else if (this_line_number == line_number_left_b)
						m_encoder_counts_for_left_b++;
					else if (this_line_number == line_number_right_a)
						m_encoder_counts_for_right_a++;
					else if (this_line_number == line_number_right_b)
						m_encoder_counts_for_right_b++;

				} // END OF: "if (returned_read_flag == 0)"

			} // END OF: "for (int i_event = 0; i_event < num_events_during_wait; i_event++)"

			// Unlock the mutex
			m_counting_mutex.unlock();

		} // END OF: "if (returned_wait_flag == 1)"
	} // END OF: "while (encoder_thread_should_count)"

	// Release the lines
	gpiod_line_release_bulk(&line_bulk);
	// Close the GPIO chip
	gpiod_chip_close(chip);
	// Inform the user
	ROS_INFO("[ENCODER READ MULTI THREADED] Lines released and GPIO chip closed");
}

// FUNCTION: main initialization of node
int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "encoder_read_multi_threaded");
	ros::NodeHandle nodeHandle("~");

	// Initialise a node handle to the group namespace
	std::string ns_for_group = ros::this_node::getNamespace();
	ros::NodeHandle nh_for_group(ns_for_group);

	// Display that this node is launcing
	ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] Now launching this node in namespace: " << ns_for_group);

	// Get the parameter values:
	// Notes:
	// > If you look at the "encoder.launch" file located in
	//   the "launch" folder, you see the following lines of code:
	//       <param
	//           name   = "line_number_for_left_side_encoder_channel_a"
	//           value  = 105
	//       />
	// > These lines of code add a parameter to the namespace of this node,
	//   with the parameter name:
	//     "line_number_for_left_side_encoder_channel_a"
	// > Thus, to access the value of this parameter, we use a handle to
	//   this node, which includes the namespace in which the node was
	//   launched.

	// Get the verbosity parameters:
	if ( !nodeHandle.getParam("encoder_read_verbosity", m_encoder_read_verbosity) ) {
		ROS_WARN("[ENCODER READ MULTI THREADED] FAILED to get \"encoder_read_verbosity\" parameter. Using default value instead.");
	}

	// Get the "gpiochip" number parameter:
	if ( !nodeHandle.getParam("gpio_chip_number_for_encoder_lines", m_gpio_chip_number) ) {
		ROS_WARN("[ENCODER READ MULTI THREADED] FAILED to get \"gpio_chip_number_for_encoder_lines\" parameter. Using default value instead.");
	}

	// Get the "should monitor line" parameters:
	// > For channel A of the left side encoder
	if ( !nodeHandle.getParam("should_monitor_left_side_encoder_channel_a", m_should_monitor_left_side_encoder_channel_a) ) {
		ROS_WARN("[ENCODER READ MULTI THREADED] FAILED to get \"should_monitor_left_side_encoder_channel_a\" parameter. Using default value instead.");
	}
	// > For channel B of the left side encoder
	if ( !nodeHandle.getParam("should_monitor_left_side_encoder_channel_b", m_should_monitor_left_side_encoder_channel_b) ) {
		ROS_WARN("[ENCODER READ MULTI THREADED] FAILED to get \"should_monitor_left_side_encoder_channel_b\" parameter. Using default value instead.");
	}
	// > For channel A of the right side encoder
	if ( !nodeHandle.getParam("should_monitor_right_side_encoder_channel_a", m_should_monitor_right_side_encoder_channel_a) ) {
		ROS_WARN("[ENCODER READ MULTI THREADED] FAILED to get \"should_monitor_right_side_encoder_channel_a\" parameter. Using default value instead.");
	}
	// > For channel A of the right side encoder
	if ( !nodeHandle.getParam("should_monitor_right_side_encoder_channel_b", m_should_monitor_right_side_encoder_channel_b) ) {
		ROS_WARN("[ENCODER READ MULTI THREADED] FAILED to get \"should_monitor_right_side_encoder_channel_b\" parameter. Using default value instead.");
	}

	// Get the line number parameters:
	// > For channel A of the left side encoder
	if ( !nodeHandle.getParam("line_number_for_left_side_encoder_channel_a", m_line_number_for_left_side_encoder_channel_a) ) {
		ROS_WARN("[ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_left_side_encoder_channel_a\" parameter. Using default value instead.");
	}
	// > For channel B of the left side encoder
	if ( !nodeHandle.getParam("line_number_for_left_side_encoder_channel_b", m_line_number_for_left_side_encoder_channel_b) ) {
		ROS_WARN("[ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_left_side_encoder_channel_b\" parameter. Using default value instead.");
	}
	// > For channel A of the right side encoder
	if ( !nodeHandle.getParam("line_number_for_right_side_encoder_channel_a", m_line_number_for_right_side_encoder_channel_a) ) {
		ROS_WARN("[ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_right_side_encoder_channel_a\" parameter. Using default value instead.");
	}
	// > For channel A of the right side encoder
	if ( !nodeHandle.getParam("line_number_for_right_side_encoder_channel_b", m_line_number_for_right_side_encoder_channel_b) ) {
		ROS_WARN("[ENCODER READ MULTI THREADED] FAILED to get \"line_number_for_right_side_encoder_channel_b\" parameter. Using default value instead.");
	}

	// Get the parameter for which line events to monitor
	if ( !nodeHandle.getParam("encoder_events_to_monitor", m_encoder_events_to_monitor) ) {
		ROS_WARN("[ENCODER READ MULTI THREADED] FAILED to get \"encoder_events_to_monitor\" parameter. Using default value instead.");
	}

	// Get the "detla t" parameter for the publishing frequency
	if ( !nodeHandle.getParam("delta_t_for_publishing_encoder_counts", m_delta_t_for_publishing_counts) ) {
		ROS_WARN("[ENCODER READ MULTI THREADED] FAILED to get \"delta_t_for_publishing_encoder_counts\" parameter. Using default value instead.");
	}

	// > Display the line numbers being monitored
	if (m_encoder_read_verbosity >= 1)
	{
		if (m_should_monitor_left_side_encoder_channel_a)
			ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] Will monitor line number " << m_line_number_for_left_side_encoder_channel_a << " for left-side encoder channel A");
		if (m_should_monitor_left_side_encoder_channel_b)
			ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] Will monitor line number " << m_line_number_for_left_side_encoder_channel_b << " for left-side encoder channel B");
		if (m_should_monitor_right_side_encoder_channel_a)
			ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] Will monitor line number " << m_line_number_for_right_side_encoder_channel_a << " for right-side encoder channel A");
		if (m_should_monitor_right_side_encoder_channel_b)
			ROS_INFO_STREAM("[ENCODER READ MULTI THREADED] Will monitor line number " << m_line_number_for_right_side_encoder_channel_b << " for right-side encoder channel B");
	}

	// Initialise a publisher for the encoder counts
	// > Note, the second is the size of our publishing queue. We choose to
	//   buffer encoder counts messages because we want to avoid losing counts.
	m_encoder_counts_publisher = nh_for_group.advertise<navigation::LRInt32>("encoder_counts", 10, false);

	// Initialise a timer
	m_timer_for_publishing = nodeHandle.createTimer(ros::Duration(m_delta_t_for_publishing_counts), timerCallbackForPublishing, false);

	// Create thread for counting the encoder events
	std::thread encoder_counting_thread (encoderCountingThreadMain);
	//boost::thread encoder_counting_thread(encoderCountingThreadMain);

	if (m_encoder_read_verbosity >= 1)
		ROS_INFO("[ENCODER READ MULTI THREADED] Node initialisation complete");

	// Spin the node
	ros::spin();

	// Set the flag to stop counting
	encoder_thread_should_count = false;

	// Join back the encoder counting thread
	encoder_counting_thread.join();

	return 0;
}
