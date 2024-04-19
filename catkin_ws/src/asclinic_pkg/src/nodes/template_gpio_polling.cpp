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
// Template node for polling the value of a GPIO pin at a fixed frequency
//
// ----------------------------------------------------------------------------





#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Int32.h"

#include <gpiod.h>





// Respond to subscriber receiving a message
void templateSubscriberCallback(const std_msgs::Int32& msg)
{
	ROS_INFO_STREAM("[TEMPLATE GPIO POLLING] Message received with data = " << msg.data);
}

int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "template_gpio_polling");
	ros::NodeHandle nodeHandle("~");

	// Initialise a node handle to the group namespace
	std::string ns_for_group = ros::this_node::getNamespace();
	ros::NodeHandle nh_for_group(ns_for_group);

	// Initialise a publisher
	ros::Publisher gpio_event_publisher = nh_for_group.advertise<std_msgs::Int32>("gpio_event", 10, false);
	// Initialise a subscriber
	// > Note that the subscriber is included only for the purpose
	//   of demonstrating this template node running stand-alone
	ros::Subscriber gpio_event_subscriber = nh_for_group.subscribe("gpio_event", 1, templateSubscriberCallback);



	// Initialise a variable with loop rate for
	// polling the GPIO pin
	// > Input argument is the frequency in hertz, as a double
	ros::Rate loop_rate(0.75);



	// Specify the chip name of the GPIO interface
	// > Note: for the 40-pin header of the Jetson SBCs, this
	//   is "/dev/gpiochip1"
	const char * gpio_chip_name = "/dev/gpiochip1";

	// Get the GPIO line number to monitor
	// Notes:
	// > If you look at the "template_gpio.launch" file located in
	//   the "launch" folder, you see the following lines of code:
	//       <param
	//           name   = "line_number"
	//           value  = 148
	//       />
	// > These lines of code add a parameter named "line_number"
	//   to the this node.
	// > Thus, to access this "line_number" parameter, we first
	//   get a handle to this node within the namespace that it
	//   was launched.
	//std::string namespace = ros::this_node::getNamespace();
	int line_number = 0;
	if ( !nodeHandle.getParam("line_number", line_number) )
	{
		ROS_INFO("[TEMPLATE GPIO POLLING] FAILED to get \"line_number\" parameter. Using default value instead.");
		// Set the line number to a default value
		line_number = 157;
	}
	// > Display the line number being monitored
	ROS_INFO_STREAM("[TEMPLATE GPIO POLLING] Will monitor \"line_number\" = " << line_number);

	// Initialise a GPIO chip, line, and event object
	struct gpiod_chip *chip;
	struct gpiod_line *line;

	// Get and print the value of the GPIO line
	// > Note: the third argument to "gpiod_ctxless_get_value"
	//   is an "active_low" boolean input argument.
	//   If true, this indicate to the function that active state
	//   of this line is low.
	int value;
	value = gpiod_ctxless_get_value(gpio_chip_name, line_number, false, "foobar");
	ROS_INFO_STREAM("[TEMPLATE GPIO POLLING] On startup of node, chip " << gpio_chip_name << " line " << line_number << " returned value = " << value);

	// Open the GPIO chip
	chip = gpiod_chip_open(gpio_chip_name);
	// Retrieve the GPIO line
	line = gpiod_chip_get_line(chip,line_number);
	// Display the status
	ROS_INFO_STREAM("[TEMPLATE GPIO POLLING] Chip " << gpio_chip_name << " opened and line " << line_number << " retrieved.");

	

	// Enter a loop that continues while ROS is still running
	while (ros::ok())
	{

		// Get the current value of the line
		// > Note: the third argument to "gpiod_ctxless_get_value"
		//   is an "active_low" boolean input argument.
		//   If true, this indicate to the function that active state
		//   of this line is low.
		int current_value;
		current_value = gpiod_ctxless_get_value(gpio_chip_name, line_number, false, "foobar");

		// Respond only if the value is 0 or 1
		if (current_value==0 || current_value==1)
		{
			// Display the value
			//ROS_INFO_STREAM("[TEMPLATE GPIO POLLING] gpiod_ctxless_get_value returned \"current_value\" = " << current_value);
			// Publish a message
			std_msgs::Int32 msg;
			msg.data = current_value;
			gpio_event_publisher.publish(msg);
		}
		else
		{
			// Display the status
			ROS_INFO_STREAM("[TEMPLATE GPIO POLLING] gpiod_ctxless_get_value returned unexpected value, \"current_value\" = " << current_value);
		}

		// Spin once so that this node can service any
		// callbacks that this node has queued.
		ros::spinOnce();

		// Sleep for the specified loop rate
		loop_rate.sleep();
	} // END OF: "while (ros::ok())"

	// Close the GPIO chip
	gpiod_chip_close(chip);

	return 0;
}
