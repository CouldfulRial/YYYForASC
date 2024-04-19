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
// Node for emulating the behaviour of the "I2C for Motors and Servos" node
// Emulation is used to perform some tests without a robot
//
// ----------------------------------------------------------------------------





#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"

#include <bitset>

// Include the asclinic message types
#include "asclinic_pkg/LeftRightFloat32.h"
#include "asclinic_pkg/ServoPulseWidth.h"

// Namespacing the package
//using namespace asclinic_pkg;





// MEMBER VARIABLES FOR THIS NODE:
// > Publisher for the current duty cycle
//   setting of the main drive motors
ros::Publisher m_current_motor_duty_cycle_publisher;



// Respond to subscriber receiving a message
// > To test this out without creating an additional
//   ROS node
//   1) First use the command:
//        rostopic list
//      To identify the full name of this subscription topic.
//   2) Then use the following command to send  message on
//      that topic
//        rostopic pub --once <namespace>/set_motor_duty_cycle asclinic_pkg/LeftRightFloat32 "{left: 10.1, right: 10.1}"
//      where "<namespace>/set_motor_duty_cycle" is the full
//      name identified in step 1.
//
void driveMotorsSubscriberCallback(const asclinic_pkg::LeftRightFloat32& msg)
{
	ROS_INFO_STREAM("[EMULATE I2C FOR MOTORS] Message received with left = " << msg.left << ", right = " << msg.right);

	// Clip the data to be in the range [-100.0,100.0]
	// > For the left value
	float pwm_duty_cycle_left = msg.left;
	if (pwm_duty_cycle_left < -100.0f)
		pwm_duty_cycle_left = -100.0f;
	if (pwm_duty_cycle_left > 100.0f)
		pwm_duty_cycle_left = 100.0f;
	// > For the right value
	float pwm_duty_cycle_right = msg.right;
	if (pwm_duty_cycle_right < -100.0f)
		pwm_duty_cycle_right = -100.0f;
	if (pwm_duty_cycle_right > 100.0f)
		pwm_duty_cycle_right = 100.0f;

	// Publish the motor duty cycles
	asclinic_pkg::LeftRightFloat32 msg_current_duty_cycle;
	msg_current_duty_cycle.left  = pwm_duty_cycle_left;
	msg_current_duty_cycle.right = pwm_duty_cycle_right;
	m_current_motor_duty_cycle_publisher.publish(msg_current_duty_cycle);

}



// Respond to subscriber receiving a message
// > To test this out without creating an additional
//   ROS node
//   1) First use the command:
//        rostopic list
//      To identify the full name of this subscription topic.
//   2) Then use the following command to send  message on
//      that topic
//        rostopic pub --once <namespace>/set_servo_pulse_width asclinic_pkg/ServoPulseWidth "{channel: 15, pulse_width_in_microseconds: 1100}"
//      where "<namespace>/set_servo_pulse_width" is the full
//      name identified in step 1.
//
void servoSubscriberCallback(const asclinic_pkg::ServoPulseWidth& msg)
{
	// Extract the channel and pulse width from the message
	uint8_t channel = msg.channel;
	uint16_t pulse_width_in_us = msg.pulse_width_in_microseconds;

	// Display the message received
	ROS_INFO_STREAM("[EMULATE I2C FOR MOTORS] Message received for servo with channel = " << static_cast<int>(channel) << ", and pulse width [us] = " << static_cast<int>(pulse_width_in_us) );

	// Limit the pulse width to be either:
	// > zero
	// > in the range [1000,2000]
	if (pulse_width_in_us > 0)
	{
		if (pulse_width_in_us < 1000)
			pulse_width_in_us = 1000;
		if (pulse_width_in_us > 2000)
			pulse_width_in_us = 2000;
	}
}



int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "emulate_i2c_for_motors");
	ros::NodeHandle nodeHandle("~");

	// Initialise a node handle to the group namespace
	std::string ns_for_group = ros::this_node::getNamespace();
	ros::NodeHandle nh_for_group(ns_for_group);

	// Initialise a subscriber for the duty cycle of the main drive motors
	ros::Subscriber set_motor_duty_cycle_subscriber = nh_for_group.subscribe("set_motor_duty_cycle", 1, driveMotorsSubscriberCallback);
	// Initialise a subscriber for the servo driver
	ros::Subscriber set_servo_pulse_width_subscriber = nh_for_group.subscribe("set_servo_pulse_width", 1, servoSubscriberCallback);

	// Initialise a publisher for the current duty cycle setting of the main drive motors
	m_current_motor_duty_cycle_publisher = nh_for_group.advertise<asclinic_pkg::LeftRightFloat32>("current_motor_duty_cycle", 10, false);

	// Display command line command for publishing a
	// motor duty cycle or servo request
	ROS_INFO_STREAM("[EMULATE I2C FOR MOTORS] publish motor duty cycle requests from command line with: rostopic pub --once " << ros::this_node::getNamespace() << "/set_motor_duty_cycle asclinic_pkg/LeftRightFloat32 \"{left: 10.1, right: 10.1}\"");
	ROS_INFO_STREAM("[EMULATE I2C FOR MOTORS] publish motor duty cycle requests from command line with: rostopic pub --once " << ros::this_node::getNamespace() << "/set_servo_pulse_width asclinic_pkg/ServoPulseWidth \"{channel: 15, pulse_width_in_microseconds: 1100}\"");

	// Spin as a single-threaded node
	ros::spin();

	return 0;
}
