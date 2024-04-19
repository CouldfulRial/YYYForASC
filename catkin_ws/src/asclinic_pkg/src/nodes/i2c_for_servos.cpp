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
// Node for I2C bus with only a servo driver device connect
//
// ----------------------------------------------------------------------------





#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"

#include "i2c_driver/i2c_driver.h"

#include "pca9685/pca9685.h"

#include <bitset>

// Include the asclinic message types
#include "asclinic_pkg/ServoPulseWidth.h"

// Include the asclinic constants
//#include "nodes/constant.h"

// Namespacing the package
//using namespace asclinic_pkg;





// MEMBER VARIABLES THAT ARE PARAMETERS FOR THIS NODE:
// > For the verbosity level of displaying info
//   Note: the levels of increasing verbosity are defined as:
//   0 : Info is not displayed. Warnings and errors are still displayed
//   1 : Startup info is displayed
//   2 : Info about messages received is displayed
int m_servo_driver_verbosity = 1;

// Settings for the servo driver
float m_pwm_frequency_in_hz = 50.0;
uint16_t m_min_pulse_width_in_us = 500;
uint16_t m_max_pulse_width_in_us = 2500;



// ALL OTHER MEMBER VARIABLES FOR THIS NODE:
// > For the I2C driver
const char * m_i2c_device_name = "/dev/i2c-8";
I2C_Driver m_i2c_driver (m_i2c_device_name);

// > For the PCA9685 PWM Servo Driver driver
const uint8_t m_pca9685_address = 0x42;
PCA9685 m_pca9685_servo_driver (&m_i2c_driver, m_pca9685_address);





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

	// Display the values from the message received
	if (m_servo_driver_verbosity >= 2)
		ROS_INFO_STREAM("[I2C FOR SERVOS] Message received for servo with channel = " << static_cast<int>(channel) << ", and pulse width [us] = " << static_cast<int>(pulse_width_in_us) );

	// Limit the pulse width to be either:
	// > zero, or
	// > in the range [m_min_pulse_width_in_us,m_max_pulse_width_in_us]
	if (pulse_width_in_us > 0)
	{
		if (pulse_width_in_us < m_min_pulse_width_in_us)
			pulse_width_in_us = m_min_pulse_width_in_us;
		if (pulse_width_in_us > m_max_pulse_width_in_us)
			pulse_width_in_us = m_max_pulse_width_in_us;
	}

	// Call the function to set the desired pulse width
	bool result = m_pca9685_servo_driver.set_pwm_pulse_in_microseconds(channel, pulse_width_in_us);

	// Display if an error occurred
	if (!result)
		ROS_WARN_STREAM("[I2C FOR SERVOS] FAILED to set pulse width for servo at channel " << static_cast<int>(channel) );
}



int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "i2c_for_servos");
	ros::NodeHandle nodeHandle("~");

	// Initialise a node handle to the group namespace
	std::string ns_for_group = ros::this_node::getNamespace();
	ros::NodeHandle nh_for_group(ns_for_group);

	// Display that this node is launcing
	ROS_INFO_STREAM("[I2C FOR SERVOS] Now launching this node in namespace: " << ns_for_group);

	// Get the parameter values:
	// > For the verbosity
	if ( !nodeHandle.getParam("servo_driver_verbosity", m_servo_driver_verbosity) ) {
		ROS_WARN("[I2C FOR SERVOS] FAILED to get \"servo_driver_verbosity\" parameter. Using default value instead.");
	}
	// > For the current limit parameter:
	if ( !nodeHandle.getParam("servo_driver_pwm_frequency_in_hertz", m_pwm_frequency_in_hz) ) {
		ROS_WARN("[I2C FOR SERVOS] FAILED to get \"pwm_frequency_in_hertz\" parameter. Using default value instead.");
	}
	// > For the min pulse width parameter:
	int temp_min_pulse_width;
	if ( !nodeHandle.getParam("servo_driver_min_pulse_width_in_microseconds", temp_min_pulse_width) ) {
		ROS_WARN("[I2C FOR SERVOS] FAILED to get \"min_pulse_width_in_microseconds\" parameter. Using default value instead.");
	}
	else {
		if (0 <= temp_min_pulse_width && temp_min_pulse_width <= 65535)
			m_min_pulse_width_in_us = static_cast<uint16_t>(temp_min_pulse_width);
		else
			ROS_WARN_STREAM("[I2C FOR SERVOS] \"min_pulse_width_in_microseconds\" of " << temp_min_pulse_width << " is not in the valid range of [0,65535]. Using default value instead.");
	}
	// > For the max pulse width parameter:
	int temp_max_pulse_width;
	if ( !nodeHandle.getParam("servo_driver_max_pulse_width_in_microseconds", temp_max_pulse_width) ) {
		ROS_WARN("[I2C FOR SERVOS] FAILED to get \"max_pulse_width_in_microseconds\" parameter. Using default value instead.");
	}
	else {
		if (0 <= temp_max_pulse_width && temp_max_pulse_width <= 65535)
			m_max_pulse_width_in_us = static_cast<uint16_t>(temp_max_pulse_width);
		else
			ROS_WARN_STREAM("[I2C FOR SERVOS] \"max_pulse_width_in_microseconds\" of " << temp_max_pulse_width << " is not in the valid range of [0,65535]. Using default value instead.");
	}

	// Initialise a subscriber for the servo driver
	ros::Subscriber set_servo_pulse_width_subscriber = nh_for_group.subscribe("set_servo_pulse_width", 1, servoSubscriberCallback);

	// Display command line command for publishing a
	// servo pulse width request
	if (m_servo_driver_verbosity >= 1)
		ROS_INFO_STREAM("[I2C FOR SERVOS] publish servo requests from command line with: rostopic pub --once " << ros::this_node::getNamespace() << "/set_servo_pulse_width asclinic_pkg/ServoPulseWidth \"{channel: 15, pulse_width_in_microseconds: 1100}\"");

	// Open the I2C device
	// > Note that the I2C driver is already instantiated
	//   as a member variable of this node
	bool open_success = m_i2c_driver.open_i2c_device();

	// Display the status
	if (!open_success)
		ROS_WARN_STREAM("[I2C FOR SERVOS] FAILED to open I2C device named " << m_i2c_driver.get_device_name());
	else {
		if (m_servo_driver_verbosity >= 1)
			ROS_INFO_STREAM("[I2C FOR SERVOS] Successfully opened named " << m_i2c_driver.get_device_name() << ", with file descriptor = " << m_i2c_driver.get_file_descriptor());
	}



	// NOTE:
	// > The servo driver is already instantiated as
	//   member variables of this node.



	// SET THE CONFIGURATION OF THE SERVO DRIVER

	// Specify the frequency of the servo driver
	float new_frequency_in_hz = m_pwm_frequency_in_hz;

	// Check if a device exists at the address
	bool servo_driver_is_connected = m_i2c_driver.check_for_device_at_address(m_pca9685_servo_driver.get_i2c_address());

	if (servo_driver_is_connected)
	{
		// Call the Servo Driver initialisation function
		bool verbose_display_for_servo_driver_init = false;
		bool result_servo_init = m_pca9685_servo_driver.initialise_with_frequency_in_hz(new_frequency_in_hz, verbose_display_for_servo_driver_init);

		// Display if an error occurred
		if (!result_servo_init)
			ROS_WARN_STREAM("[I2C FOR SERVOS] FAILED - while initialising servo driver with I2C address " << static_cast<int>(m_pca9685_servo_driver.get_i2c_address()) );
	}
	else
	{
		// Display that the device is not connected
		ROS_WARN_STREAM("[I2C FOR SERVOS] FAILED - Servo driver device NOT detected at I2C address " << static_cast<int>(m_pca9685_servo_driver.get_i2c_address()) );
	}

	if (m_servo_driver_verbosity >= 1)
		ROS_INFO("[I2C FOR SERVOS] Node initialisation complete");

	// Spin as a single-threaded node
	ros::spin();

	// Close the I2C device
	bool close_success = m_i2c_driver.close_i2c_device();
	
	// Display the status
	if (!close_success)
	{
		ROS_WARN_STREAM("[I2C FOR SERVOS] FAILED to close I2C device named " << m_i2c_driver.get_device_name());
	}
	else
	{
		ROS_INFO_STREAM("[I2C FOR SERVOS] Successfully closed device named " << m_i2c_driver.get_device_name());
	}

	return 0;
}
