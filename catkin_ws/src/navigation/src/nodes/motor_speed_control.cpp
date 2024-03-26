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
// Node for I2C bus with Pololu SMC devices connected
//
// ----------------------------------------------------------------------------





#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"

#include "i2c_driver/i2c_driver.h"

#include "pololu_smc_g2/pololu_smc_g2.h"

#include <bitset>

// Include the asclinic message types
#include "navigation/LRFloat32.h"

// Include the asclinic constants
//#include "nodes/constant.h"

// Namespacing the package
//using namespace navigation;





// MEMBER VARIABLES THAT ARE PARAMETERS FOR THIS NODE:
// > For the verbosity level of displaying info
//   Note: the levels of increasing verbosity are defined as:
//   0 : Info is not displayed. Warnings and errors are still displayed
//   1 : Startup info is displayed
//   2 : Info about messages received is displayed
int m_motor_driver_verbosity = 2;

// Settings for the motor drivers
int   m_current_limit_in_milliamps      = 5000;
float m_max_duty_cycle_limit_in_percent = 100.0;
float m_max_accel_limit_in_percent      = 0.04;
float m_max_decel_limit_in_percent      = 0.16;

// Multiplier for adjusting the direction of each motor
float m_left_side_multiplier  = 1.0;
float m_right_side_multiplier = 1.0;



// ALL OTHER MEMBER VARIABLES FOR THIS NODE:
// > For the I2C driver
const char * m_i2c_device_name = "/dev/i2c-1";
I2C_Driver m_i2c_driver (m_i2c_device_name);

// > For the Pololu Simple Motor Controller (SMC) driver
const uint8_t m_pololu_smc_address_left = 13;
Pololu_SMC_G2 m_pololu_smc_left (&m_i2c_driver, m_pololu_smc_address_left);

const uint8_t m_pololu_smc_address_right = 14;
Pololu_SMC_G2 m_pololu_smc_right (&m_i2c_driver, m_pololu_smc_address_right);

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
//        rostopic pub --once <namespace>/set_motor_duty_cycle navigation/LRFloat32 "{left: 10.1, right: 10.1}"
//      where "<namespace>/set_motor_duty_cycle" is the full
//      name identified in step 1.
//
void driveMotorsSubscriberCallback(const navigation::LRFloat32& msg)
{
	// Sequnce number for tracking the order of published messages
	static unsigned long int s_sequence_number_for_msgs = 1;

	// Display the values from the message received
	if (m_motor_driver_verbosity >= 2)
		ROS_INFO_STREAM("[I2C FOR MOTORS] Message received with left = " << msg.left << ", right = " << msg.right);

	// Clip the data to be in the range [-100.0,100.0]
	// > For the left value
	float pwm_duty_cycle_left = msg.left * m_left_side_multiplier;
	if (pwm_duty_cycle_left < -100.0f)
		pwm_duty_cycle_left = -100.0f;
	if (pwm_duty_cycle_left > 100.0f)
		pwm_duty_cycle_left = 100.0f;
	// > For the right value
	float pwm_duty_cycle_right = msg.right * m_right_side_multiplier;
	if (pwm_duty_cycle_right < -100.0f)
		pwm_duty_cycle_right = -100.0f;
	if (pwm_duty_cycle_right > 100.0f)
		pwm_duty_cycle_right = 100.0f;

	// Initialise one boolean variable for the result
	// of all calls to Pololu_SMC_G2 functions
	bool result;

	// SET THE TARGET DUTY CYCLE FOR EACH MOTOR DRIVER
	// > NOTE: it may be necessary to negate one or both
	//   of the duty cycles depending on the polarity
	//   with which each motor is plugged in

	// > Set the LEFT motor driver
	result = m_pololu_smc_left.set_motor_target_duty_cycle_percent(pwm_duty_cycle_left);
	if (!result)
		ROS_WARN_STREAM("[I2C FOR MOTORS] FAILED - Pololu SMC - set motor percent NOT successful for I2C address " << static_cast<int>(m_pololu_smc_left.get_i2c_address()) );

	// > Set the RIGHT motor driver
	result = m_pololu_smc_right.set_motor_target_duty_cycle_percent(-pwm_duty_cycle_right);
	if (!result)
		ROS_WARN_STREAM("[I2C FOR MOTORS] FAILED - Pololu SMC - set motor percent NOT successful for I2C address " << static_cast<int>(m_pololu_smc_right.get_i2c_address()) );

	// Publish the motor duty cycles
	navigation::LRFloat32 msg_current_duty_cycle;
	msg_current_duty_cycle.left  = pwm_duty_cycle_left;
	msg_current_duty_cycle.right = pwm_duty_cycle_right;
	msg_current_duty_cycle.seq_num = s_sequence_number_for_msgs;
	m_current_motor_duty_cycle_publisher.publish(msg_current_duty_cycle);

	// Increment the sequence number
	s_sequence_number_for_msgs++;
}



int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "i2c_for_motors");
	ros::NodeHandle nodeHandle("~");

	// Initialise a node handle to the group namespace
	std::string ns_for_group = ros::this_node::getNamespace();
	ros::NodeHandle nh_for_group(ns_for_group);

	// Display that this node is launcing
	ROS_INFO_STREAM("[I2C FOR MOTORS] Now launching this node in namespace: " << ns_for_group);

	// Get the parameter values:
	// > For the verbosity
	if ( !nodeHandle.getParam("motor_driver_verbosity", m_motor_driver_verbosity) ) {
		ROS_WARN("[I2C FOR MOTORS] FAILED to get \"motor_driver_verbosity\" parameter. Using default value instead.");
	}
	// > For the current limit parameter:
	if ( !nodeHandle.getParam("motor_driver_current_limit_in_milliamps", m_current_limit_in_milliamps) ) {
		ROS_WARN("[I2C FOR MOTORS] FAILED to get \"motor_driver_current_limit_in_milliamps\" parameter. Using default value instead.");
	}
	// > For the max duty cycle parameter:
	if ( !nodeHandle.getParam("motor_driver_max_duty_cycle_limit_in_percent", m_max_duty_cycle_limit_in_percent) ) {
		ROS_WARN("[I2C FOR MOTORS] FAILED to get \"motor_driver_max_duty_cycle_limit_in_percent\" parameter. Using default value instead.");
	}
	// > For the max acceleration parameter:
	if ( !nodeHandle.getParam("motor_driver_max_accel_limit_in_percent_per_millisecond", m_max_accel_limit_in_percent) ) {
		ROS_WARN("[I2C FOR MOTORS] FAILED to get \"motor_driver_max_accel_limit_in_percent\" parameter. Using default value instead.");
	}
	// > For the max deceleration parameter:
	if ( !nodeHandle.getParam("motor_driver_max_decel_limit_in_percent_per_millisecond", m_max_decel_limit_in_percent) ) {
		ROS_WARN("[I2C FOR MOTORS] FAILED to get \"motor_driver_max_decel_limit_in_percent\" parameter. Using default value instead.");
	}
	// > For the multiplier for the left-hand-side motor
	if ( !nodeHandle.getParam("motor_driver_left_side_multiplier", m_left_side_multiplier) ) {
		ROS_WARN("[I2C FOR MOTORS] FAILED to get \"motor_driver_left_side_multiplier\" parameter. Using default value instead.");
	}
	// > For the multiplier for the right-hand-side motor
	if ( !nodeHandle.getParam("motor_driver_right_side_multiplier", m_right_side_multiplier) ) {
		ROS_WARN("[I2C FOR MOTORS] FAILED to get \"motor_driver_right_side_multiplier\" parameter. Using default value instead.");
	}

	// Initialise a subscriber for the duty cycle of the main drive motors
	ros::Subscriber set_motor_duty_cycle_subscriber = nh_for_group.subscribe("set_motor_duty_cycle", 1, driveMotorsSubscriberCallback);

	// Initialise a publisher for the current duty cycle setting of the main drive motors
	m_current_motor_duty_cycle_publisher = nh_for_group.advertise<navigation::LRFloat32>("current_motor_duty_cycle", 10, true);

	// Display command line command for publishing a
	// motor duty cycle request
	if (m_motor_driver_verbosity >= 1)
		ROS_INFO_STREAM("[I2C FOR MOTORS] publish motor duty cycle requests from command line with: rostopic pub --once " << ros::this_node::getNamespace() << "/set_motor_duty_cycle navigation/LRFloat32 \"{left: 10.1, right: 10.1}\"");

	// Open the I2C device
	// > Note that the I2C driver is already instantiated
	//   as a member variable of this node
	bool open_success = m_i2c_driver.open_i2c_device();

	// Display the status
	if (!open_success)
		ROS_WARN_STREAM("[I2C FOR MOTORS] FAILED to open I2C device named " << m_i2c_driver.get_device_name());
	else {
		if (m_motor_driver_verbosity >= 1)
			ROS_INFO_STREAM("[I2C FOR MOTORS] Successfully opened named " << m_i2c_driver.get_device_name() << ", with file descriptor = " << m_i2c_driver.get_file_descriptor());
	}



	// NOTE:
	// > The drivers are already instantiated as
	//   member variables of this node for:
	//   > Each of the Pololu simple motor controllers (SMC)



	// SET THE CONFIGURATION OF EACH MOTOR DRIVER

	// Specify the various limits
	int new_current_limit_in_milliamps = m_current_limit_in_milliamps;
	int new_max_duty_cycle_limit = static_cast<int>(32.0 * m_max_duty_cycle_limit_in_percent);
	int new_max_accel_limit      = static_cast<int>(32.0 * m_max_accel_limit_in_percent);
	int new_max_decel_limit      = static_cast<int>(32.0 * m_max_decel_limit_in_percent);

	// Clip the limits to an appropriate range
	// > For the maximum motor current
	if (new_current_limit_in_milliamps < 0) {
		new_current_limit_in_milliamps = 0;
		ROS_WARN("[I2C FOR MOTORS] Current limit increased to 0 milliamps.");
	}

	// > For the maximum duty cycle
	if (new_max_duty_cycle_limit < 0) {
		new_max_duty_cycle_limit = 0;
		ROS_WARN("[I2C FOR MOTORS] Max duty cycle limit increased to 0 percent.");
	}
	if (new_max_duty_cycle_limit > 3200) {
		new_max_duty_cycle_limit = 3200;
		ROS_WARN("[I2C FOR MOTORS] Max duty cycle limit reduced to 100 percent.");
	}

	// > For the maximum duty cycle accerlation (away from 0) per millisecond
	if (new_max_accel_limit < 0) {
		new_max_accel_limit = 0;
		ROS_WARN("[I2C FOR MOTORS] Max acceleration limit increased to 0 percent per millisecond.");
	}
	if (new_max_accel_limit > 3200) {
		new_max_accel_limit = 3200;
		ROS_WARN("[I2C FOR MOTORS] Max acceleration limit reduced to 100 percent per millisecond.");
	}

	// > For the maximum duty cycle decerlation (towards 0) per millisecond
	if (new_max_decel_limit < 0) {
		new_max_decel_limit = 0;
		ROS_WARN("[I2C FOR MOTORS] Max deceleration limit increased to 0 percent per millisecond.");
	}
	if (new_max_decel_limit > 3200) {
		new_max_decel_limit = 3200;
		ROS_WARN("[I2C FOR MOTORS] Max deceleration limit reduced to 100 percent per millisecond.");
	}

	// Display the actual acceleration and deceleration limits
	if (m_motor_driver_verbosity >= 1) {
		ROS_INFO_STREAM("[I2C FOR MOTORS] Actual acceleration limit = " << (static_cast<float>(new_max_accel_limit)/3200.0) << "%/ms (after conversation to \"internal units\")");
		ROS_INFO_STREAM("[I2C FOR MOTORS] Actual deceleration limit = " << (static_cast<float>(new_max_decel_limit)/3200.0) << "%/ms (after conversation to \"internal units\")");
	}

	// Initialise a pointer for iterating through
	// the Pololu SMC objects
	Pololu_SMC_G2 * pololu_smc_pointer;

	// Initialise each of the Pololu SMC objects
	// with the limits specified above.

	// Iterate over the pololu objects
	for (int i_smc=0;i_smc<2;i_smc++)
	{
		// Point to the appropriate motor driver
		if (i_smc==0)
			pololu_smc_pointer = &m_pololu_smc_left;
		else
			pololu_smc_pointer = &m_pololu_smc_right;

		// Display the object about to be initialised
		if (m_motor_driver_verbosity >= 1)
			ROS_INFO_STREAM("[I2C FOR MOTORS] Now initialising SMC with I2C address " << static_cast<int>(pololu_smc_pointer->get_i2c_address()) );

		// Check if a device exists at the address
		bool this_pololu_smc_is_connected = m_i2c_driver.check_for_device_at_address(pololu_smc_pointer->get_i2c_address());

		if (this_pololu_smc_is_connected)
		{
			// Call the Pololu SMC initialisation function
			bool verbose_display_for_SMC_init = false;
			bool result_smc_init = pololu_smc_pointer->initialise_with_limits(new_current_limit_in_milliamps,new_max_duty_cycle_limit,new_max_accel_limit,new_max_decel_limit,verbose_display_for_SMC_init);

			// Display if an error occurred
			if (!result_smc_init)
				ROS_WARN_STREAM("[I2C FOR MOTORS] FAILED - while initialising SMC with I2C address " << static_cast<int>(pololu_smc_pointer->get_i2c_address()) );
		}
		else
		{
			// Display that the device is not connected
			ROS_WARN_STREAM("[I2C FOR MOTORS] FAILED - SMC device NOT detected at I2C address " << static_cast<int>(pololu_smc_pointer->get_i2c_address()) );
		}
	}

	if (m_motor_driver_verbosity >= 1)
		ROS_INFO("[I2C FOR MOTORS] Node initialisation complete");

	// Spin as a single-threaded node
	ros::spin();

	// Close the I2C device
	bool close_success = m_i2c_driver.close_i2c_device();
	
	// Display the status
	if (!close_success)
	{
		ROS_WARN_STREAM("[I2C FOR MOTORS] FAILED to close I2C device named " << m_i2c_driver.get_device_name());
	}
	else
	{
		ROS_INFO_STREAM("[I2C FOR MOTORS] Successfully closed device named " << m_i2c_driver.get_device_name());
	}

	return 0;
}
