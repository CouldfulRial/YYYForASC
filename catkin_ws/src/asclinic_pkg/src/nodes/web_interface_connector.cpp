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
// Template node for I2C devices connected inside the robot
//
// ----------------------------------------------------------------------------





#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/Bool.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"

#include <bitset>

// Include the asclinic message types
#include "asclinic_pkg/LeftRightFloat32.h"

// Namespacing the package
//using namespace asclinic_pkg;



// MEMBER VARIABLES FOR THIS NODE:
// Publisher for motor duty cycle
// > To the "I2C for motors and servos" node
ros::Publisher m_set_motor_duty_cycle_publisher;
// > To the web interface
ros::Publisher m_current_motor_duty_cycles_publisher;

// Variables for the current target duty cycle of each motor
float m_current_duty_cycle_left = 0.0;
float m_current_duty_cycle_right = 0.0;

// Timer that when triggered determines that a
// heartbeat was missed
ros::Timer m_timer_web_interface_heartbeat_timeout_check;
float m_timer_web_interface_heartbeat_duration_in_seconds = 2.0;



void currentMotorDutyCycleFromNodeSubscriberCallback(const asclinic_pkg::LeftRightFloat32& msg)
{
	// Update the member variables
	m_current_duty_cycle_left  = msg.left;
	m_current_duty_cycle_right = msg.right;

	// Publish the values
	geometry_msgs::Vector3 msg_for_web_interface;
	msg_for_web_interface.x = m_current_duty_cycle_left;
	msg_for_web_interface.y = m_current_duty_cycle_right;
	m_current_motor_duty_cycles_publisher.publish(msg_for_web_interface);
}

void publishMotorDutyCycleRequest(float duty_cycle_left, float duty_cycle_right)
{
	asclinic_pkg::LeftRightFloat32 msg;
	msg.left  = duty_cycle_left;
	msg.right = duty_cycle_right;
	m_set_motor_duty_cycle_publisher.publish(msg);
}


void setMotorDutyCycleFromWebInterfaceSubscriberCallback(const geometry_msgs::Vector3& msg)
{
	ROS_INFO_STREAM("[WEB INTERFACE CONNECTOR] Set motor duty cycle callback received message: .(x,y) = ( " << msg.x << " , " << msg.y << " )");

	// Restart the heartbeat timer
	// > Stop any previous instance that might still be running
	m_timer_web_interface_heartbeat_timeout_check.stop();
	// > Set the period again (second argument is reset)
	m_timer_web_interface_heartbeat_timeout_check.setPeriod( ros::Duration(m_timer_web_interface_heartbeat_duration_in_seconds), true);
	// > Start the timer again
	m_timer_web_interface_heartbeat_timeout_check.start();

	// Publish the message to the respective node
	publishMotorDutyCycleRequest( msg.x , msg.y );
}


void incrementMotorDutyCycleFromWebInterfaceSubscriberCallback(const geometry_msgs::Vector3& msg)
{
	ROS_INFO_STREAM("[WEB INTERFACE CONNECTOR] Increment motor duty cycle callback received message: .(x,y) = ( " << msg.x << " , " << msg.y << " )");
	
	// Restart the heartbeat timer
	// > Stop any previous instance that might still be running
	m_timer_web_interface_heartbeat_timeout_check.stop();
	// > Set the period again (second argument is reset)
	m_timer_web_interface_heartbeat_timeout_check.setPeriod( ros::Duration(m_timer_web_interface_heartbeat_duration_in_seconds), true);
	// > Start the timer again
	m_timer_web_interface_heartbeat_timeout_check.start();

	// Add the message data to the current speed
	float new_duty_cycle_left  = msg.x + m_current_duty_cycle_left;
	float new_duty_cycle_right = msg.y + m_current_duty_cycle_right;

	// Publish the message to the respective node
	publishMotorDutyCycleRequest( new_duty_cycle_left , new_duty_cycle_right );
}

void webInterfaceHeartbeatSubscriberCallback(const std_msgs::Bool& msg)
{
	// Restart the heartbeat timer
	// > Stop any previous instance that might still be running
	m_timer_web_interface_heartbeat_timeout_check.stop();
	// > Set the period again (second argument is reset)
	m_timer_web_interface_heartbeat_timeout_check.setPeriod( ros::Duration(m_timer_web_interface_heartbeat_duration_in_seconds), true);
	// > Start the timer again
	m_timer_web_interface_heartbeat_timeout_check.start();
}

void timerCallbackHeartbeatTimeoutCheck(const ros::TimerEvent&)
{
	ROS_INFO_STREAM("[WEB INTERFACE CONNECTOR] Web interface heartbeat timeout occurred. Turning motors off.");
	
	// Ensure that the timer is stopped
	m_timer_web_interface_heartbeat_timeout_check.stop();
	// Call the function to set the duty cycles to zero
	publishMotorDutyCycleRequest(0.0, 0.0);
}

int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "web_interface_connector");
	ros::NodeHandle nodeHandle("~");

	// Initialise a node handle to the group namespace
	std::string ns_for_group = ros::this_node::getNamespace();
	ros::NodeHandle nh_for_group(ns_for_group);

	// Initialise a subscriber for motor duty cycle
	// > Requests from the web interface
	ros::Subscriber set_motor_duty_cycle_subscriber = nh_for_group.subscribe("set_motor_duty_cycle_from_web_interface", 1, setMotorDutyCycleFromWebInterfaceSubscriberCallback);
	ros::Subscriber inc_motor_duty_cycle_subscriber = nh_for_group.subscribe("increment_motor_duty_cycle_from_web_interface", 10, incrementMotorDutyCycleFromWebInterfaceSubscriberCallback);
	// > Updates from the "I2C for motors and servos" node
	ros::Subscriber current_motor_duty_cycle_subscriber = nh_for_group.subscribe("current_motor_duty_cycle", 10, currentMotorDutyCycleFromNodeSubscriberCallback);

	// Initialise a publisher for motor duty cycle
	// > Requests to the "I2C for motors and servos" node
	m_set_motor_duty_cycle_publisher = nh_for_group.advertise<asclinic_pkg::LeftRightFloat32>("set_motor_duty_cycle", 1, false);
	// > Updates to the web interface
	m_current_motor_duty_cycles_publisher = nh_for_group.advertise<geometry_msgs::Vector3>("current_motor_duty_cycle_for_web_interface", 10, true);

	// Initialise a subscriber for the web interface heartbeart
	ros::Subscriber web_interface_heartbeat = nh_for_group.subscribe("web_interface_heartbeat",1,webInterfaceHeartbeatSubscriberCallback);

	// Initialise the timer for checking the heartbeat
	m_timer_web_interface_heartbeat_timeout_check = nodeHandle.createTimer(ros::Duration(m_timer_web_interface_heartbeat_duration_in_seconds), timerCallbackHeartbeatTimeoutCheck, true);
	m_timer_web_interface_heartbeat_timeout_check.stop();

	// Spin as a single-threaded node
	ros::spin();

	return 0;
}
