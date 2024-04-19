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
// Node for I2C bus with time-of-flight distance sensors connected
//
// ----------------------------------------------------------------------------





#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/UInt16.h"

#include "i2c_driver/i2c_driver.h"

#include "vl53l1x/vl53l1x.h"


// Respond to subscriber receiving a message
void tofDistanceDataCallback(const std_msgs::UInt16& msg)
{
	ROS_INFO_STREAM("[I2C FOR SENSORS] Message received with data = " << msg.data);
}

int main(int argc, char* argv[])
{
	// Initialise the node
	ros::init(argc, argv, "i2c_for_sensors");
	ros::NodeHandle nodeHandle("~");
	// Initialise a publisher
	ros::Publisher tof_distance_publisher = nodeHandle.advertise<std_msgs::UInt16>("tof_distance", 10, false);
	// Initialise a subscriber
	ros::Subscriber tof_distance_subscriber = nodeHandle.subscribe("tof_distance", 1, tofDistanceDataCallback);

	// Initialise a variable with loop rate for
	// polling the sensors
	// > Input argument is the frequency in hertz, as a double
	ros::Rate loop_rate(0.75);


	// Specify the name of the I2C interface
	const char * i2c_device_name = "/dev/i2c-8";

	// Instantiate an "i2c_driver" object
	I2C_Driver i2c_driver (i2c_device_name);


	// Open the I2C device
	bool open_success = i2c_driver.open_i2c_device();

	// Display the status
	if (!open_success)
	{
		ROS_INFO_STREAM("[I2C FOR SENSORS] FAILED to open I2C device named " << i2c_driver.get_device_name());
	}
	else
	{
		ROS_INFO_STREAM("[I2C FOR SENSORS] Successfully opened named " << i2c_driver.get_device_name() << ", with file descriptor = " << i2c_driver.get_file_descriptor());
	}

	// Initialise an object for the VL53L1X distance sensor
	// > If connected directly to the I2C bus
	const uint8_t vl53l1x_address = 0x29;
	VL53L1X vl53l1x_object (&i2c_driver, vl53l1x_address);
	// > If connected to the I2C multiplexer
	//const uint8_t vl53l1x_address = 0x29;
	//const uint8_t mux_channel     = 0;
	//const uint8_t mux_i2c_address = 0x70;
	//VL53L1X vl53l1x_object_on_mux_ch0 (&i2c_driver, vl53l1x_address, mux_channel, mux_i2c_address);

	// Specify the distancing specifications
	// > Distance Mode (1 = short distance, 2 = long distance)
	uint16_t distance_mode = 2;

	// Initialise the VL53L1X distance sensor
	bool is_available_vl53l1x = vl53l1x_object.initialise_and_start_ranging(distance_mode);

	if (!is_available_vl53l1x)
	{
		ROS_INFO("[I2C FOR SENSORS] FAILED to initialised the VL53L1X distance sensor. Sensor is NOT available for usage.");
	}


	// Enter a loop that continues while ROS is still running
	while (ros::ok())
	{
		// Read data from the VL53L1X distance sensor
		VL53L1X_Result_t tof_res;
		bool success_get_distance = vl53l1x_object.get_distance_measurement(&tof_res);

		// If a result was succefully retrieved:
		if (success_get_distance)
		{
			// If the result status is good:
			if (tof_res.Status == 0)
			{
				// Then publish the distance measured
				std_msgs::UInt16 msg;
				msg.data = tof_res.Distance;
				tof_distance_publisher.publish(msg);
			}
			else
			{
				// Otherwise display the error status
				uint16_t temp_status = tof_res.Status;
				ROS_INFO_STREAM("[I2C FOR SENSORS] VL53L1X \"get_distance_measurement\" returned with an error status, status = " << temp_status << ", distance = " << tof_res.Distance << ", ambient = " << tof_res.Ambient << ", signal per SPAD = " << tof_res.SigPerSPAD << ", # of SPADs = " << tof_res.NumSPADs);
			}
		}
		else
		{
			// Otherwise display the error
			ROS_INFO("[I2C FOR SENSORS] FAILED to \"get_distance_measurement\" from VL53L1X distance sensor.");
		}


		// Spin once so that this node can service any
		// callbacks that this node has queued.
		ros::spinOnce();

		// Sleep for the specified loop rate
		loop_rate.sleep();
	} // END OF: "while (ros::ok())"

	// Close the I2C device
	bool close_success = i2c_driver.close_i2c_device();
	
	// Display the status
	if (!close_success)
	{
		ROS_INFO_STREAM("[I2C FOR SENSORS] FAILED to close I2C device named " << i2c_driver.get_device_name());
	}
	else
	{
		ROS_INFO_STREAM("[I2C FOR SENSORS] Successfully closed device named " << i2c_driver.get_device_name());
	}

	return 0;
}
