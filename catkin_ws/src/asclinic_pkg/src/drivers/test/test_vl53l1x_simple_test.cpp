#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include <iostream>
#include <bitset>

#include "i2c_driver/i2c_driver.h"
#include "vl53l1x/vl53l1x.h"

int main()
{
	// Choose the I2C device.
	const char * i2c_device_name = "/dev/i2c-8";

	// Initialise a driver for the I2C device
	I2C_Driver i2c_driver (i2c_device_name);	

	printf("Now opening i2c device with name = %s\n", i2c_driver.get_device_name() );

	bool openSuccess = i2c_driver.open_i2c_device();
	if (!openSuccess)
	{
		printf("FAILED to open I2C device.\n" );
		return 1;
	}
	else
	{
		printf("I2C Device successfully opened with file descriptor = %d\n", i2c_driver.get_file_descriptor() );
	}


	// Initialise an object for the INA260 current sensor
	const uint8_t vl53l1x_address = 0x29;
	VL53L1X vl53l1x_object (&i2c_driver, vl53l1x_address);


	// Initialise a boolean for the result of each call
	bool result;


	printf("\n\n");
	printf("=========================\n");
	printf("CALL SENSOR INIT FUNCTION\n");


	// > Get the Manufacturer ID
	uint16_t manufacturer_id;
	result = vl53l1x_object.sensor_init();
	if (result)
	{
		printf("VL53L1X - sensor_init: %d, for I2C address %d\n", result, vl53l1x_object.get_i2c_address() );
	}
	else
	{
		printf("FAILED - VL53L1X - sensor_init NOT successful for I2C address %d\n", vl53l1x_object.get_i2c_address() );
	}


	// Sleep for specified micro seconds
	usleep(10000);


	printf("\n\n");
	printf("=======================\n");
	printf("READ MODULE ID AND TYPE\n");

	bool bool_status;
	uint8_t byteData, sensorState = 0;
	uint16_t wordData;
	VL53L1X_Result_t tof_res;
	uint8_t first_range = 1;

	// For the model ID
	bool_status = vl53l1x_object.read_byte(0x010F, &byteData);
	if (bool_status)
		printf("VL53L1X Model_ID: %X\n", byteData);
	else
		printf("VL53L1X Model_ID: error occurred\n");

	// For the module type
	bool_status = vl53l1x_object.read_byte(0x0110, &byteData);
	if (bool_status)
		printf("VL53L1X Module_Type: %X\n", byteData);
	else
		printf("VL53L1X Module_Type: error occurred\n");

	// For the model ID as a word
	bool_status = vl53l1x_object.read_word(0x010F, &wordData);
	if (bool_status)
		printf("VL53L1X Model_ID word: %X\n", wordData);
	else
		printf("VL53L1X Model_ID word: error occurred\n");



	printf("\n\n");
	printf("========================\n");
	printf("SET THE DISTANCING SPECS\n");

	// Set the Distance Mode
	// > 1 = short distance
	// > 2 = long distance
	uint16_t distance_mode = 2;
	bool_status = vl53l1x_object.set_distance_mode(distance_mode);
	if (bool_status)
		printf("VL53L1X distance mode set to: %d\n", distance_mode);
	else
		printf("VL53L1X distance mode set to: error occurred\n");


	//status += VL53L1X_SetTimingBudgetInMs(Dev, 100);
	//status += VL53L1X_SetInterMeasurementInMs(Dev, 100);


	printf("\n\n");
	printf("=============\n");
	printf("START RANGING\n");

	bool_status = vl53l1x_object.start_ranging();
	if (bool_status)
		printf("VL53L1X started ranging\n");
	else
		printf("VL53L1X request to start ranging returned that an error occurred\n");



	printf("\n\n");
	printf("=============\n");
	printf("POLL FOR DATA\n");

	for (int i_data=0; i_data<10; i_data++)
	{
		// Wait until data is ready
		uint8_t is_data_ready = 0;
		while (is_data_ready == 0) {
			bool_status = vl53l1x_object.check_for_data_ready(&is_data_ready);
			usleep(1);
		}


		// Get the data
		bool_status = vl53l1x_object.get_result(&tof_res);
		if (bool_status)
		{
			printf("Get resutls returned: status = %2d, dist = %5d, Ambient = %2d, Signal = %5d, #ofSpads = %5d\n",
				tof_res.Status, tof_res.Distance, tof_res.Ambient, tof_res.SigPerSPAD, tof_res.NumSPADs);
		}
		else
		{
			printf("ERROR OCCURRED during call to VL53L1X Get Result\n");
		}

		// Trigger the next ranging
		bool_status = vl53l1x_object.clear_interrupt();
		if (bool_status)
			printf("VL53L1X interrupt cleared successfully\n");
		else
			printf("VL53L1X interrupt clear error occurred\n");


		// The very first measurement shall be ignored,
		// thus requires clearing the interrupt twice.
		if (first_range)
		{
			bool_status = vl53l1x_object.clear_interrupt();
			if (bool_status)
			{
				printf("VL53L1X interrupt cleared successfully\n");
				first_range = 0;
			}
			else
			{
				printf("VL53L1X interrupt clear error occurred\n");
			}
		}


		// Sleep for  bit before checking again
		usleep(1000000);
	}


	// Close the I2C device
	bool closeSuccess = i2c_driver.close_i2c_device();
	if (!closeSuccess)
	{
		printf("FAILED to close I2C device.\n" );
		return 1;
	}
	else
	{
		printf("I2C Device successfully closed.\n" );
	}

	// Return
	return 0;
}
