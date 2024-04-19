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


	// Initialise two objects
	const uint8_t vl53l1x_address = 0x29;
	const uint8_t mux_channel_a    = 4;
	const uint8_t mux_i2c_address = 0x70;
	VL53L1X vl53l1x_object_a (&i2c_driver, vl53l1x_address, mux_channel_a, mux_i2c_address);

	const uint8_t mux_channel_b    = 6;
	VL53L1X vl53l1x_object_b (&i2c_driver, vl53l1x_address, mux_channel_b, mux_i2c_address);


	// Initialise a boolean for the result of each call
	bool result;


	printf("\n\n");
	printf("===================================\n");
	printf("CALL SENSOR INIT AND START FUNCTION\n");

	// Specify the Distance Mode
	// > 1 = short distance
	// > 2 = long distance
	uint16_t distance_mode = 2;

	result = vl53l1x_object_a.initialise_and_start_ranging(distance_mode);
	if (result)
	{
		printf("VL53L1X - sensor initialise and ranging started for mux channel %d, for I2C address %d\n", vl53l1x_object_a.get_mux_channel(), vl53l1x_object_a.get_i2c_address() );
	}
	else
	{
		printf("FAILED - VL53L1X - initialise_and_start_ranging NOT successful for I2C address %d\n", vl53l1x_object_a.get_i2c_address() );
	}

	result = vl53l1x_object_b.initialise_and_start_ranging(distance_mode);
	if (result)
	{
		printf("VL53L1X - sensor initialise and ranging started for mux channel %d, for I2C address %d\n", vl53l1x_object_b.get_mux_channel(), vl53l1x_object_b.get_i2c_address() );
	}
	else
	{
		printf("FAILED - VL53L1X - initialise_and_start_ranging NOT successful for I2C address %d\n", vl53l1x_object_b.get_i2c_address() );
	}


	// Sleep for specified micro seconds
	usleep(10000);


	printf("\n\n");
	printf("=============\n");
	printf("POLL FOR DATA\n");

	// Initialise a pointer for iterating through
	// the Pololu SMC objects
	VL53L1X * vl53l1x_pointer;

	for (int i_data=0; i_data<10; i_data++)
	{
		for (int i_sensor=0; i_sensor<2; i_sensor++)
		{
			if (i_sensor==0)
			{
				vl53l1x_pointer = &vl53l1x_object_a;
				printf("Channel A:");
			}
			else
			{
				vl53l1x_pointer = &vl53l1x_object_b;
				printf("Channel B:");
			}

			// Read data from the VL53L1X distance sensor
			VL53L1X_Result_t tof_res;
			bool success_get_distance = vl53l1x_pointer->get_distance_measurement(&tof_res);

			// If a result was succefully retrieved:
			if (success_get_distance)
			{
				// If the result status is good:
				if (tof_res.Status == 0)
				{
					// Print out the result
					printf("Get resutls returned: status = %2d, dist = %5d, Ambient = %2d, Signal = %5d, #ofSpads = %5d\n",
						tof_res.Status, tof_res.Distance, tof_res.Ambient, tof_res.SigPerSPAD, tof_res.NumSPADs);
				}
				else
				{
					// Otherwise display the error status
					uint16_t temp_status = tof_res.Status;
					printf("\"get_distance_measurement\" returned with an error status, status = %2d, dist = %5d, Ambient = %2d, Signal = %5d, #ofSpads = %5d\n",
						tof_res.Status, tof_res.Distance, tof_res.Ambient, tof_res.SigPerSPAD, tof_res.NumSPADs);
				}
			}
			else
			{
				// Otherwise display the error
				printf("FAILED to \"get_distance_measurement\" from VL53L1X distance sensor.");
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
