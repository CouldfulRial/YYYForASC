#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include <iostream>
#include <bitset>

#include "i2c_driver/i2c_driver.h"
#include "vl53l5cx/vl53l5cx.h"

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


	// Initialise one VL53L3CX object
	const uint8_t vl53l5cx_address = 0x29;
	const uint8_t mux_channel_a    = 0;
	const uint8_t mux_i2c_address  = 0x70;
	VL53L5CX vl53l5cx_object_a (&i2c_driver, vl53l5cx_address, mux_channel_a, mux_i2c_address);


	// Initialise a boolean for the result of each call
	bool result;


	printf("\n\n");
	printf("===================================\n");
	printf("CALL SENSOR INIT AND START RANGING\n");

	// Specify the details for sensor configuration:
	// > Resolution:
	//   VL53L5CX_RESOLUTION_4X4 = 16
	//   VL53L5CX_RESOLUTION_8X8 = 64
	uint8_t resolution = VL53L5CX_RESOLUTION_4X4;
	// > Ranging frequency:
	//   For 4x4, allowed values are: [1;60]
 	//   For 8x8, allowed values are: [1;15]
	uint8_t ranging_frequency_hz = 10;
	// > Integration time
	//   Allowed values are: [2;1000] ms
	//   Only used for the "Autonomous"ranging mode
	uint8_t integration_time_ms = 50;
	// > Sharpener percent:
	//   Allowed values are: [0;99] %
	//   0% means disabled, 5% is the default
	uint8_t sharpener_percent = 20;
	// > Target order:
	//   VL53L5CX_TARGET_ORDER_STRONGEST = 2 <default>
	//   VL53L5CX_TARGET_ORDER_CLOSEST   = 1
	uint8_t target_order = VL53L5CX_TARGET_ORDER_CLOSEST;
	// > Ranging mode:
	//   VL53L5CX_RANGING_MODE_CONTINUOUS = 1
	//   VL53L5CX_RANGING_MODE_AUTONOMOUS = 3 <default>
	uint8_t ranging_mode = VL53L5CX_RANGING_MODE_CONTINUOUS;

	result = vl53l5cx_object_a.initialise_and_start_ranging(resolution, ranging_frequency_hz, integration_time_ms, sharpener_percent, target_order, ranging_mode);
	if (result)
	{
		printf("VL53L5CX - sensor initialised and ranging started for mux channel %d, for I2C address %d\n", vl53l5cx_object_a.get_mux_channel(), vl53l5cx_object_a.get_i2c_address() );
	}
	else
	{
		printf("FAILED - VL53L5CX - initialise_and_start_ranging NOT successful for I2C address %d\n", vl53l5cx_object_a.get_i2c_address() );
	}


	// Sleep for specified micro seconds
	usleep(10000);


	printf("\n\n");
	printf("=============\n");
	printf("POLL FOR DATA\n");

	printf("For completeness, the target status interpretation is as follows");
	printf("| Code | Description\n"); 
	printf("|   0  | Ranging data are not updated\n");
	printf("|   1  | Signal rate too low on SPAD array\n");
	printf("|   2  | Target phase\n");
	printf("|   3  | Sigma estimator too high\n");
	printf("|   4  | Target consistency failed\n");
	printf("|   5  | Range valid\n");
	printf("|   6  | Wrap around not performed (Typically the first range)\n");
	printf("|   7  | Rate consistency failed\n");
	printf("|   8  | Signal rate too low for the current target\n");
	printf("|   9  | Range valid with large pulse (may be due to a merged target)\n");
	printf("|  10  | Range valid, but no target detected at previous range\n");
	printf("|  11  | Measurement consistency failed\n");
	printf("|  12  | Target blurred by another one, due to sharpener\n");
	printf("|  13  | Target detected but inconsistent data. Frequently happens for secondary targets.)\n");
	printf("| 255  | No target detected (only if number of target detected is enabled)\n");


	while (true)
	{
		//printf("Press \"m\" to get a ranging, and any other key to stop");


		// Read data from the VL53L5CX distance sensor
		VL53L5CX_ResultsData tof_result;
		bool success_get_ranging_data = vl53l5cx_object_a.get_ranging_data(&tof_result);

		// If a result was successfully retrieved:
		if (success_get_ranging_data)
		{
			printf("Get results was successful.");
			printf("\"silicon_temp_degc\" = %5d\n", tof_result.silicon_temp_degc);

			printf("\"target_status\" and \"distance_mm\" =\n");
			for (int i_row=0; i_row<4; i_row++)
			{
				for (int i_col=0; i_col<4; i_col++)
					printf("%4d", tof_result.target_status[i_row*4+i_col]);
				printf("    ");
				for (int i_col=0; i_col<4; i_col++)
					printf("%5d", tof_result.distance_mm[i_row*4+i_col]);
				printf("\n");
			}
		}
		else
		{
			// Otherwise display the error
			printf("FAILED to \"get_ranging_data\" from VL53L5CX distance sensor.\n");
		}

		// Sleep for  bit before checking again
		usleep(100000);
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
