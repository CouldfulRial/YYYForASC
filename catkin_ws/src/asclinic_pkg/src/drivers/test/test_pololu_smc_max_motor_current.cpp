#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include <iostream>
#include <bitset>

#include "i2c_driver/i2c_driver.h"
#include "pololu_smc_g2/pololu_smc_g2.h"

int main()
{
	// Choose the I2C device.
	const char * i2c_device_name = "/dev/i2c-1";

	// Initialise a driver for the I2C device
	I2C_Driver i2c_driver (i2c_device_name);	

	printf("Now opening i2c device with name = %s\n", i2c_driver.get_device_name() );

	bool openSuccess = i2c_driver.open_i2c_device();
	if (!openSuccess)
	{
		printf("FAILED to open I2C device.\n\n\n" );
		return 1;
	}
	else
	{
		printf("I2C Device successfully opened with file descriptor = %d\n\n\n", i2c_driver.get_file_descriptor() );
	}


	// Initialise an object for each of the Pololu
	// simple motor controllers
	const uint8_t pololu_smc_address1 = 70;
	Pololu_SMC_G2 pololu_smc1 (&i2c_driver, pololu_smc_address1);
	const uint8_t pololu_smc_address2 = 13;
	Pololu_SMC_G2 pololu_smc2 (&i2c_driver, pololu_smc_address2);

	// Initialise a pointer for iterating through
	// the Pololu SMC objects
	Pololu_SMC_G2 * pololu_smc_pointer;

	// Initialise one boolean variable for the result
	// of all calls to Pololu_SMC_G2 functions
	bool result;





	// Iterate over the pololu objects to check
	// some status details
	for (int i_smc=0;i_smc<2;i_smc++)
	{
		if (i_smc==0)
		{
			pololu_smc_pointer = &pololu_smc1;
		}
		else if (i_smc==1)
		{
			pololu_smc_pointer = &pololu_smc2;
		}
		else
		{
			pololu_smc_pointer = &pololu_smc1;
		}

		printf("Now checking status for Pololu SMC with I2C address %d\n\n", pololu_smc_pointer->get_i2c_address() );

		// > Send the "exit safe start" command
		result = pololu_smc_pointer->exit_safe_start();
		if (result)
		{
			printf("Pololu SMC - exit safe start successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - exit safe start NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		// > Check the status flag registers
		uint16_t error_status;
		result = pololu_smc_pointer->get_error_status(&error_status);
		if (result)
		{
			//printf("Pololu SMC - get error status returned: %d, for I2C address %d\n", error_status, pololu_smc_pointer->get_i2c_address() );
			std::cout << "Pololu SMC - get error status returned: " << std::bitset<16>(error_status) << ", for I2C address " << pololu_smc_pointer->get_i2c_address() << "\n";
		}
		else
		{
			printf("FAILED - Pololu SMC - get error status NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		// > Check the input voltage
		float input_voltage_value;
		result = pololu_smc_pointer->get_input_voltage_in_volts(&input_voltage_value);
		if (result)
		{
			printf("Pololu SMC - get input voltage value returned: %f [Volts], for I2C address %d\n", input_voltage_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get input voltage value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}
		
		printf("\nFinished checking status for Pololu SMC with I2C address %d\n\n\n", pololu_smc_pointer->get_i2c_address() );

	}



	// SPECIFY THE VARIOUS LIMITS
	int new_current_limit_in_milliamps = 4000;
	int new_max_speed_limit = 2560;
	int new_max_accel_limit = 1;
	int new_max_decel_limit = 5;


	usleep(10000);
	// Iterate over the pololu objects to set
	// the current limit
	for (int i_smc=0;i_smc<2;i_smc++)
	{
		if (i_smc==0)
		{
			pololu_smc_pointer = &pololu_smc1;
		}
		else if (i_smc==1)
		{
			pololu_smc_pointer = &pololu_smc2;
		}
		else
		{
			pololu_smc_pointer = &pololu_smc1;
		}

		printf("Now setting the current limit for Pololu SMC with I2C address %d\n\n", pololu_smc_pointer->get_i2c_address() );

		// > Send the "exit safe start" command
		result = pololu_smc_pointer->set_current_limit_in_milliamps(new_current_limit_in_milliamps);
		if (result)
		{
			printf("Pololu SMC - set current limit successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - set current limit NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		// > Check the current limit that was set
		uint16_t current_limit_value;
		result = pololu_smc_pointer->get_current_limit(&current_limit_value);
		if (result)
		{
			printf("Pololu SMC - get current limit returned: %d, for I2C address %d\n", current_limit_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get current limit NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		printf("\nFinished setting and checking the current limit for Pololu SMC with I2C address %d\n\n\n", pololu_smc_pointer->get_i2c_address() );

	}
	usleep(10000);


	usleep(10000);
	// Iterate over the pololu objects to set
	// the max speed limit
	for (int i_smc=0;i_smc<2;i_smc++)
	{
		if (i_smc==0)
		{
			pololu_smc_pointer = &pololu_smc1;
		}
		else if (i_smc==1)
		{
			pololu_smc_pointer = &pololu_smc2;
		}
		else
		{
			pololu_smc_pointer = &pololu_smc1;
		}

		printf("Now setting the max speed limit for Pololu SMC with I2C address %d\n\n", pololu_smc_pointer->get_i2c_address() );

		// > Send the "exit safe start" command
		int max_speed_limit_response_code;
		result = pololu_smc_pointer->set_motor_limit_max_duty_cycle(new_max_speed_limit, &max_speed_limit_response_code);
		if (result)
		{
			printf("Pololu SMC - set max speed limit successful with response code %d, for I2C address %d\n", max_speed_limit_response_code, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - set max speed limit NOT successful with response code %d, for I2C address %d\n", max_speed_limit_response_code, pololu_smc_pointer->get_i2c_address() );
		}

		// > Check the max speed limit that was set
		uint16_t max_speed_limit_value;
		result = pololu_smc_pointer->get_max_duty_cycle_forward(&max_speed_limit_value);
		if (result)
		{
			printf("Pololu SMC - get max speed limit returned: %d, for I2C address %d\n", max_speed_limit_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get max speed limit NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		printf("\nFinished setting and checking the max speed limit for Pololu SMC with I2C address %d\n\n\n", pololu_smc_pointer->get_i2c_address() );

	}
	usleep(10000);


	usleep(10000);
	// Iterate over the pololu objects to set
	// the max acceleration limit
	for (int i_smc=0;i_smc<2;i_smc++)
	{
		if (i_smc==0)
		{
			pololu_smc_pointer = &pololu_smc1;
		}
		else if (i_smc==1)
		{
			pololu_smc_pointer = &pololu_smc2;
		}
		else
		{
			pololu_smc_pointer = &pololu_smc1;
		}

		printf("Now setting the max acceleration limit for Pololu SMC with I2C address %d\n\n", pololu_smc_pointer->get_i2c_address() );

		// > Send the "exit safe start" command
		int max_accel_limit_response_code;
		result = pololu_smc_pointer->set_motor_limit_max_acceleration(new_max_accel_limit, &max_accel_limit_response_code);
		if (result)
		{
			printf("Pololu SMC - set max acceleration limit successful with response code %d, for I2C address %d\n", max_accel_limit_response_code, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - set max acceleration limit NOT successful with response code %d for I2C address %d\n", max_accel_limit_response_code, pololu_smc_pointer->get_i2c_address() );
		}

		// > Check the max speed acceleration that was set
		uint16_t max_accel_limit_value;
		result = pololu_smc_pointer->get_max_acceleration_forward(&max_accel_limit_value);
		if (result)
		{
			printf("Pololu SMC - get max acceleration limit returned: %d, for I2C address %d\n", max_accel_limit_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get max acceleration limit NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		printf("\nFinished setting and checking the max acceleration limit for Pololu SMC with I2C address %d\n\n\n", pololu_smc_pointer->get_i2c_address() );

	}
	usleep(10000);


	usleep(10000);
	// Iterate over the pololu objects to set
	// the max deceleration limit
	for (int i_smc=0;i_smc<2;i_smc++)
	{
		if (i_smc==0)
		{
			pololu_smc_pointer = &pololu_smc1;
		}
		else if (i_smc==1)
		{
			pololu_smc_pointer = &pololu_smc2;
		}
		else
		{
			pololu_smc_pointer = &pololu_smc1;
		}

		printf("Now setting the max deceleration limit for Pololu SMC with I2C address %d\n\n", pololu_smc_pointer->get_i2c_address() );

		// > Send the "exit safe start" command
		int max_decel_limit_response_code;
		result = pololu_smc_pointer->set_motor_limit_max_deceleration(new_max_decel_limit, &max_decel_limit_response_code);
		if (result)
		{
			printf("Pololu SMC - set max deceleration limit successful with response code %d, for I2C address %d\n", max_decel_limit_response_code, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - set max deceleration limit NOT successful with response code %d for I2C address %d\n", max_decel_limit_response_code, pololu_smc_pointer->get_i2c_address() );
		}

		// > Check the max speed deceleration that was set
		uint16_t max_decel_limit_value;
		result = pololu_smc_pointer->get_max_deceleration_forward(&max_decel_limit_value);
		if (result)
		{
			printf("Pololu SMC - get max deceleration limit returned: %d, for I2C address %d\n", max_decel_limit_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get max deceleration limit NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		printf("\nFinished setting and checking the max deceleration limit for Pololu SMC with I2C address %d\n\n\n", pololu_smc_pointer->get_i2c_address() );

	}
	usleep(10000);





	// Iterate over steps of a ramp in motor speed
	int delta_t_current_measurement_in_micro_seconds = 10000;
	int num_delta_t_steps_per_speed_increment = 20;
	int num_speed_steps = 1;
	int num_detla_t_steps_at_max = 200;
	int speed_increment_percent = 80;
	int speed_minimum = 8;
	for (int i_accel=0;i_accel<2;i_accel++)
	{
		if (i_accel==0)
			printf("\n\nNOW STARTING ACCELERATION\n");
		else
			printf("\n\nNOW STARTING DECELERATION\n");


		for (int i_time=0;i_time<num_speed_steps;i_time++)
		{

			printf("\n\nNow starting time step %d\n", i_time);

			// Convert the time to a target speed
			int this_target_speed_percent = 0;
			if (i_accel==0)
			{
				this_target_speed_percent = (i_time+1) * speed_increment_percent;
				// Clip to the min and max
				if (this_target_speed_percent<speed_minimum)
					this_target_speed_percent = speed_minimum;
				if (this_target_speed_percent>100)
					this_target_speed_percent = 100;
			}
			else
			{
				this_target_speed_percent = (num_speed_steps-(i_time+1)) * speed_increment_percent;
				// Clip to the min and max
				if (this_target_speed_percent<speed_minimum)
					this_target_speed_percent = speed_minimum;
				if (this_target_speed_percent>100)
					this_target_speed_percent = 100;
				// Ensure that the last command is zero
				if (i_time==(num_speed_steps-1))
					this_target_speed_percent = 0;
			}
				
			

			// > Get the current speed value to check
			//   that it was set correctly
			for (int i_smc=0;i_smc<2;i_smc++)
			{
				if (i_smc==0)
					pololu_smc_pointer = &pololu_smc1;
				else if (i_smc==1)
					pololu_smc_pointer = &pololu_smc2;

				int16_t current_speed_value;
				result = pololu_smc_pointer->get_duty_cycle_3200(&current_speed_value);
				if (result)
				{
					printf("Pololu SMC - get speed value returned: %d, for I2C address %d\n", current_speed_value, pololu_smc_pointer->get_i2c_address() );
				}
				else
				{
					printf("FAILED - Pololu SMC - get speed value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
				}
			}

			// Set the new speed
			for (int i_smc=0;i_smc<2;i_smc++)
			{
				int direction_multiplier = 1;

				if (i_smc==0)
				{
					pololu_smc_pointer = &pololu_smc1;
					direction_multiplier = 1;
				}
				else if (i_smc==1)
				{
					pololu_smc_pointer = &pololu_smc2;
					direction_multiplier = -1;
				}

				int temp_target_speed = this_target_speed_percent * direction_multiplier;

				result = pololu_smc_pointer->set_motor_target_duty_cycle_percent(temp_target_speed);
				if (result)
				{
					printf("Pololu SMC - motor percent set to: %d, for I2C address %d\n", this_target_speed_percent, pololu_smc_pointer->get_i2c_address() );
				}
				else
				{
					printf("FAILED - Pololu SMC - set motor percent NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
				}
			}


			// > Get the target speed value to check
			//   that it was set correctly
			for (int i_smc=0;i_smc<2;i_smc++)
			{
				if (i_smc==0)
					pololu_smc_pointer = &pololu_smc1;
				else if (i_smc==1)
					pololu_smc_pointer = &pololu_smc2;

				int16_t current_target_speed_value;
				result = pololu_smc_pointer->get_target_duty_cycle_3200(&current_target_speed_value);
				if (result)
				{
					printf("Pololu SMC - get target speed value returned: %d, for I2C address %d\n", current_target_speed_value, pololu_smc_pointer->get_i2c_address() );
				}
				else
				{
					printf("FAILED - Pololu SMC - get target speed value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
				}
			}

			printf("\n");

			if (i_time<(num_speed_steps-1))
			{
				// Measure the current between speed changes
				printf("Now Measuring current:\n");
				for (int i_inc=0;i_inc<num_delta_t_steps_per_speed_increment;i_inc++)
				{
					// > Get the current measurement
					for (int i_smc=0;i_smc<2;i_smc++)
					{
						if (i_smc==0)
							pololu_smc_pointer = &pololu_smc1;
						else if (i_smc==1)
							pololu_smc_pointer = &pololu_smc2;

						uint16_t motor_current_value;
						result = pololu_smc_pointer->get_current_in_milliamps(&motor_current_value);
						if (result)
						{
							printf("I = %4d [mA], for I2C address %d\n", motor_current_value, pololu_smc_pointer->get_i2c_address() );
						}
						else
						{
							printf("FAILED - Pololu SMC - get motor current value value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
						}
						// > Check the input voltage
						float input_voltage_value;
						result = pololu_smc_pointer->get_input_voltage_in_volts(&input_voltage_value);
						if (result)
						{
							printf("I = %4.1f [V], for I2C address %d\n", input_voltage_value, pololu_smc_pointer->get_i2c_address() );
						}
						else
						{
							printf("FAILED - Pololu SMC - get input voltage value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
						}
					}
					usleep(delta_t_current_measurement_in_micro_seconds);
				}
			}

		} // END OF: "for (int i_time=0;i_time<num_time_steps_total;i_time++)"

		// Measure the current between accel. and deccel.
		printf("Now Measuring current:\n");
		for (int i_inc=0;i_inc<num_detla_t_steps_at_max;i_inc++)
		{
			// > Get the current measurement
			for (int i_smc=0;i_smc<2;i_smc++)
			{
				if (i_smc==0)
					pololu_smc_pointer = &pololu_smc1;
				else if (i_smc==1)
					pololu_smc_pointer = &pololu_smc2;

				uint16_t motor_current_value;
				result = pololu_smc_pointer->get_current_in_milliamps(&motor_current_value);
				if (result)
				{
					printf("I = %4d [mA], for I2C address %d\n", motor_current_value, pololu_smc_pointer->get_i2c_address() );
				}
				else
				{
					printf("FAILED - Pololu SMC - get motor current value value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
				}
				// > Check the input voltage
				float input_voltage_value;
				result = pololu_smc_pointer->get_input_voltage_in_volts(&input_voltage_value);
				if (result)
				{
					printf("I = %4.1f [V], for I2C address %d\n", input_voltage_value, pololu_smc_pointer->get_i2c_address() );
				}
				else
				{
					printf("FAILED - Pololu SMC - get input voltage value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
				}
			}
			usleep(delta_t_current_measurement_in_micro_seconds);
		}

	} // END OF: "for (int i_direction=0;i_direction<2;i_direction++)"


	// Iterate over the pololu objects and send the
	// "stop motor" command
	printf("\n");
	for (int i_smc=0;i_smc<2;i_smc++)
	{
		if (i_smc==0)
			pololu_smc_pointer = &pololu_smc1;
		else if (i_smc==1)
			pololu_smc_pointer = &pololu_smc2;

		result = pololu_smc_pointer->stop_motor();
		if (result)
		{
			printf("Pololu SMC - stop motor command sent to I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - stop motor command NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}
	}
	printf("\n");





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
