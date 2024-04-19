#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include <iostream>
#include <bitset>

#include "i2c_driver/i2c_driver.h"
#include "ina260/ina260.h"
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
		printf("FAILED to open I2C device.\n" );
		return 1;
	}
	else
	{
		printf("I2C Device successfully opened with file descriptor = %d\n", i2c_driver.get_file_descriptor() );
	}


	// Initialise an object for the INA260 current sensor
	const uint8_t ina260_address = 0x40;
	INA260 ina260 (&i2c_driver, ina260_address);

	// Initialise an object for each of the Pololu
	// simple motor controllers
	const uint8_t pololu_smc_address1 = 70;
	Pololu_SMC_G2 pololu_smc1 (&i2c_driver, pololu_smc_address1);
	const uint8_t pololu_smc_address2 = 13;
	Pololu_SMC_G2 pololu_smc2 (&i2c_driver, pololu_smc_address2);

	// Initialise a pointer for iterating through
	// the Pololu SMC objects
	Pololu_SMC_G2 * pololu_smc_pointer;


	// Initialise a boolean for the result of each call
	bool result;


	printf("\n\n");
	printf("=================\n");
	printf("INA260: CONFIGURE\n");


	// > Set the configuration of the INA260
	INA260::Operating_Mode op_mode = INA260::Operating_Mode::current_voltage_continuous;
	INA260::Conversion_Time conv_time_current = INA260::Conversion_Time::t_1100_us;
	INA260::Conversion_Time conv_time_voltage = INA260::Conversion_Time::t_2116_us;
	INA260::Averaging_Mode avg_mode = INA260::Averaging_Mode::samples_0004;
	result = ina260.set_configuration(op_mode, conv_time_current, conv_time_voltage, avg_mode);
	if (result)
	{
		printf("INA260 - set configuration successfully, for I2C address %d\n", ina260.get_i2c_address() );
	}
	else
	{
		printf("FAILED - INA260 - set configuration NOT successful for I2C address %d\n", ina260.get_i2c_address() );
	}

	// > Get the configuration to double check
	INA260::Operating_Mode op_mode_retrieved;
	INA260::Conversion_Time conv_time_current_retrieved;
	INA260::Conversion_Time conv_time_voltage_retrieved;
	INA260::Averaging_Mode avg_mode_retrieved;
	result = ina260.get_configuration(&op_mode_retrieved, &conv_time_current_retrieved, &conv_time_voltage_retrieved, &avg_mode_retrieved);
	if (result)
	{
		printf("INA260 - get configuration returned:\n> op mode = %d\n> current conversion time = %d\n> voltage conversion time = %d\n> num averaging samples = %d\nfor I2C address %d\n", (int)op_mode_retrieved, ina260.conversion_time_enum_to_micro_seconds(conv_time_current_retrieved), ina260.conversion_time_enum_to_micro_seconds(conv_time_voltage_retrieved), ina260.averaging_mode_enum_to_samples(avg_mode_retrieved), ina260.get_i2c_address() );
	}
	else
	{
		printf("FAILED - INA260 - get configuration NOT successful for I2C address %d\n", ina260.get_i2c_address() );
	}


	printf("\n\n");
	printf("==========================\n");
	printf("INA260: OVER CURRENT LIMIT\n");


	// > Set the alert configuration
	float over_current_limit_value = 0.700f;
	bool should_alert_when_conversion_ready = false;
	bool should_use_alert_active_high       = false;
	bool should_enable_alert_latch          = false;
	result = ina260.set_alert_over_current_limit_value_in_amps(over_current_limit_value, should_alert_when_conversion_ready, should_use_alert_active_high, should_enable_alert_latch);
	if (result)
	{
		printf("INA260 - set alert configuration successfully, for I2C address %d\n", ina260.get_i2c_address() );
	}
	else
	{
		printf("FAILED - INA260 - set alert configuration NOT successful for I2C address %d\n", ina260.get_i2c_address() );
	}

	// > Get the alert configuration to double check
	uint16_t alert_configuration_register;
	result = ina260.get_alert_configuration_register(&alert_configuration_register);
	if (result)
	{
		std::cout << "INA260 - get alert configuration returned: " << std::bitset<16>(alert_configuration_register) << ", for I2C address " << static_cast<int>(ina260.get_i2c_address()) << "\n";
		std::cout << "                                 expected: " << "1000000000000000" << "\n";
	}
	else
	{
		printf("FAILED - INA260 - get alert configuration NOT successful for I2C address %d\n", ina260.get_i2c_address() );
	}

	// > Get the alert limit to double check
	uint16_t alert_limit_value_register;
	result = ina260.get_alert_limit_value_register(&alert_limit_value_register);
	if (result)
	{
		//printf("INA260 - get alert limit returned: %f [Amps], for I2C address %d\n", ina260.current_uint16_to_float_in_amps(alert_limit_value_register), ina260.get_i2c_address() );
		std::cout << "INA260 - get alert limit returned uint16: " << std::bitset<16>(alert_limit_value_register) << ", i.e., " << ina260.current_uint16_to_float_in_amps(alert_limit_value_register) << "[Amps], for I2C address " << static_cast<int>(ina260.get_i2c_address()) << "\n";
	}
	else
	{
		printf("FAILED - INA260 - get alert limit NOT successful for I2C address %d\n", ina260.get_i2c_address() );
	}

	
	printf("\n\n");
	printf("==========================\n");
	printf("MOTORS: CHECK ERROR STATUS\n");


	// Iterate over the pololu objects to check
	// some status details
	for (int i_smc=0;i_smc<2;i_smc++)
	{
		if (i_smc==0)
			pololu_smc_pointer = &pololu_smc1;
		else if (i_smc==1)
			pololu_smc_pointer = &pololu_smc2;
		else
			pololu_smc_pointer = &pololu_smc1;

		printf("Now checking status for Pololu SMC with I2C address %d\n", pololu_smc_pointer->get_i2c_address() );

		printf("\n");

		// > Send the "exit safe start" command
		result = pololu_smc_pointer->exit_safe_start();
		if (result)
			printf("Pololu SMC - exit safe start successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		else
			printf("FAILED - Pololu SMC - exit safe start NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );

		printf("\n");

		// > Check the status flag registers
		uint16_t error_status;
		result = pololu_smc_pointer->get_error_status(&error_status);
		if (result)
			std::cout << "Pololu SMC - get error status returned: " << std::bitset<16>(error_status) << ", for I2C address " << pololu_smc_pointer->get_i2c_address() << "\n";
		else
			printf("FAILED - Pololu SMC - get error status NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );

		printf("\n");

		// > Check the input voltage
		float input_voltage_value;
		result = pololu_smc_pointer->get_input_voltage_in_volts(&input_voltage_value);
		if (result)
			printf("Pololu SMC - get input voltage value returned: %f [Volts], for I2C address %d\n", input_voltage_value, pololu_smc_pointer->get_i2c_address() );
		else
			printf("FAILED - Pololu SMC - get input voltage value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );

		printf("\n");		
		printf("Finished checking status for Pololu SMC with I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
	}



	printf("\n\n");
	printf("==================\n");
	printf("MOTORS: SET LIMITS\n");


	// SPECIFY THE VARIOUS LIMITS
	int new_current_limit_in_milliamps = 5000;
	int new_max_speed_limit = 2560;
	int new_max_accel_limit = 1;
	int new_max_decel_limit = 5;


	usleep(10000);
	// Iterate over the pololu objects to set
	// the limits
	for (int i_smc=0;i_smc<2;i_smc++)
	{
		if (i_smc==0)
			pololu_smc_pointer = &pololu_smc1;
		else if (i_smc==1)
			pololu_smc_pointer = &pololu_smc2;
		else
			pololu_smc_pointer = &pololu_smc1;

		printf("Now setting the limits for Pololu SMC with I2C address %d\n", pololu_smc_pointer->get_i2c_address() );

		printf("\n");

		// > Set the current limit
		result = pololu_smc_pointer->set_current_limit_in_milliamps(new_current_limit_in_milliamps);
		if (result)
			printf("Pololu SMC - set current limit successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		else
			printf("FAILED - Pololu SMC - set current limit NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );

		printf("\n");

		// > Check the current limit that was set
		uint16_t current_limit_value;
		result = pololu_smc_pointer->get_current_limit(&current_limit_value);
		if (result)
			printf("Pololu SMC - get current limit returned: %d, for I2C address %d\n", current_limit_value, pololu_smc_pointer->get_i2c_address() );
		else
			printf("FAILED - Pololu SMC - get current limit NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );

		printf("\n");

		// > Set the max speed limit
		int max_speed_limit_response_code;
		result = pololu_smc_pointer->set_motor_limit_max_duty_cycle(new_max_speed_limit, &max_speed_limit_response_code);
		if (result)
			printf("Pololu SMC - set max duty cycle limit successful with response code %d, for I2C address %d\n", max_speed_limit_response_code, pololu_smc_pointer->get_i2c_address() );
		else
			printf("FAILED - Pololu SMC - set max duty cycle limit NOT successful with response code %d, for I2C address %d\n", max_speed_limit_response_code, pololu_smc_pointer->get_i2c_address() );

		printf("\n");

		// > Check the max speed limit that was set
		uint16_t max_duty_cycle_limit_value;
		result = pololu_smc_pointer->get_max_duty_cycle_forward(&max_duty_cycle_limit_value);
		if (result)
			printf("Pololu SMC - get max duty cycle limit returned: %d, for I2C address %d\n", max_duty_cycle_limit_value, pololu_smc_pointer->get_i2c_address() );
		else
			printf("FAILED - Pololu SMC - get max duty cycle limit NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );

		printf("\n");

		// > Set the max acceleration limit
		int max_accel_limit_response_code;
		result = pololu_smc_pointer->set_motor_limit_max_acceleration(new_max_accel_limit, &max_accel_limit_response_code);
		if (result)
			printf("Pololu SMC - set max acceleration limit successful with response code %d, for I2C address %d\n", max_accel_limit_response_code, pololu_smc_pointer->get_i2c_address() );
		else
			printf("FAILED - Pololu SMC - set max acceleration limit NOT successful with response code %d for I2C address %d\n", max_accel_limit_response_code, pololu_smc_pointer->get_i2c_address() );

		printf("\n");

		// > Check the max speed acceleration that was set
		uint16_t max_accel_limit_value;
		result = pololu_smc_pointer->get_max_acceleration_forward(&max_accel_limit_value);
		if (result)
			printf("Pololu SMC - get max acceleration limit returned: %d, for I2C address %d\n", max_accel_limit_value, pololu_smc_pointer->get_i2c_address() );
		else
			printf("FAILED - Pololu SMC - get max acceleration limit NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );

		printf("\n");

		// > Check the max speed deceleration that was set
		int max_decel_limit_response_code;
		result = pololu_smc_pointer->set_motor_limit_max_deceleration(new_max_decel_limit, &max_decel_limit_response_code);
		if (result)
			printf("Pololu SMC - set max deceleration limit successful with response code %d, for I2C address %d\n", max_decel_limit_response_code, pololu_smc_pointer->get_i2c_address() );
		else
			printf("FAILED - Pololu SMC - set max deceleration limit NOT successful with response code %d for I2C address %d\n", max_decel_limit_response_code, pololu_smc_pointer->get_i2c_address() );

		printf("\n");

		// > Check the max speed deceleration that was set
		uint16_t max_decel_limit_value;
		result = pololu_smc_pointer->get_max_deceleration_forward(&max_decel_limit_value);
		if (result)
			printf("Pololu SMC - get max deceleration limit returned: %d, for I2C address %d\n", max_decel_limit_value, pololu_smc_pointer->get_i2c_address() );
		else
			printf("FAILED - Pololu SMC - get max deceleration limit NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );

		printf("\n");

		printf("\nFinished setting and checking the limits for Pololu SMC with I2C address %d\n", pololu_smc_pointer->get_i2c_address() );

	}
	usleep(10000);


	// Sleep for 100 milli seconds
	usleep(100000);

	printf("\n\n");
	printf("=====================\n");
	printf("NOW STARTING THE TEST\n");
	printf("\n\n");


	// Specify how the test will be performed
	int num_measurements = 70;
	int start_motors_at = 10;
	int stop_motors_at = 60;
	if (stop_motors_at>num_measurements)
		stop_motors_at = (num_measurements-1);
	int measurement_delta_t_in_us = 100000;

	// Iterate over the number of time steps
	for (int i_time_step=0;i_time_step<num_measurements;i_time_step++)
	{
		// Start/Stop the motors
		if ( (i_time_step==start_motors_at) || (i_time_step==stop_motors_at) )
		{
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

				int temp_target_speed = 0;
				if (i_time_step==start_motors_at)
					temp_target_speed = 100 * direction_multiplier;

				result = pololu_smc_pointer->set_motor_target_duty_cycle_percent(temp_target_speed);
				if (result)
					printf("Pololu SMC - motor percent set to: %d, for I2C address %d\n", temp_target_speed, pololu_smc_pointer->get_i2c_address() );
				else
					printf("FAILED - Pololu SMC - set motor percent NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
			}
		}

		// Get the measurements
		float I_measurement = 0.0f;
		float V_measurement = 0.0f;
		float P_measurement = 0.0f;
		bool result_I = ina260.get_current_measurement_in_amps(&I_measurement);
		bool result_V = ina260.get_voltage_measurement_in_volts(&V_measurement);
		bool result_P = ina260.get_power_measurement_in_watts(&P_measurement);

		//uint16_t I_measurement_uint16 = 0;
		//bool result_I_uint16 = ina260.get_current_measurement_as_uint16(&I_measurement_uint16);

		// Get the alert register
		uint16_t alert_register;
		bool result_alert = ina260.get_alert_configuration_register(&alert_register);

		// Check the alert bit
		bool isActive_alert_bit = static_cast<bool>( (alert_register & (0x0001 << 4)) >> 4  );

		if (!result_I)
			printf("FAILED - INA260 - get current measurement NOT successful for I2C address %d\n", ina260.get_i2c_address() );
		if (!result_V)
			printf("FAILED - INA260 - get voltage measurement NOT successful for I2C address %d\n", ina260.get_i2c_address() );
		if (!result_P)
			printf("FAILED - INA260 - get power measurement NOT successful for I2C address %d\n", ina260.get_i2c_address() );
		//if (!result_I_uint16)
		//	printf("FAILED - INA260 - get current measurement as uint16 NOT successful for I2C address %d\n", ina260.get_i2c_address() );
		if (!result_alert)
			printf("FAILED - INA260 - get alert configuration NOT successful for I2C address %d\n", ina260.get_i2c_address() );

		if (result_I || result_V || result_P || result_alert)
		{
			printf("INA260 > I = %6.3f [A], %5.2f [V], %7.3f [W]\n", I_measurement, V_measurement, P_measurement );
			std::cout << " Alert Reg = " << std::bitset<16>(alert_register);
			if (isActive_alert_bit)
				printf("  ALERT BIT IS ACTIVE (limit = %f [Amps])", over_current_limit_value);
			printf("\n");
		}

		// Sleep for specified micro seconds
		usleep(measurement_delta_t_in_us);
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
