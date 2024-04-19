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
		printf("FAILED to open I2C device.\n" );
		return 1;
	}
	else
	{
		printf("I2C Device successfully opened with file descriptor = %d\n", i2c_driver.get_file_descriptor() );
	}


	// Initialise an object for each of the Pololu
	// simple motor controllers
	const uint8_t pololu_smc_address1 = 14;
	Pololu_SMC_G2 pololu_smc1 (&i2c_driver, pololu_smc_address1);
	const uint8_t pololu_smc_address2 = 13;
	Pololu_SMC_G2 pololu_smc2 (&i2c_driver, pololu_smc_address2);

	// Initialise a pointer for iterating through
	// the Pololu SMC objects
	Pololu_SMC_G2 * pololu_smc_pointer;
	

	// Iterate over the pololu objects
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



		bool result = pololu_smc_pointer->exit_safe_start();
		if (result)
		{
			printf("Pololu SMC - exit safe start successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - exit safe start NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}





		// > For the product ID and firmware version
		uint16_t product_id;
		uint8_t firmware_major;
		uint8_t firmware_minor;
		result = pololu_smc_pointer->get_firmware_version(&product_id, &firmware_major, &firmware_minor);
		if (result)
		{
			printf("Pololu SMC - get firmware version returned product ID: %d, and firmware: %d.%d, for I2C address %d\n", product_id, firmware_major, firmware_minor, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get firmware version NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}





		// > For the status flag registers
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





		// > For the RC channels
		uint16_t rc1_unlimited_raw_value;
		result = pololu_smc_pointer->get_rc1_unlimited_raw_value(&rc1_unlimited_raw_value);
		if (result)
		{
			printf("Pololu SMC - get RC1 unlimited raw value returned: %d, for I2C address %d\n", rc1_unlimited_raw_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get RC1 unlimited raw value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		uint16_t rc1_raw_value;
		result = pololu_smc_pointer->get_rc1_raw_value(&rc1_raw_value);
		if (result)
		{
			printf("Pololu SMC - get RC1 raw value returned: %d, for I2C address %d\n", rc1_raw_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get RC1 raw value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		int16_t rc1_scaled_value;
		result = pololu_smc_pointer->get_rc1_scaled_value(&rc1_scaled_value);
		if (result)
		{
			printf("Pololu SMC - get RC1 scaled value returned: %d, for I2C address %d\n", rc1_scaled_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get RC1 scaled value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		uint16_t rc2_unlimited_raw_value;
		result = pololu_smc_pointer->get_rc2_unlimited_raw_value(&rc2_unlimited_raw_value);
		if (result)
		{
			printf("Pololu SMC - get RC2 unlimited raw value returned: %d, for I2C address %d\n", rc2_unlimited_raw_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get RC2 unlimited raw value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		uint16_t rc2_raw_value;
		result = pololu_smc_pointer->get_rc2_raw_value(&rc2_raw_value);
		if (result)
		{
			printf("Pololu SMC - get RC2 raw value returned: %d, for I2C address %d\n", rc2_raw_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get RC2 raw value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		int16_t rc2_scaled_value;
		result = pololu_smc_pointer->get_rc2_scaled_value(&rc2_scaled_value);
		if (result)
		{
			printf("Pololu SMC - get RC2 scaled value returned: %d, for I2C address %d\n", rc2_scaled_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get RC2 scaled value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}





		// > For the analog channels
		uint16_t an1_unlimited_raw_value;
		result = pololu_smc_pointer->get_an1_unlimited_raw_value(&an1_unlimited_raw_value);
		if (result)
		{
			printf("Pololu SMC - get AN1 unlimited raw value returned: %d, for I2C address %d\n", an1_unlimited_raw_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get AN1 unlimited raw value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		uint16_t an1_raw_value;
		result = pololu_smc_pointer->get_an1_raw_value(&an1_raw_value);
		if (result)
		{
			printf("Pololu SMC - get AN1 raw value returned: %d, for I2C address %d\n", an1_raw_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get AN1 raw value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		int16_t an1_scaled_value;
		result = pololu_smc_pointer->get_an1_scaled_value(&an1_scaled_value);
		if (result)
		{
			printf("Pololu SMC - get AN1 scaled value returned: %d, for I2C address %d\n", an1_scaled_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get AN1 scaled value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		uint16_t an2_unlimited_raw_value;
		result = pololu_smc_pointer->get_an2_unlimited_raw_value(&an2_unlimited_raw_value);
		if (result)
		{
			printf("Pololu SMC - get AN2 unlimited raw value returned: %d, for I2C address %d\n", an2_unlimited_raw_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get AN2 unlimited raw value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		uint16_t an2_raw_value;
		result = pololu_smc_pointer->get_an2_raw_value(&an2_raw_value);
		if (result)
		{
			printf("Pololu SMC - get AN2 raw value returned: %d, for I2C address %d\n", an2_raw_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get AN2 raw value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		int16_t an2_scaled_value;
		result = pololu_smc_pointer->get_an2_scaled_value(&an2_scaled_value);
		if (result)
		{
			printf("Pololu SMC - get AN2 scaled value returned: %d, for I2C address %d\n", an2_scaled_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get AN2 scaled value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}
		




		// > For diagnostic variables
		int16_t target_speed_value;
		result = pololu_smc_pointer->get_target_duty_cycle_3200(&target_speed_value);
		if (result)
		{
			printf("Pololu SMC - get target speed value returned: %d, for I2C address %d\n", target_speed_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get target speed value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		int16_t speed_value;
		result = pololu_smc_pointer->get_duty_cycle_3200(&speed_value);
		if (result)
		{
			printf("Pololu SMC - get speed value returned: %d, for I2C address %d\n", speed_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get speed value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		uint16_t brake_amount_value;
		result = pololu_smc_pointer->get_brake_amount(&brake_amount_value);
		if (result)
		{
			printf("Pololu SMC - get brake amount value returned: %d, for I2C address %d\n", brake_amount_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get brake amount value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

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

		float temperature_a_value;
		result = pololu_smc_pointer->get_temperature_a(&temperature_a_value);
		if (result)
		{
			printf("Pololu SMC - get temperature A value returned: %f[degrees], for I2C address %d\n", temperature_a_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get temperature A value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		float temperature_b_value;
		result = pololu_smc_pointer->get_temperature_b(&temperature_b_value);
		if (result)
		{
			printf("Pololu SMC - get temperature B value returned: %f [degrees], for I2C address %d\n", temperature_b_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get temperature B value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		float rc_period_value;
		result = pololu_smc_pointer->get_rc_period_in_seconds(&rc_period_value);
		if (result)
		{
			printf("Pololu SMC - get RC period value returned: %f [seconds], for I2C address %d\n", rc_period_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get RC period value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		float baud_rate_in_bps_value;
		result = pololu_smc_pointer->get_baud_rate_register_in_bps(&baud_rate_in_bps_value);
		if (result)
		{
			printf("Pololu SMC - get baud rate value returned: %f [bits per second], for I2C address %d\n", baud_rate_in_bps_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get baud rate value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		uint16_t up_time_low_value;
		result = pololu_smc_pointer->get_up_time_low(&up_time_low_value);
		if (result)
		{
			printf("Pololu SMC - get up time low value returned: %d [milliseconds], for I2C address %d\n", up_time_low_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get up time low value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		uint16_t up_time_high_value;
		result = pololu_smc_pointer->get_up_time_high(&up_time_high_value);
		if (result)
		{
			printf("Pololu SMC - get up time high value returned: %d [65536 milliseconds], for I2C address %d\n", up_time_high_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get up time high value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		float up_time_in_seconds_value;
		result = pololu_smc_pointer->get_up_time_in_seconds(&up_time_in_seconds_value);
		if (result)
		{
			printf("Pololu SMC - get up time value returned: %f [seconds], for I2C address %d\n", up_time_in_seconds_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get up time value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}





		// > For motor speed limits (forward)
		uint16_t max_speed_forward_value;
		result = pololu_smc_pointer->get_max_duty_cycle_forward(&max_speed_forward_value);
		if (result)
		{
			printf("Pololu SMC - get max speed forward value returned: %d [0-3200], for I2C address %d\n", max_speed_forward_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get max speed forward value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		uint16_t max_acceleration_forward_value;
		result = pololu_smc_pointer->get_max_acceleration_forward(&max_acceleration_forward_value);
		if (result)
		{
			printf("Pololu SMC - get max acceleration forward value returned: %d [0-3200 delta per update period], for I2C address %d\n", max_acceleration_forward_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get max acceleration forward value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		uint16_t max_deceleration_forward_value;
		result = pololu_smc_pointer->get_max_deceleration_forward(&max_deceleration_forward_value);
		if (result)
		{
			printf("Pololu SMC - get max deceleration forward value returned: %d [0-3200 delta per update period], for I2C address %d\n", max_deceleration_forward_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get max deceleration forward value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		uint16_t brake_duration_forward_value;
		result = pololu_smc_pointer->get_brake_duration_forward(&brake_duration_forward_value);
		if (result)
		{
			printf("Pololu SMC - get brake duration value returned: %d [milliseconds], for I2C address %d\n", brake_duration_forward_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get brake duration forward value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		float brake_duration_forward_in_seconds_value;
		result = pololu_smc_pointer->get_brake_duration_forward_in_seconds(&brake_duration_forward_in_seconds_value);
		if (result)
		{
			printf("Pololu SMC - get brake duration value returned: %f [seconds], for I2C address %d\n", brake_duration_forward_in_seconds_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get brake duration forward value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		uint16_t starting_speed_forward_value;
		result = pololu_smc_pointer->get_starting_duty_cycle_forward(&starting_speed_forward_value);
		if (result)
		{
			printf("Pololu SMC - get starting speed forward value returned: %d [0-3200], for I2C address %d\n", starting_speed_forward_value, pololu_smc_pointer->get_i2c_address() );
		}
		else
		{
			printf("FAILED - Pololu SMC - get starting speed forward value NOT successful for I2C address %d\n", pololu_smc_pointer->get_i2c_address() );
		}

		printf("\nFinished getting variables for Pololu SMC with I2C address %d\n\n\n", pololu_smc_pointer->get_i2c_address() );

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
