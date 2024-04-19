#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include <iostream>
#include <bitset>

#include "i2c_driver/i2c_driver.h"
#include "ina260/ina260.h"

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


	// Initialise a boolean for the result of each call
	bool result;


	printf("\n\n");
	printf("==============\n");
	printf("GET UNIQUE IDs\n");


	// > Get the Manufacturer ID
	uint16_t manufacturer_id;
	result = ina260.get_manufacturer_uid(&manufacturer_id);
	if (result)
	{
		printf("INA260 - get manufacturer UID returned: %d, for I2C address %d\n", manufacturer_id, ina260.get_i2c_address() );
	}
	else
	{
		printf("FAILED - INA260 - get manufacturer UID NOT successful for I2C address %d\n", ina260.get_i2c_address() );
	}


	// > Get the Die ID
	uint16_t die_id;
	result = ina260.get_die_and_revision_uid(&die_id);
	if (result)
	{
		printf("INA260 - get die UID returned: %d, for I2C address %d\n", manufacturer_id, ina260.get_i2c_address() );
	}
	else
	{
		printf("FAILED - INA260 - get die UID NOT successful for I2C address %d\n", ina260.get_i2c_address() );
	}


	printf("\n\n");
	printf("=====================\n");
	printf("SET THE CONFIGURATION\n");


	// > Set the configuration
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
	printf("================\n");
	printf("OVER POWER LIMIT\n");


	// > Set the alert configuration
	float over_power_limit_value = 123.4f;
	bool should_alert_when_conversion_ready = false;
	bool should_use_alert_active_high       = false;
	bool should_enable_alert_latch          = true;
	result = ina260.set_alert_over_power_limit_value_in_watts(over_power_limit_value, should_alert_when_conversion_ready, should_use_alert_active_high, should_enable_alert_latch);
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
		std::cout << "                                 expected: " << "0000100000000001" << "\n";
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
		std::cout << "INA260 - get alert limit returned uint16: " << std::bitset<16>(alert_limit_value_register) << ", i.e., " << ina260.power_uint16_to_float_in_watts(alert_limit_value_register) << "[Watts], for I2C address " << static_cast<int>(ina260.get_i2c_address()) << "\n";
	}
	else
	{
		printf("FAILED - INA260 - get alert limit NOT successful for I2C address %d\n", ina260.get_i2c_address() );
	}


	printf("\n\n");
	printf("===================\n");
	printf("UNDER VOLTAGE LIMIT\n");


	// > Set the alert configuration
	float under_voltage_limit_value = 9.876f;
	should_alert_when_conversion_ready = false;
	should_use_alert_active_high       = true;
	should_enable_alert_latch          = false;
	result = ina260.set_alert_under_voltage_limit_value_in_volts(under_voltage_limit_value, should_alert_when_conversion_ready, should_use_alert_active_high, should_enable_alert_latch);
	if (result)
	{
		printf("INA260 - set alert configuration successfully, for I2C address %d\n", ina260.get_i2c_address() );
	}
	else
	{
		printf("FAILED - INA260 - set alert configuration NOT successful for I2C address %d\n", ina260.get_i2c_address() );
	}

	// > Get the alert configuration to double check
	alert_configuration_register;
	result = ina260.get_alert_configuration_register(&alert_configuration_register);
	if (result)
	{
		std::cout << "INA260 - get alert configuration returned: " << std::bitset<16>(alert_configuration_register) << ", for I2C address " << static_cast<int>(ina260.get_i2c_address()) << "\n";
		std::cout << "                                 expected: " << "0001000000000010" << "\n";
	}
	else
	{
		printf("FAILED - INA260 - get alert configuration NOT successful for I2C address %d\n", ina260.get_i2c_address() );
	}

	// > Get the alert limit to double check
	alert_limit_value_register;
	result = ina260.get_alert_limit_value_register(&alert_limit_value_register);
	if (result)
	{
		std::cout << "INA260 - get alert limit returned uint16: " << std::bitset<16>(alert_limit_value_register) << ", i.e., " << ina260.voltage_uint16_to_float_in_volts(alert_limit_value_register) << "[Volts], for I2C address " << static_cast<int>(ina260.get_i2c_address()) << "\n";
	}
	else
	{
		printf("FAILED - INA260 - get alert limit NOT successful for I2C address %d\n", ina260.get_i2c_address() );
	}


	printf("\n\n");
	printf("==================\n");
	printf("OVER VOLTAGE LIMIT\n");


	// > Set the alert configuration
	float over_voltage_limit_value = 12.345f;
	should_alert_when_conversion_ready = true;
	should_use_alert_active_high       = false;
	should_enable_alert_latch          = false;
	result = ina260.set_alert_over_voltage_limit_value_in_volts(over_voltage_limit_value, should_alert_when_conversion_ready, should_use_alert_active_high, should_enable_alert_latch);
	if (result)
	{
		printf("INA260 - set alert configuration successfully, for I2C address %d\n", ina260.get_i2c_address() );
	}
	else
	{
		printf("FAILED - INA260 - set alert configuration NOT successful for I2C address %d\n", ina260.get_i2c_address() );
	}

	// > Get the alert configuration to double check
	alert_configuration_register;
	result = ina260.get_alert_configuration_register(&alert_configuration_register);
	if (result)
	{
		std::cout << "INA260 - get alert configuration returned: " << std::bitset<16>(alert_configuration_register) << ", for I2C address " << static_cast<int>(ina260.get_i2c_address()) << "\n";
		std::cout << "                                 expected: " << "0010010000000000" << "\n";
	}
	else
	{
		printf("FAILED - INA260 - get alert configuration NOT successful for I2C address %d\n", ina260.get_i2c_address() );
	}

	// > Get the alert limit to double check
	alert_limit_value_register;
	result = ina260.get_alert_limit_value_register(&alert_limit_value_register);
	if (result)
	{
		std::cout << "INA260 - get alert limit returned uint16: " << std::bitset<16>(alert_limit_value_register) << ", i.e., " << ina260.voltage_uint16_to_float_in_volts(alert_limit_value_register) << "[Volts], for I2C address " << static_cast<int>(ina260.get_i2c_address()) << "\n";
	}
	else
	{
		printf("FAILED - INA260 - get alert limit NOT successful for I2C address %d\n", ina260.get_i2c_address() );
	}


	printf("\n\n");
	printf("===================\n");
	printf("UNDER CURRENT LIMIT\n");


	// > Set the alert configuration
	float under_current_limit_value = 0.12345f;
	should_alert_when_conversion_ready = true;
	should_use_alert_active_high       = true;
	should_enable_alert_latch          = true;
	result = ina260.set_alert_under_current_limit_value_in_amps(under_current_limit_value, should_alert_when_conversion_ready, should_use_alert_active_high, should_enable_alert_latch);
	if (result)
	{
		printf("INA260 - set alert configuration successfully, for I2C address %d\n", ina260.get_i2c_address() );
	}
	else
	{
		printf("FAILED - INA260 - set alert configuration NOT successful for I2C address %d\n", ina260.get_i2c_address() );
	}

	// > Get the alert configuration to double check
	alert_configuration_register;
	result = ina260.get_alert_configuration_register(&alert_configuration_register);
	if (result)
	{
		std::cout << "INA260 - get alert configuration returned: " << std::bitset<16>(alert_configuration_register) << ", for I2C address " << static_cast<int>(ina260.get_i2c_address()) << "\n";
		std::cout << "                                 expected: " << "0100010000000011" << "\n";
	}
	else
	{
		printf("FAILED - INA260 - get alert configuration NOT successful for I2C address %d\n", ina260.get_i2c_address() );
	}

	// > Get the alert limit to double check
	alert_limit_value_register;
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
	printf("==================\n");
	printf("OVER CURRENT LIMIT\n");


	// > Set the alert configuration
	float over_current_limit_value = 0.34567f;
	should_alert_when_conversion_ready = false;
	should_use_alert_active_high       = false;
	should_enable_alert_latch          = false;
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
	alert_configuration_register;
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
	alert_limit_value_register;
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

	printf("\n");



	// Sleep for 100 milli seconds
	usleep(100000);


	// > Retireive the {current,voltage,power} measurements
	//   for a few seconds
	int num_measurements = 10;
	int measurement_delta_t_in_us = 100000;
	for (int i_time_step=0;i_time_step<num_measurements;i_time_step++)
	{
		// Get the measurements
		float I_measurement = 0.0f;
		float V_measurement = 0.0f;
		float P_measurement = 0.0f;
		bool result_I = ina260.get_current_measurement_in_amps(&I_measurement);
		bool result_V = ina260.get_voltage_measurement_in_volts(&V_measurement);
		bool result_P = ina260.get_power_measurement_in_watts(&P_measurement);

		uint16_t I_measurement_uint16 = 0;
		bool result_I_uint16 = ina260.get_current_measurement_as_uint16(&I_measurement_uint16);

		if (!result_I)
			printf("FAILED - INA260 - get current measurement NOT successful for I2C address %d\n", ina260.get_i2c_address() );
		if (!result_V)
			printf("FAILED - INA260 - get voltage measurement NOT successful for I2C address %d\n", ina260.get_i2c_address() );
		if (!result_P)
			printf("FAILED - INA260 - get power measurement NOT successful for I2C address %d\n", ina260.get_i2c_address() );
		if (!result_I_uint16)
			printf("FAILED - INA260 - get current measurement as uint16 NOT successful for I2C address %d\n", ina260.get_i2c_address() );

		if (result_I || result_V || result_P || result_I_uint16)
		{
			printf("INA260 - get {current,voltage,power} measurement returned:\n> I = %6.3f [A], %5.2f [V], %7.3f [W]\n", I_measurement, V_measurement, P_measurement );
			std::cout << "> I = " << std::bitset<16>(I_measurement_uint16) << "\n";
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
