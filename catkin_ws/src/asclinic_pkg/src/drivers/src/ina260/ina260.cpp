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
// I2C driver for the INA260 {current,voltage,power} sensor
//
// ----------------------------------------------------------------------------





#include "ina260/ina260.h"





// ----------------------------------------------------------------------------
//   CCCC   OOO   N   N   SSSS  TTTTT  RRRR   U   U   CCCC  TTTTT   OOO   RRRR
//  C      O   O  NN  N  S        T    R   R  U   U  C        T    O   O  R   R
//  C      O   O  N N N   SSS     T    RRRR   U   U  C        T    O   O  RRRR
//  C      O   O  N  NN      S    T    R   R  U   U  C        T    O   O  R   R
//   CCCC   OOO   N   N  SSSS     T    R   R   UUU    CCCC    T     OOO   R   R
// ----------------------------------------------------------------------------
INA260::INA260(I2C_Driver * i2c_driver)
{
	this->m_i2c_address = INA260_I2C_ADDRESS_DEFAULT;
	this->m_i2c_driver = i2c_driver;
}

INA260::INA260(I2C_Driver * i2c_driver, uint8_t address)
{
	// Check that the address is in the range [0,127]
	if (address > 127)
	{
		// Inform the user
		perror("Address supplied is greater than 127. Instead setting the address to the default of 13.");
		// Default the address to INA260_I2C_ADDRESS_DEFAULT
		address = INA260_I2C_ADDRESS_DEFAULT;
	}

	this->m_i2c_address = address;
	this->m_i2c_driver = i2c_driver;
}





// ------------------------------------------------------
//   GGGG  EEEEE  TTTTT       &&&      SSSS  EEEEE  TTTTT
//  G      E        T        &        S      E        T
//  G  GG  EEE      T        && &      SSS   EEE      T
//  G   G  E        T       &  &          S  E        T
//   GGGG  EEEEE    T        && &     SSSS   EEEEE    T
// ------------------------------------------------------

uint8_t INA260::get_i2c_address()
{
	return this->m_i2c_address;
}

bool INA260::set_i2c_address(uint8_t new_address)
{
	if (new_address<0 || new_address>127)
	{
		return false;
	}
	this->m_i2c_address = new_address;
	return true;
}





// ------------------------------------------------------------
//  FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N   SSSS
//  F      U   U  NN  N  C        T     I   O   O  NN  N  S
//  FFF    U   U  N N N  C        T     I   O   O  N N N   SSS
//  F      U   U  N  NN  C        T     I   O   O  N  NN      S
//  F       UUU   N   N   CCCC    T    III   OOO   N   N  SSSS
// ------------------------------------------------------------

// PRIVATE FUNCTION:
bool INA260::read_register(uint8_t register_address, uint16_t * value)
{
	// Put the "register address" into a uint8 array
	uint8_t write_array[] = { register_address };
	// Specify the number of bytes in the array
	int num_write_bytes = sizeof(write_array);

	// Initialise a uint8 array for the returned
	// value of the requested register
	// > Note that all registers are returned as
	//   a two byte response
	int num_value_bytes = 2;
	uint8_t value_array[num_value_bytes];

	// Call the i2c_driver function
	bool wasSuccessful = this->m_i2c_driver->write_data_then_read_data(this->m_i2c_address, num_write_bytes, write_array, num_value_bytes, value_array);
	// Check the status
	if (wasSuccessful)
	{
		// Convert the two unit8 values into the
		// uint16 value of the variable
		// > Note that the most significant byte is
		//   read first, i.e., "value_array[0]",
		//   followed by the least significant byte
		// > Note that the calling function is
		//   responsible to convert this to int16
		//   if the variable takes signed values.
		*value = 256 * value_array[0] + value_array[1];
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Inform the user
		//perror("FAILED to get the requested variable.");
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PRIVATE FUNCTION:
bool INA260::write_register(uint8_t register_address, uint16_t value)
{
	// Convert the new limit value to its two
	// byte representation
	// > The first data byte written contains the
	//   most significant eight bits of the value
	// > The second data byte written contains the
	//   least significant eight bits of the value	
	uint8_t value_byte_1 = value >> 8;
	uint8_t value_byte_2 = value & 0xFF;

	// Put the "register address" and the "value"
	// into a uint8 array
	uint8_t write_array[] = { register_address , value_byte_1 , value_byte_2 };
	// Specify the number of bytes in the array
	int num_write_bytes = sizeof(write_array);

	// Call the i2c_driver function
	bool wasSuccessful = this->m_i2c_driver->write_data(this->m_i2c_address, num_write_bytes, write_array);
	// Check the status
	if (wasSuccessful)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Inform the user
		//perror("FAILED to exit safe start.");
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PRIVATE FUNCTION
bool INA260::set_alert_configuration_and_limit(uint16_t limit_value, uint16_t alert_function_bits, bool should_alert_when_conversion_ready, bool should_use_alert_active_high, bool should_enable_alert_latch)
{
	// Construct the 16-bit Mask/Enable register
	// > For the alert on conversion ready bit
	uint16_t conversion_ready_bit = 0x0000;
	if (should_alert_when_conversion_ready)
		conversion_ready_bit = static_cast<uint16_t>(INA260::Alert_Functions::on_conversion_ready);
	// > For the alert polarity bit
	uint16_t alert_polarity_bit = 0x0000;
	if (should_use_alert_active_high)
		alert_polarity_bit = static_cast<uint16_t>(INA260::Alert_Functions::polarity_active_high);
	// > For the alert latch bit
	uint16_t alert_latch_bit = 0x0000;
	if (should_enable_alert_latch)
		alert_latch_bit = static_cast<uint16_t>(INA260::Alert_Functions::enable_alert_latch);

	// Combine using "bitwise or"
	uint16_t mask_enable_value = alert_function_bits | conversion_ready_bit | alert_polarity_bit | alert_latch_bit;

	// Call the function to write the mask/enable register
	bool wasSuccessful_mask_enable = this->write_register(INA260_REGISTER_MASK_ENABLE, mask_enable_value);

	// Call the function to write the alert limit register
	bool wasSuccessful_alert_limit = this->write_register(INA260_REGISTER_ALERT_LIMIT_VALUE, limit_value);

	// Return the combine success flag
	return (wasSuccessful_mask_enable && wasSuccessful_alert_limit);
}

// PRIVATE FUNCTION
bool INA260::set_alert_current_limit_value_in_amps(float limit_value, uint16_t alert_function_bits, bool should_alert_when_conversion_ready, bool should_use_alert_active_high, bool should_enable_alert_latch)
{
	// Restrict the limit value to be in the range
	// [-15.0,15.0]
	if (limit_value < -15.0)
		limit_value = 15.0;
	else if (limit_value > 15.0)
		limit_value = 15.0;

	// Convert the limit value to the 16-bit representation for
	// the alert register
	uint16_t limit_value_as_uint16 = this->current_float_in_amps_to_uint16(limit_value);

	// Call the function to write the alert configuration
	// register and the alert limit register
	return this->set_alert_configuration_and_limit(limit_value_as_uint16, alert_function_bits, should_alert_when_conversion_ready, should_use_alert_active_high, should_enable_alert_latch);
}

// PRIVATE FUNCTION
bool INA260::set_alert_voltage_limit_value_in_volts(float limit_value, uint16_t alert_function_bits, bool should_alert_when_conversion_ready, bool should_use_alert_active_high, bool should_enable_alert_latch)
{
	// Restrict the limit value to be in the range
	// [0,40.96]
	if (limit_value < 0.0)
		limit_value = 0.0;
	else if (limit_value > 40.95)
		limit_value = 40.95;

	// Convert the limit value to the 16-bit representation for
	// the alert register
	uint16_t limit_value_as_uint16 = this->voltage_float_in_volts_to_uint16(limit_value);

	// Call the function to write the alert configuration
	// register and the alert limit register
	return this->set_alert_configuration_and_limit(limit_value_as_uint16, alert_function_bits, should_alert_when_conversion_ready, should_use_alert_active_high, should_enable_alert_latch);
}

// PUBLIC FUNCTION
bool INA260::set_alert_over_current_limit_value_in_amps(float limit_value, bool should_alert_when_conversion_ready, bool should_use_alert_active_high, bool should_enable_alert_latch)
{
	// Select the alert function bits of 16-bit Mask/Enable
	// register so that the over voltage alert is active
	uint16_t alert_function_bits = static_cast<uint16_t>(INA260::Alert_Functions::over_current_limit);

	// Call the function to convert the limit value and then
	// write the alert configuration register and the alert
	// limit register
	return this->set_alert_current_limit_value_in_amps(limit_value, alert_function_bits, should_alert_when_conversion_ready, should_use_alert_active_high, should_enable_alert_latch);
}

// PUBLIC FUNCTION
bool INA260::set_alert_under_current_limit_value_in_amps(float limit_value, bool should_alert_when_conversion_ready, bool should_use_alert_active_high, bool should_enable_alert_latch)
{
	// Select the alert function bits of 16-bit Mask/Enable
	// register so that the over voltage alert is active
	uint16_t alert_function_bits = static_cast<uint16_t>(INA260::Alert_Functions::under_current_limit);

	// Call the function to convert the limit value and then
	// write the alert configuration register and the alert
	// limit register
	return this->set_alert_current_limit_value_in_amps(limit_value, alert_function_bits, should_alert_when_conversion_ready, should_use_alert_active_high, should_enable_alert_latch);
}

// PUBLIC FUNCTION
bool INA260::set_alert_over_voltage_limit_value_in_volts(float limit_value, bool should_alert_when_conversion_ready, bool should_use_alert_active_high, bool should_enable_alert_latch)
{
	// Select the alert function bits of 16-bit Mask/Enable
	// register so that the over voltage alert is active
	uint16_t alert_function_bits = static_cast<uint16_t>(INA260::Alert_Functions::over_voltage_limit);

	// Call the function to convert the limit value and then
	// write the alert configuration register and the alert
	// limit register
	return this->set_alert_voltage_limit_value_in_volts(limit_value, alert_function_bits, should_alert_when_conversion_ready, should_use_alert_active_high, should_enable_alert_latch);
}

// PUBLIC FUNCTION
bool INA260::set_alert_under_voltage_limit_value_in_volts(float limit_value, bool should_alert_when_conversion_ready, bool should_use_alert_active_high, bool should_enable_alert_latch)
{
	// Select the alert function bits of 16-bit Mask/Enable
	// register so that the over voltage alert is active
	uint16_t alert_function_bits = static_cast<uint16_t>(INA260::Alert_Functions::under_voltage_limit);

	return this->set_alert_voltage_limit_value_in_volts(limit_value, alert_function_bits, should_alert_when_conversion_ready, should_use_alert_active_high, should_enable_alert_latch);
}

// PUBLIC FUNCTION
bool INA260::set_alert_over_power_limit_value_in_watts(float limit_value, bool should_alert_when_conversion_ready, bool should_use_alert_active_high, bool should_enable_alert_latch)
{
	// Select the alert function bits of 16-bit Mask/Enable
	// register so that the over voltage alert is active
	uint16_t alert_function_bits = static_cast<uint16_t>(INA260::Alert_Functions::over_power_limit);

	// Restrict the limit value to be in the range
	// [0,419.42]
	if (limit_value < 0.0)
		limit_value = 0.0;
	else if (limit_value > 419.42)
		limit_value = 419.42;

	// Convert the limit value to the 16-bit representation for
	// the alert register
	uint16_t limit_value_as_uint16 = this->power_float_in_watts_to_uint16(limit_value);

	// Call the function to write the alert configuration
	// register and the alert limit register
	return this->set_alert_configuration_and_limit(limit_value_as_uint16, alert_function_bits, should_alert_when_conversion_ready, should_use_alert_active_high, should_enable_alert_latch);
}

// PUBLIC FUNCTION
// SETTING THE CONFIGURATION REGISTER
bool INA260::set_configuration(INA260::Operating_Mode op_mode, INA260::Conversion_Time conv_time_current, INA260::Conversion_Time conv_time_voltage, INA260::Averaging_Mode avg_mode)
{
	// Shift the configuration bits as required
	uint16_t op_mode_shifted           = static_cast<uint16_t>(op_mode)           << 0;
	uint16_t conv_time_current_shifted = static_cast<uint16_t>(conv_time_current) << 3;
	uint16_t conv_time_voltage_shifted = static_cast<uint16_t>(conv_time_voltage) << 6;
	uint16_t avg_mode_shifted          = static_cast<uint16_t>(avg_mode)          << 9;

	// Combine using "bitwise or"
	uint16_t config_value = op_mode_shifted | conv_time_current_shifted | conv_time_voltage_shifted | avg_mode_shifted;

	// Call the function to write the mask/enable register
	return this->write_register(INA260_REGISTER_CONFIG, config_value);
}

// PUBLIC FUNCTION
// GETTING THE CONFIGURATION REGISTER VALUES
bool INA260::get_configuration(INA260::Operating_Mode * op_mode, INA260::Conversion_Time * conv_time_current, INA260::Conversion_Time * conv_time_voltage, INA260::Averaging_Mode * avg_mode)
{
	// Initialise a uint16 for reading the register
	uint16_t value_as_uint16;
	// Call the read register function
	bool wasSuccessful = this->read_register(INA260_REGISTER_CONFIG, &value_as_uint16);
	// Check the status
	if (wasSuccessful)
	{
		// Extract the various parts of the unit16 value
		bool flag1 = this->operating_mode_uint16_to_enum(  (value_as_uint16 & (0x0007 << 0)) >> 0 , op_mode );
		bool flag2 = this->conversion_time_uint16_to_enum( (value_as_uint16 & (0x0007 << 3)) >> 3 , conv_time_current );
		bool flag3 = this->conversion_time_uint16_to_enum( (value_as_uint16 & (0x0007 << 6)) >> 6 , conv_time_voltage );
		bool flag4 = this->averaging_mode_uint16_to_enum(  (value_as_uint16 & (0x0007 << 9)) >> 9 , avg_mode );
		// Check that all configurations were recognised
		if (flag1 && flag2 && flag3 && flag4)
		{
			// Return flag that the operation was successful
			return true;
		}
		else
		{
			// Return flag that the operation was unsuccessful
			return false;
		}
	}
	else
	{
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool INA260::get_current_measurement_in_amps(float * value)
{
	// Initialise a uint16 for reading the register
	uint16_t value_as_uint16;
	// Call the read register function
	bool wasSuccessful = this->read_register(INA260_REGISTER_CURRENT_MEASUREMENT, &value_as_uint16);
	// Check the status
	if (wasSuccessful)
	{
		// Convert the unit16 value into the current value
		// in Volts
		*value = this->current_uint16_to_float_in_amps(value_as_uint16);
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Inform the user
		//perror("FAILED to get the current measurement.");
		// Return flag that the operation was unsuccessful
		return false;
	}
}

bool INA260::get_current_measurement_as_uint16(uint16_t * value)
{
	// Call the read register function
	bool wasSuccessful = this->read_register(INA260_REGISTER_CURRENT_MEASUREMENT, value);
	// Check the status
	if (wasSuccessful)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool INA260::get_voltage_measurement_in_volts(float * value)
{
	// Initialise a uint16 for reading the register
	uint16_t value_as_uint16;
	// Call the read register function
	bool wasSuccessful = this->read_register(INA260_REGISTER_BUS_VOLTAGE_MEASUREMENT, &value_as_uint16);
	// Check the status
	if (wasSuccessful)
	{
		// Convert the unit16 value into the voltage value
		// in Volts
		*value = this->voltage_uint16_to_float_in_volts(value_as_uint16);
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Inform the user
		//perror("FAILED to get the voltage measurement.");
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool INA260::get_power_measurement_in_watts(float * value)
{
	// Initialise a uint16 for reading the register
	uint16_t value_as_uint16;
	// Call the read register function
	bool wasSuccessful = this->read_register(INA260_REGISTER_POWER_CALCULATION, &value_as_uint16);
	// Check the status
	if (wasSuccessful)
	{
		// Convert the unit16 value into the power value
		// in Watts
		*value = this->power_uint16_to_float_in_watts(value_as_uint16);
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Inform the user
		//perror("FAILED to get the power measurement.");
		// Return flag that the operation was unsuccessful
		return false;
	}
}

// PUBLIC FUNCTION
bool INA260::get_alert_configuration_register(uint16_t * value)
{
	return this->read_register(INA260_REGISTER_MASK_ENABLE, value);
}

// PUBLIC FUNCTION
bool INA260::get_alert_limit_value_register(uint16_t * value)
{
	return this->read_register(INA260_REGISTER_ALERT_LIMIT_VALUE, value);
}

// PUBLIC FUNCTION
bool INA260::get_manufacturer_uid(uint16_t * value)
{
	return this->read_register(INA260_REGISTER_MANUFACTURER_UID, value);
}

// PUBLIC FUNCTION
bool INA260::get_die_and_revision_uid(uint16_t * value)
{
	return this->read_register(INA260_REGISTER_DIE_AND_REVISION_UID, value);
}

// CONVENIENCE FUNCTIONS
// PUBLIC FUNCTION
uint16_t INA260::conversion_time_enum_to_micro_seconds(INA260::Conversion_Time conv_time_as_emum)
{
	switch (conv_time_as_emum)
	{
		case INA260::Conversion_Time::t_0140_us:
		{
			return 140;
			break;
		}
		case INA260::Conversion_Time::t_0204_us:
		{
			return 204;
			break;
		}
		case INA260::Conversion_Time::t_0332_us:
		{
			return 332;
			break;
		}
		case INA260::Conversion_Time::t_0558_us:
		{
			return 558;
			break;
		}
		case INA260::Conversion_Time::t_1100_us:
		{
			return 1100;
			break;
		}
		case INA260::Conversion_Time::t_2116_us:
		{
			return 2116;
			break;
		}
		case INA260::Conversion_Time::t_4156_us:
		{
			return 4156;
			break;
		}
		case INA260::Conversion_Time::t_8244_us:
		{
			return 8244;
			break;
		}
		default:
		{
			return 140;
			break;
		}
	}
}

bool INA260::conversion_time_uint16_to_enum(uint16_t conv_time_as_uint16, INA260::Conversion_Time * conv_time_as_emum)
{
	switch (conv_time_as_uint16)
	{
		case static_cast<uint16_t>(INA260::Conversion_Time::t_0140_us):
		{
			*conv_time_as_emum = INA260::Conversion_Time::t_0140_us;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Conversion_Time::t_0204_us):
		{
			*conv_time_as_emum = INA260::Conversion_Time::t_0204_us;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Conversion_Time::t_0332_us):
		{
			*conv_time_as_emum = INA260::Conversion_Time::t_0332_us;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Conversion_Time::t_0558_us):
		{
			*conv_time_as_emum = INA260::Conversion_Time::t_0558_us;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Conversion_Time::t_1100_us):
		{
			*conv_time_as_emum = INA260::Conversion_Time::t_1100_us;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Conversion_Time::t_2116_us):
		{
			*conv_time_as_emum = INA260::Conversion_Time::t_2116_us;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Conversion_Time::t_4156_us):
		{
			*conv_time_as_emum = INA260::Conversion_Time::t_4156_us;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Conversion_Time::t_8244_us):
		{
			*conv_time_as_emum = INA260::Conversion_Time::t_8244_us;
			return true;
			break;
		}
		default:
		{
			return false;
			break;
		}
	}
}

// PUBLIC FUNCTION	
uint16_t INA260::averaging_mode_enum_to_samples(INA260::Averaging_Mode avg_mode_as_emum)
{
	switch (avg_mode_as_emum)
	{
		case INA260::Averaging_Mode::samples_0001:
		{
			return 1;
			break;
		}
		case INA260::Averaging_Mode::samples_0004:
		{
			return 4;
			break;
		}
		case INA260::Averaging_Mode::samples_0016:
		{
			return 16;
			break;
		}
		case INA260::Averaging_Mode::samples_0064:
		{
			return 64;
			break;
		}
		case INA260::Averaging_Mode::samples_0128:
		{
			return 128;
			break;
		}
		case INA260::Averaging_Mode::samples_0256:
		{
			return 256;
			break;
		}
		case INA260::Averaging_Mode::samples_0512:
		{
			return 512;
			break;
		}
		case INA260::Averaging_Mode::samples_1024:
		{
			return 1024;
			break;
		}
		default:
		{
			return 1;
			break;
		}
	}
}

bool INA260::averaging_mode_uint16_to_enum(uint16_t avg_mode_as_uint16, INA260::Averaging_Mode * avg_mode_as_emum)
{
	switch (avg_mode_as_uint16)
	{
		case static_cast<uint16_t>(INA260::Averaging_Mode::samples_0001):
		{
			*avg_mode_as_emum = INA260::Averaging_Mode::samples_0001;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Averaging_Mode::samples_0004):
		{
			*avg_mode_as_emum = INA260::Averaging_Mode::samples_0004;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Averaging_Mode::samples_0016):
		{
			*avg_mode_as_emum = INA260::Averaging_Mode::samples_0016;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Averaging_Mode::samples_0064):
		{
			*avg_mode_as_emum = INA260::Averaging_Mode::samples_0064;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Averaging_Mode::samples_0128):
		{
			*avg_mode_as_emum = INA260::Averaging_Mode::samples_0128;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Averaging_Mode::samples_0256):
		{
			*avg_mode_as_emum = INA260::Averaging_Mode::samples_0256;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Averaging_Mode::samples_0512):
		{
			*avg_mode_as_emum = INA260::Averaging_Mode::samples_0512;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Averaging_Mode::samples_1024):
		{
			*avg_mode_as_emum = INA260::Averaging_Mode::samples_1024;
			return true;
			break;
		}
		default:
		{
			return false;
			break;
		}
	}
}

bool INA260::operating_mode_uint16_to_enum(uint16_t op_mode_as_uint16, INA260::Operating_Mode * op_mode_as_emum)
{
	switch (op_mode_as_uint16)
	{
		case static_cast<uint16_t>(INA260::Operating_Mode::power_down):
		{
			*op_mode_as_emum = INA260::Operating_Mode::power_down;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Operating_Mode::current_triggered):
		{
			*op_mode_as_emum = INA260::Operating_Mode::current_triggered;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Operating_Mode::voltage_triggered):
		{
			*op_mode_as_emum = INA260::Operating_Mode::voltage_triggered;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Operating_Mode::current_voltage_triggered):
		{
			*op_mode_as_emum = INA260::Operating_Mode::current_voltage_triggered;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Operating_Mode::power_down_alt):
		{
			*op_mode_as_emum = INA260::Operating_Mode::power_down_alt;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Operating_Mode::current_continuous):
		{
			*op_mode_as_emum = INA260::Operating_Mode::current_continuous;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Operating_Mode::voltage_continuous):
		{
			*op_mode_as_emum = INA260::Operating_Mode::voltage_continuous;
			return true;
			break;
		}
		case static_cast<uint16_t>(INA260::Operating_Mode::current_voltage_continuous):
		{
			*op_mode_as_emum = INA260::Operating_Mode::current_voltage_continuous;
			return true;
			break;
		}
		default:
		{
			return false;
			break;
		}
	}
}

// PUBLIC FUNCTION
float INA260::current_uint16_to_float_in_amps(uint16_t current_as_uint16)
{
	// Extract the most significant bit to determine
	// the sign of the current
	uint16_t sign_bit = current_as_uint16 & 0x8000;
	// Convert the unit16 value into the current value
	// in Amps
	if (sign_bit == 0x0000)
	{
		return static_cast<float>(current_as_uint16) * INA260_CURRENT_REGISTER_LSB_UNITS_MILLIAMPS / 1000.0;
	}
	else
	{
		// Note: the current is represented in two's complement
		// format to support negative current values. Hence the
		// conversion for a negative value is:
		// 1) Compute the 15-bit representation of the positive value
		// 2) Subtract 2^15, i.e., subtract 32768
		return static_cast<float>((current_as_uint16 & 0x7FFF) - 32768)  * INA260_CURRENT_REGISTER_LSB_UNITS_MILLIAMPS / 1000.0;
	}
}

// PUBLIC FUNCTION
uint16_t INA260::current_float_in_amps_to_uint16(float current_as_float)
{
	if (current_as_float >= 0.0)
	{
		return static_cast<uint16_t>( (current_as_float * 1000.0) / INA260_CURRENT_REGISTER_LSB_UNITS_MILLIAMPS );
	}
	else
	{
		// Note: the current is represented in two's complement
		// format to support negative current values. Hence the
		// respresentation of negative values is best computes as:
		// 1) Compute the 15-bit representation of the positive value
		// 2) Apply bitwise NOT
		// 3) Add 1
		return ( ( ~( static_cast<uint16_t>( -(current_as_float * 1000.0) / INA260_CURRENT_REGISTER_LSB_UNITS_MILLIAMPS) ) ) + 1);
	}
}

// PUBLIC FUNCTION
float INA260::voltage_uint16_to_float_in_volts(uint16_t voltage_as_uint16)
{
	return ( static_cast<float>(voltage_as_uint16) * INA260_VOLTAGE_REGISTER_LSB_UNITS_MILLIVOLTS / 1000.0);
}

// PUBLIC FUNCTION
uint16_t INA260::voltage_float_in_volts_to_uint16(float voltage_as_float)
{
	return static_cast<uint16_t>( (voltage_as_float * 1000.0) / INA260_VOLTAGE_REGISTER_LSB_UNITS_MILLIVOLTS );
}

// PUBLIC FUNCTION
float INA260::power_uint16_to_float_in_watts(uint16_t power_as_uint16)
{
	return ( static_cast<float>(power_as_uint16) * INA260_POWER_REGISTER_LSB_UNITS_MILLIWATTS / 1000.0);
}

// PUBLIC FUNCTION
uint16_t INA260::power_float_in_watts_to_uint16(float power_as_float)
{
	return static_cast<uint16_t>( (power_as_float * 1000.0) / INA260_POWER_REGISTER_LSB_UNITS_MILLIWATTS );
}
