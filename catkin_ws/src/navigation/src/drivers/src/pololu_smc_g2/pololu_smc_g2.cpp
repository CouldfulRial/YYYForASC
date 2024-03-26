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
// I2C driver for the Pololu Simple Motor Controller (SMC) G2
//
// ----------------------------------------------------------------------------





#include "pololu_smc_g2/pololu_smc_g2.h"





// ----------------------------------------------------------------------------
//   CCCC   OOO   N   N   SSSS  TTTTT  RRRR   U   U   CCCC  TTTTT   OOO   RRRR
//  C      O   O  NN  N  S        T    R   R  U   U  C        T    O   O  R   R
//  C      O   O  N N N   SSS     T    RRRR   U   U  C        T    O   O  RRRR
//  C      O   O  N  NN      S    T    R   R  U   U  C        T    O   O  R   R
//   CCCC   OOO   N   N  SSSS     T    R   R   UUU    CCCC    T     OOO   R   R
// ----------------------------------------------------------------------------
Pololu_SMC_G2::Pololu_SMC_G2()
{
	this->m_i2c_address = POLOLU_SMC_G2_I2C_ADDRESS_DEFAULT;
	//this->m_i2c_driver = i2c_driver;
}

Pololu_SMC_G2::Pololu_SMC_G2(I2C_Driver * i2c_driver)
{
	this->m_i2c_address = POLOLU_SMC_G2_I2C_ADDRESS_DEFAULT;
	this->m_i2c_driver = i2c_driver;
}

Pololu_SMC_G2::Pololu_SMC_G2(I2C_Driver * i2c_driver, uint8_t address)
{
	// Check that the address is in the range [0,127]
	if (address > 127)
	{
		// Inform the user
		perror("Address supplied is greater than 127. Instead setting the address to the default of 13.");
		// Default the address to POLOLU_SMC_G2_I2C_ADDRESS_DEFAULT
		address = POLOLU_SMC_G2_I2C_ADDRESS_DEFAULT;
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

uint8_t Pololu_SMC_G2::get_i2c_address()
{
	return this->m_i2c_address;
}

bool Pololu_SMC_G2::set_i2c_address(uint8_t new_address)
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
bool Pololu_SMC_G2::get_variable(uint8_t variable_id, uint16_t * value)
{
	// Put the "get variable" command and the
	// variable id into a uint8 array
	uint8_t command_array[] = { POLOLU_SMC_G2_COMMAND_GET_VARIABLE, variable_id };
	// Specify the number of bytes in the command
	int num_command_bytes = sizeof(command_array);

	// Initialise a uint8 array for the returned
	// value of the requested variable
	// > Note that all variables are returned as
	//   a two byte response
	int num_value_bytes = 2;
	uint8_t value_array[num_value_bytes];

	// Call the i2c_driver function
	bool wasSuccessful = this->m_i2c_driver->write_data_then_read_data(this->m_i2c_address, num_command_bytes, command_array, num_value_bytes, value_array);
	// Check the status
	if (wasSuccessful)
	{
		// Convert the two unit8 values into the
		// uint16 value of the variable
		// > Note that the calling function is
		//   responsible to convert this to int16
		//   if the variable takes signed values.
		*value = value_array[0] + 256 * value_array[1];
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
bool Pololu_SMC_G2::set_motor_limit(uint8_t limit_id, uint16_t value, int * response_code )
{
	// Convert the new limit value to its two
	// byte representation
	// > The first data byte contains the low
	//   seven bits of the duty cycle
	// > The second data byte contains the high
	//   seven bits of the duty cycle.
	uint8_t value_byte_1 = value % 128;
	uint8_t value_byte_2 = value / 128;

	// Put the "set motor limit" command and new
	// limit value into a uint8 array
	uint8_t command_array[] = { POLOLU_SMC_G2_COMMAND_SET_MOTOR_LIMIT , limit_id, value_byte_1 , value_byte_2 };
	// Specify the number of bytes in the command
	int num_command_bytes = sizeof(command_array);

	// Initialise a uint8 array for the returned
	// response code
	int num_response_bytes = 1;
	uint8_t response_array[num_response_bytes];

	// Call the i2c_driver function
	bool wasSuccessful = this->m_i2c_driver->write_data_then_read_data(this->m_i2c_address, num_command_bytes, command_array, num_response_bytes, response_array);
	// Check the status
	if (wasSuccessful)
	{
		// Put the unit8 response code into
		// appropriate variable
		*response_code = response_array[0];
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Inform the user
		//perror("FAILED to set motor reverse 3200");
		// Put a "failed" response code into
		// appropriate variable
		*response_code = -1;
		// Return flag that the operation was unsuccessful
		return false;
	}
}





// DIRECT COMMANDS:
bool Pololu_SMC_G2::exit_safe_start()
{
	// Put the "exit safe start" command into a
	// uint8 array
	uint8_t command_array[] = { POLOLU_SMC_G2_COMMAND_EXIT_SAFE_START };
	// Specify the number of bytes in the command
	int num_command_bytes = sizeof(command_array);
	// Call the i2c_driver function
	bool wasSuccessful = this->m_i2c_driver->write_data(this->m_i2c_address, num_command_bytes, command_array);
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

bool Pololu_SMC_G2::motor_forward_3200(int target_duty_cycle)
{
	// Restrict the target duty cycle to be in the range
	// [0,3200]
	if (target_duty_cycle < 0)
		target_duty_cycle = 0;
	else if (target_duty_cycle > 3200)
		target_duty_cycle = 3200;

	// Convert the target duty cycle to its two byte
	// representation
	// > The first data byte contains the low
	//   five bits of the duty cycle
	// > The second data byte contains the high
	//   seven bits of the duty cycle.
	uint8_t target_duty_cycle_byte_1 = target_duty_cycle % 32;
	uint8_t target_duty_cycle_byte_2 = target_duty_cycle / 32;

	// Put the "motor forward" command and target
	// duty cycle into a uint8 array
	uint8_t command_array[] = { POLOLU_SMC_G2_COMMAND_MOTOR_FORWARD , target_duty_cycle_byte_1 , target_duty_cycle_byte_2 };
	// Specify the number of bytes in the command
	int num_command_bytes = sizeof(command_array);
	// Call the i2c_driver function
	bool wasSuccessful = this->m_i2c_driver->write_data(this->m_i2c_address, num_command_bytes, command_array);
	// Check the status
	if (wasSuccessful)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Inform the user
		//perror("FAILED to set motor forward 3200");
		// Return flag that the operation was unsuccessful
		return false;
	}
}

bool Pololu_SMC_G2::motor_forward_percent(int target_duty_cycle)
{
	// Restrict the target duty cycle to be in the range
	// [0,100]
	if (target_duty_cycle < 0)
		target_duty_cycle = 0;
	else if (target_duty_cycle > 100)
		target_duty_cycle = 100;

	// Put the "motor forward" command and target
	// duty cycle into a uint8 array
	uint8_t command_array[] = { POLOLU_SMC_G2_COMMAND_MOTOR_FORWARD , 0 , (uint8_t)target_duty_cycle };
	// Specify the number of bytes in the command
	int num_command_bytes = sizeof(command_array);
	// Call the i2c_driver function
	bool wasSuccessful = this->m_i2c_driver->write_data(this->m_i2c_address, num_command_bytes, command_array);
	// Check the status
	if (wasSuccessful)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Inform the user
		//perror("FAILED to set motor forward percent");
		// Return flag that the operation was unsuccessful
		return false;
	}
}

bool Pololu_SMC_G2::motor_forward_7bit(int target_duty_cycle)
{
	// Restrict the target duty cycle to be in the range
	// [0,127]
	if (target_duty_cycle < 0)
		target_duty_cycle = 0;
	else if (target_duty_cycle > 127)
		target_duty_cycle = 127;

	// Put the "motor forward 7-bit" command and
	// target duty cycle into a uint8 array
	uint8_t command_array[] = { POLOLU_SMC_G2_COMMAND_MOTOR_FORWARD_7BIT , 0 , (uint8_t)target_duty_cycle };
	// Specify the number of bytes in the command
	int num_command_bytes = sizeof(command_array);
	// Call the i2c_driver function
	bool wasSuccessful = this->m_i2c_driver->write_data(this->m_i2c_address, num_command_bytes, command_array);
	// Check the status
	if (wasSuccessful)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Inform the user
		//perror("FAILED to set motor forward 7-bit");
		// Return flag that the operation was unsuccessful
		return false;
	}
}

bool Pololu_SMC_G2::motor_reverse_3200(int target_duty_cycle)
{
	// Restrict the target duty cycle to be in the range
	// [0,3200]
	if (target_duty_cycle < 0)
		target_duty_cycle = 0;
	else if (target_duty_cycle > 3200)
		target_duty_cycle = 3200;

	// Convert the target duty cycle to its two byte
	// representation
	// > The first data byte contains the low
	//   five bits of the duty cycle
	// > The second data byte contains the high
	//   seven bits of the duty cycle.
	uint8_t target_duty_cycle_byte_1 = target_duty_cycle % 32;
	uint8_t target_duty_cycle_byte_2 = target_duty_cycle / 32;

	// Put the "motor reverse" command and target
	// duty cycle into a uint8 array
	uint8_t command_array[] = { POLOLU_SMC_G2_COMMAND_MOTOR_REVERSE , target_duty_cycle_byte_1 , target_duty_cycle_byte_2 };
	// Specify the number of bytes in the command
	int num_command_bytes = sizeof(command_array);
	// Call the i2c_driver function
	bool wasSuccessful = this->m_i2c_driver->write_data(this->m_i2c_address, num_command_bytes, command_array);
	// Check the status
	if (wasSuccessful)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Inform the user
		//perror("FAILED to set motor reverse 3200");
		// Return flag that the operation was unsuccessful
		return false;
	}
}

bool Pololu_SMC_G2::motor_reverse_percent(int target_duty_cycle)
{
	// Restrict the target duty cycle to be in the range
	// [0,100]
	if (target_duty_cycle < 0)
		target_duty_cycle = 0;
	else if (target_duty_cycle > 100)
		target_duty_cycle = 100;

	// Put the "motor reverse" command and target
	// duty cycle into a uint8 array
	uint8_t command_array[] = { POLOLU_SMC_G2_COMMAND_MOTOR_REVERSE , 0 , (uint8_t)target_duty_cycle };
	// Specify the number of bytes in the command
	int num_command_bytes = sizeof(command_array);
	// Call the i2c_driver function
	bool wasSuccessful = this->m_i2c_driver->write_data(this->m_i2c_address, num_command_bytes, command_array);
	// Check the status
	if (wasSuccessful)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Inform the user
		//perror("FAILED to set motor reverse percent");
		// Return flag that the operation was unsuccessful
		return false;
	}
}

bool Pololu_SMC_G2::motor_reverse_7bit(int target_duty_cycle)
{
	// Restrict the target duty cycle to be in the range
	// [0,127]
	if (target_duty_cycle < 0)
		target_duty_cycle = 0;
	else if (target_duty_cycle > 127)
		target_duty_cycle = 127;

	// Put the "motor reverse 7-bit" command and
	// target duty cycle into a uint8 array
	uint8_t command_array[] = { POLOLU_SMC_G2_COMMAND_MOTOR_REVERSE_7BIT , 0 , (uint8_t)target_duty_cycle };
	// Specify the number of bytes in the command
	int num_command_bytes = sizeof(command_array);
	// Call the i2c_driver function
	bool wasSuccessful = this->m_i2c_driver->write_data(this->m_i2c_address, num_command_bytes, command_array);
	// Check the status
	if (wasSuccessful)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Inform the user
		//perror("FAILED to set motor reverse 7-bit");
		// Return flag that the operation was unsuccessful
		return false;
	}
}

bool Pololu_SMC_G2::motor_brake(int brake_amount)
{
	// Restrict the brake amount to be in the range
	// [0,32]
	if (brake_amount < 0)
		brake_amount = 0;
	else if (brake_amount > 32)
		brake_amount = 32;

	// Put the "motor brake" command and brake
	// amount into a uint8 array
	uint8_t command_array[] = { POLOLU_SMC_G2_COMMAND_MOTOR_BRAKE , (uint8_t)brake_amount };
	// Specify the number of bytes in the command
	int num_command_bytes = sizeof(command_array);
	// Call the i2c_driver function
	bool wasSuccessful = this->m_i2c_driver->write_data(this->m_i2c_address, num_command_bytes, command_array);
	// Check the status
	if (wasSuccessful)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Inform the user
		//perror("FAILED to set motor brake");
		// Return flag that the operation was unsuccessful
		return false;
	}
}

bool Pololu_SMC_G2::stop_motor()
{
	// Put the "stop motor" command into a
	// uint8 array
	uint8_t command_array[] = { POLOLU_SMC_G2_COMMAND_STOP_MOTOR };
	// Specify the number of bytes in the command
	int num_command_bytes = sizeof(command_array);
	// Call the i2c_driver function
	bool wasSuccessful = this->m_i2c_driver->write_data(this->m_i2c_address, num_command_bytes, command_array);
	// Check the status
	if (wasSuccessful)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Inform the user
		//perror("FAILED to stop motor.");
		// Return flag that the operation was unsuccessful
		return false;
	}
}





// CONVENIENCE FUNCTIONS FOR SETTING
// A SIGNED MOTOR TARGET DUTY CYCLE
bool Pololu_SMC_G2::set_motor_target_duty_cycle_3200(int target_duty_cycle)
{
	if (target_duty_cycle<0)
		return this->motor_reverse_3200(-target_duty_cycle);
	else
		return this->motor_forward_3200(target_duty_cycle);
}

bool Pololu_SMC_G2::set_motor_target_duty_cycle_percent(int target_duty_cycle)
{
	if (target_duty_cycle<0)
		return this->motor_reverse_percent(-target_duty_cycle);
	else
		return this->motor_forward_percent(target_duty_cycle);
}

bool Pololu_SMC_G2::set_motor_target_duty_cycle_percent(float target_duty_cycle)
{
	// Convert the percent to the nearest 3200 integer
	int target_duty_cycle_3200 = static_cast<int>( std::round(target_duty_cycle * 32.0f) );
	// Call the function for signed 3200 duty cycles
	return this->set_motor_target_duty_cycle_3200(target_duty_cycle_3200);
}

bool Pololu_SMC_G2::set_motor_target_duty_cycle_7bit(int target_duty_cycle)
{
	if (target_duty_cycle<0)
		return this->motor_reverse_7bit(-target_duty_cycle);
	else
		return this->motor_forward_7bit(target_duty_cycle);
}





// SET MOTOR LIMITS
// > For setting forward and reverse limits at the same time
bool Pololu_SMC_G2::set_motor_limit_max_duty_cycle(int new_max_duty_cycle, int * response_code)
{
	// Restrict the new max dutycycle to be in the
	// range [0,3200]
	if (new_max_duty_cycle < 0)
		new_max_duty_cycle = 0;
	else if (new_max_duty_cycle > 3200)
		new_max_duty_cycle = 3200;
	// Call the function that set the limit
	return this->set_motor_limit(POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_SPEED, new_max_duty_cycle, response_code);
}

bool Pololu_SMC_G2::set_motor_limit_max_acceleration(int new_max_acceleration, int * response_code)
{
	// Restrict the new max acceleration to be in the
	// range [0,3200]
	// Note: a limit of 0 mean no limit
	// Note: this is a limit in the change of duty cycle
	//       per "speed update period"
	// Note: the "speed update period" is set via the
	//       windows-based Pololu SMC G2 software
	//       > Default Value   = 1 ms
	//       > Possible Values = {1,5,10,25,50,100} ms
	if (new_max_acceleration < 0)
		new_max_acceleration = 0;
	else if (new_max_acceleration > 3200)
		new_max_acceleration = 3200;
	// Call the function that set the limit
	return this->set_motor_limit(POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_ACCELERATION, new_max_acceleration, response_code);
}

bool Pololu_SMC_G2::set_motor_limit_max_deceleration(int new_max_deceleration, int * response_code)
{
	// Restrict the new max deceleration to be in the
	// range [0,3200]
	// Note: a limit of 0 mean no limit
	// Note: this is a limit in the change of duty cycle
	//       per "speed update period"
	// Note: the "speed update period" is set via the
	//       windows-based Pololu SMC G2 software
	//       > Default Value   = 1 ms
	//       > Possible Values = {1,5,10,25,50,100} ms
	if (new_max_deceleration < 0)
		new_max_deceleration = 0;
	else if (new_max_deceleration > 3200)
		new_max_deceleration = 3200;
	// Call the function that set the limit
	return this->set_motor_limit(POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_DECELERATION, new_max_deceleration, response_code);
}

bool Pololu_SMC_G2::set_motor_limit_max_brake_duration(int new_max_brake_duration, int * response_code)
{
	// Restrict the new max brake duration to be in the
	// range [0,16383]
	if (new_max_brake_duration < 0)
		new_max_brake_duration = 0;
	else if (new_max_brake_duration > 16383)
		new_max_brake_duration = 16383;
	// Call the function that set the limit
	return this->set_motor_limit(POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_BRAKE_DURATION, new_max_brake_duration, response_code);
}

// > For setting forward limits
bool Pololu_SMC_G2::set_motor_limit_max_duty_cycle_forward(int new_max_duty_cycle, int * response_code)
{
	// Restrict the new max duty cycle to be in the
	// range [0,3200]
	if (new_max_duty_cycle < 0)
		new_max_duty_cycle = 0;
	else if (new_max_duty_cycle > 3200)
		new_max_duty_cycle = 3200;
	// Call the function that set the limit
	return this->set_motor_limit(POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_SPEED_FORWARD, new_max_duty_cycle, response_code);
}

bool Pololu_SMC_G2::set_motor_limit_max_acceleration_forward(int new_max_acceleration, int * response_code)
{
	// Restrict the new max acceleration to be in the
	// range [0,3200]
	// Note: a limit of 0 mean no limit
	if (new_max_acceleration < 0)
		new_max_acceleration = 0;
	else if (new_max_acceleration > 3200)
		new_max_acceleration = 3200;
	// Call the function that set the limit
	return this->set_motor_limit(POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_ACCELERATION_FORWARD, new_max_acceleration, response_code);
}

bool Pololu_SMC_G2::set_motor_limit_max_deceleration_forward(int new_max_deceleration, int * response_code)
{
	// Restrict the new max deceleration to be in the
	// range [0,3200]
	// Note: a limit of 0 mean no limit
	if (new_max_deceleration < 0)
		new_max_deceleration = 0;
	else if (new_max_deceleration > 3200)
		new_max_deceleration = 3200;
	// Call the function that set the limit
	return this->set_motor_limit(POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_DECELERATION_FORWARD, new_max_deceleration, response_code);
}

bool Pololu_SMC_G2::set_motor_limit_max_brake_duration_forward(int new_max_brake_duration, int * response_code)
{
	// Restrict the new max brake duration to be in the
	// range [0,16383]
	if (new_max_brake_duration < 0)
		new_max_brake_duration = 0;
	else if (new_max_brake_duration > 16383)
		new_max_brake_duration = 16383;
	// Call the function that set the limit
	return this->set_motor_limit(POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_BRAKE_DURATION_FORWARD, new_max_brake_duration, response_code);
}

// > For setting reverse limits
bool Pololu_SMC_G2::set_motor_limit_max_duty_cycle_reverse(int new_max_duty_cycle, int * response_code)
{
	// Restrict the new max duty cycle to be in the
	// range [0,3200]
	if (new_max_duty_cycle < 0)
		new_max_duty_cycle = 0;
	else if (new_max_duty_cycle > 3200)
		new_max_duty_cycle = 3200;
	// Call the function that set the limit
	return this->set_motor_limit(POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_SPEED_REVERSE, new_max_duty_cycle, response_code);
}

bool Pololu_SMC_G2::set_motor_limit_max_acceleration_reverse(int new_max_acceleration, int * response_code)
{
	// Restrict the new max acceleration to be in the
	// range [0,3200]
	// Note: a limit of 0 mean no limit
	if (new_max_acceleration < 0)
		new_max_acceleration = 0;
	else if (new_max_acceleration > 3200)
		new_max_acceleration = 3200;
	// Call the function that set the limit
	return this->set_motor_limit(POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_ACCELERATION_REVERSE, new_max_acceleration, response_code);
}

bool Pololu_SMC_G2::set_motor_limit_max_deceleration_reverse(int new_max_deceleration, int * response_code)
{
	// Restrict the new max deceleration to be in the
	// range [0,3200]
	// Note: a limit of 0 mean no limit
	if (new_max_deceleration < 0)
		new_max_deceleration = 0;
	else if (new_max_deceleration > 3200)
		new_max_deceleration = 3200;
	// Call the function that set the limit
	return this->set_motor_limit(POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_DECELERATION_REVERSE, new_max_deceleration, response_code);
}

bool Pololu_SMC_G2::set_motor_limit_max_brake_duration_reverse(int new_max_brake_duration, int * response_code)
{
	// Restrict the new max brake duration to be in the
	// range [0,16383]
	if (new_max_brake_duration < 0)
		new_max_brake_duration = 0;
	else if (new_max_brake_duration > 16383)
		new_max_brake_duration = 16383;
	// Call the function that set the limit
	return this->set_motor_limit(POLOLU_SMC_G2_SET_MOTOR_LIMIT_MAX_BRAKE_DURATION_REVERSE, new_max_brake_duration, response_code);
}





// SET THE CURRENT LIMIT
bool Pololu_SMC_G2::set_current_limit_in_internal_units(int new_current_limit)
{
	// Restrict the new current limit to be in the
	// range [0,16380]
	// > Note: The upper limit is (2^14-1) rounded down to
	//   the nearest 10
	if (new_current_limit < 0)
		new_current_limit = 0;
	else if (new_current_limit > 16380)
		new_current_limit = 16380;

	// Convert the new current limit to its two
	// byte representation
	// > The first data byte contains the low
	//   seven bits of the current
	// > The second data byte contains the high
	//   seven bits of the current.
	uint8_t new_current_limit_byte_1 = new_current_limit % 128;
	uint8_t new_current_limit_byte_2 = new_current_limit / 128;

	// Put the "set current limit" command and new
	// current limit into a uint8 array
	uint8_t command_array[] = { POLOLU_SMC_G2_COMMAND_SET_CURRENT_LIMIT , new_current_limit_byte_1 , new_current_limit_byte_2 };
	// Specify the number of bytes in the command
	int num_command_bytes = sizeof(command_array);
	// Call the i2c_driver function
	bool wasSuccessful = this->m_i2c_driver->write_data(this->m_i2c_address, num_command_bytes, command_array);
	// Check the status
	if (wasSuccessful)
	{
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Inform the user
		//perror("FAILED to set motor reverse 3200");
		// Return flag that the operation was unsuccessful
		return false;
	}
}

bool Pololu_SMC_G2::set_current_limit_in_milliamps(int new_current_limit)
{
	// Conversion steps copied from the Pololu documentation:
	// 1) Take the desired current limit in units of milliamps and multiply it by 3200.
	// 2) Multiply this number by the following number, which depends on the Simple Motor Controller G2 model: 2 for the 18v15, 3 for the 24v12, 1 for the 18v25, or 2 for the 24v19.
	// 3) Divide by the current scale calibration setting (8057 by default).
	// 4) Add the current offset calibration setting (993 by default).
	// 5) Multiply by 3200 and then divide by 65536 to get the final result.

	// For the 18v15 model and default values, this is:
	// [internal units] = ([mA] * 3200 * 2 / 8057 + 993) * 3200/65536
	//                  = ([mA] * 0.794340 + 993) * 0.0488281
	// For example:
	//  1000 [mA] =   87.3 [internal units]
	//  2000 [mA] =  126.1 [internal units]
	//  5000 [mA] =  242.4 [internal units]
	// 10000 [mA] =  436.3 [internal units]
	// 15000 [mA] =  630.3 [internal units]
	// 25000 [mA] = 1018.1 [internal units]

	// Restrict the new current limit to be in the
	// range [0,25000] milliamps
	if (new_current_limit < 0)
		new_current_limit = 0;
	else if (new_current_limit > 25000)
		new_current_limit = 25000;

	// Convert from milliamps to internal units
	int new_limit_in_internal_units = ( float(new_current_limit) * 0.794340 + 993.0 ) * 0.0488281;

	// Call the function for setting the current limit
	// in internal units
	return this->set_current_limit_in_internal_units(new_limit_in_internal_units);
}





// GET THE FIRMWARE VERSION
bool Pololu_SMC_G2::get_firmware_version(uint16_t * product_id, uint8_t * firmware_major , uint8_t * firmware_minor)
{
	// Put the "get firmware version" command
	// into a uint8 array
	uint8_t command_array[] = { POLOLU_SMC_G2_COMMAND_GET_FIRMWARE_VERSION };
	// Specify the number of bytes in the command
	int num_command_bytes = sizeof(command_array);

	// Initialise a uint8 array for the returned
	// value of the firmware version
	// > Note that this command returns 4 bytes
	//   of data
	//   Byte 1: product ID low byte
	//   Byte 2: product ID high byte
	//   Byte 3: minor FW version (BCD format)
	//   Byte 4: major FW version (BCD format)
	int num_value_bytes = 4;
	uint8_t value_array[num_value_bytes];

	// Call the i2c_driver function
	bool wasSuccessful = this->m_i2c_driver->write_data_then_read_data(this->m_i2c_address, num_command_bytes, command_array, num_value_bytes, value_array);
	// Check the status
	if (wasSuccessful)
	{
		// Convert the two unit8 values into the
		// uint16 value of the product id
		*product_id = value_array[0] + 256 * value_array[1];
		// Set the value of the firmware version
		*firmware_major = value_array[3];
		*firmware_minor = value_array[2];
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





// GET VARIABLES
// > For the status flag registers

// Gets a number where each bit represents a different
// error, and the bit is 1 if the error is currently
// active. See the user's guide for definitions of the
// different error bits.
bool Pololu_SMC_G2::get_error_status(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_ERROR_STATUS, value);
}

bool Pololu_SMC_G2::get_error_occurred(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_ERRORS_OCCURRED, value);
}

bool Pololu_SMC_G2::get_serial_erros_occurred(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_SERIAL_ERRORS_OCCURRED, value);
}

bool Pololu_SMC_G2::get_limit_status(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_LIMIT_STATUS, value);
}

bool Pololu_SMC_G2::get_reset_flags(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_RESET_FLAGS, value);
}

// > For the RC channels
bool Pololu_SMC_G2::get_rc1_unlimited_raw_value(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_RC1_UNLIMITED_RAW_VALUE, value);
}

bool Pololu_SMC_G2::get_rc1_raw_value(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_RC1_RAW_VALUE, value);
}

bool Pololu_SMC_G2::get_rc1_scaled_value(int16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_RC1_SCALED_VALUE, (uint16_t *)value);
}

bool Pololu_SMC_G2::get_rc2_unlimited_raw_value(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_RC2_UNLIMITED_RAW_VALUE, value);
}

bool Pololu_SMC_G2::get_rc2_raw_value(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_RC2_RAW_VALUE, value);
}

bool Pololu_SMC_G2::get_rc2_scaled_value(int16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_RC2_SCALED_VALUE, (uint16_t *)value);
}





// > For the analog channels
bool Pololu_SMC_G2::get_an1_unlimited_raw_value(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_AN1_UNLIMITED_RAW_VALUE, value);
}

bool Pololu_SMC_G2::get_an1_raw_value(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_AN1_RAW_VALUE, value);
}

bool Pololu_SMC_G2::get_an1_scaled_value(int16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_AN1_SCALED_VALUE, (uint16_t *)value);
}

bool Pololu_SMC_G2::get_an2_unlimited_raw_value(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_AN2_UNLIMITED_RAW_VALUE, value);
}

bool Pololu_SMC_G2::get_an2_raw_value(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_AN2_RAW_VALUE, value);
}

bool Pololu_SMC_G2::get_an2_scaled_value(int16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_AN2_SCALED_VALUE, (uint16_t *)value);
}





// > For diagnostic variables
// Gets the target duty cycle (-3200 to 3200).
bool Pololu_SMC_G2::get_target_duty_cycle_3200(int16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_TARGET_SPEED, (uint16_t *)value);
}

bool Pololu_SMC_G2::get_duty_cycle_3200(int16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_SPEED, (uint16_t *)value);
}

bool Pololu_SMC_G2::get_brake_amount(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_BRAKE_AMOUNT, value);
}

bool Pololu_SMC_G2::get_input_voltage_in_volts(float * value)
{
	uint16_t value_as_int;
	bool returnSuccess = this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_INPUT_VOLTAGE, &value_as_int);
	*value = float(value_as_int) * 0.001f;
	return returnSuccess;
}

// Gets the temperature at point A (0 to 299.9).
// > Note 300.0 indicates an error measuring the
//   temperature
bool Pololu_SMC_G2::get_temperature_a(float * value)
{
	uint16_t value_as_int;
	bool returnSuccess = this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_TEMPERATURE_A, &value_as_int);
	*value = float(value_as_int) * 0.1f;
	return returnSuccess;
}

// Gets the temperature at point B (0 to 299.9).
// > Note 300.0 indicates an error measuring the
//   temperature
bool Pololu_SMC_G2::get_temperature_b(float * value)
{
	uint16_t value_as_int;
	bool returnSuccess = this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_TEMPERATURE_B, &value_as_int);
	*value = float(value_as_int) * 0.1f;
	return returnSuccess;
}

bool Pololu_SMC_G2::get_rc_period_in_seconds(float * value)
{
	uint16_t value_as_int;
	bool returnSuccess = this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_RC_PERIOD, &value_as_int);
	*value = float(value_as_int) * 0.1f;
	return returnSuccess;
}

bool Pololu_SMC_G2::get_baud_rate_register_in_bps(float * value)
{
	uint16_t value_as_int;
	bool returnSuccess = this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_BAUD_RATE_REGISTER, &value_as_int);
	if (value_as_int == 0)
	{
		*value = 0.0f;
	}
	else
	{
		*value = 72000000.0f / float(value_as_int);
	}
	return returnSuccess;
}

// Gets the number of milliseconds since last reset or power up (0 to 65535).
bool Pololu_SMC_G2::get_up_time_low(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_UP_TIME_LOW, value);
}

// Gets the number of 65536 millisecond blocks since last reset or power up (0 to 65535).
bool Pololu_SMC_G2::get_up_time_high(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_UP_TIME_HIGH, value);
}

// Gets the number of seconds since last reset or power up
bool Pololu_SMC_G2::get_up_time_in_seconds(float * value)
{
	// Get the up time low
	uint16_t up_time_low;
	bool success_for_up_time_low = this->get_up_time_low(&up_time_low);
	// Get the up time high
	uint16_t up_time_high;
	bool success_for_up_time_high = this->get_up_time_high(&up_time_high);

	if (success_for_up_time_low && success_for_up_time_high)
	{
		// Compute the up time in seconds
		*value = float(up_time_low)*0.001f + float(up_time_high)*65.536f;
		// Return flag that the operation was successful
		return true;
	}
	else
	{
		// Set the value to an "error value"
		*value = -1.0f;
		// Return flag that the operation was unsuccessful
		return false;
	}
}





// > For motor duty cycle limits (forward)
bool Pololu_SMC_G2::get_max_duty_cycle_forward(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_MAX_SPEED_FORWARD, value);
}

bool Pololu_SMC_G2::get_max_acceleration_forward(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_MAX_ACCELERATION_FORWARD, value);
}

bool Pololu_SMC_G2::get_max_deceleration_forward(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_MAX_DECELERATION_FORWARD, value);
}

bool Pololu_SMC_G2::get_brake_duration_forward(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_BRAKE_DURATION_FORWARD, value);
}

bool Pololu_SMC_G2::get_brake_duration_forward_in_seconds(float * value)
{
	uint16_t value_as_int;
	bool returnSuccess = this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_BRAKE_DURATION_FORWARD, &value_as_int);
	*value = float(value_as_int) * 0.001f;
	return returnSuccess;
}

bool Pololu_SMC_G2::get_starting_duty_cycle_forward(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_STARTING_SPEED_FORWARD, value);
}





// > For motor duty cycle limits (reverse)
bool Pololu_SMC_G2::get_max_duty_cycle_reverse(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_MAX_SPEED_REVERSE, value);
}

bool Pololu_SMC_G2::get_max_acceleration_reverse(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_MAX_ACCELERATION_REVERSE, value);
}

bool Pololu_SMC_G2::get_max_deceleration_reverse(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_MAX_DECELERATION_REVERSE, value);
}

bool Pololu_SMC_G2::get_brake_duration_reverse(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_BRAKE_DURATION_REVERSE, value);
}

bool Pololu_SMC_G2::get_brake_duration_reverse_in_seconds(float * value)
{
	uint16_t value_as_int;
	bool returnSuccess = this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_BRAKE_DURATION_REVERSE, &value_as_int);
	*value = float(value_as_int) * 0.001f;
	return returnSuccess;
}

bool Pololu_SMC_G2::get_starting_duty_cycle_reverse(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_STARTING_SPEED_REVERSE, value);
}





// > For current limiting and measurement
bool Pololu_SMC_G2::get_current_limit(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_CURRENT_LIMIT, value);
}

bool Pololu_SMC_G2::get_raw_current(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_RAW_CURRENT, value);
}

bool Pololu_SMC_G2::get_current_in_milliamps(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_CURRENT, value);
}

bool Pololu_SMC_G2::get_current_limiting_consecutive_count(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_CURRENT_LIMITING_CONSECUTIVE_COUNT, value);
}

bool Pololu_SMC_G2::get_current_limiting_occurrence_count(uint16_t * value)
{
	return this->get_variable(POLOLU_SMC_G2_GET_VARIABLE_CURRENT_LIMITING_OCCURRENCE_COUNT, value);
}





// CONVENIENCE FUNCTIONS
bool Pololu_SMC_G2::initialise_with_limits(int new_current_limit_in_milliamps, int new_max_duty_cycle_limit, int new_max_accel_limit, int new_max_decel_limit, bool verbose)
{
	// Initialise a boolean variable for the result
	// of calls to functions
	bool result;

	// Initialise a boolean variable for the overall
	// result to return
	bool return_result = true;

	// Send the "exit safe start" command
	result = this->exit_safe_start();
	if (verbose)
	{
		if (!result)
			std::cout << "POLOLU SMC G2 DRIVER: FAILED - exit safe start NOT successful for I2C address " << static_cast<int>(this->get_i2c_address());
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// > Check the status flag registers
	uint16_t error_status;
	result = this->get_error_status(&error_status);
	if (verbose)
	{
		if (result)
			std::cout << "POLOLU SMC G2 DRIVER: get error status returned: " << std::bitset<16>(error_status) << ", for I2C address " << static_cast<int>(this->get_i2c_address());
		else
			std::cout << "POLOLU SMC G2 DRIVER: FAILED - get error status NOT successful for I2C address " << static_cast<int>(this->get_i2c_address());
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// Short sleep
	usleep(1000);

	// > Check the input voltage
	float input_voltage_value;
	result = this->get_input_voltage_in_volts(&input_voltage_value);
	if (verbose)
	{
		if (result)
			std::cout << "POLOLU SMC G2 DRIVER: get input voltage value returned: " << input_voltage_value << " [Volts], for I2C address " << static_cast<int>(this->get_i2c_address());
		else
			std::cout << "POLOLU SMC G2 DRIVER: FAILED - get input voltage value NOT successful for I2C address " << static_cast<int>(this->get_i2c_address());
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// Short sleep
	usleep(1000);

	// Set the current limit
	result = this->set_current_limit_in_milliamps(new_current_limit_in_milliamps);
	if (verbose)
	{
		if (!result)
			std::cout << "POLOLU SMC G2 DRIVER: FAILED - set current limit NOT successful for I2C address " << static_cast<int>(this->get_i2c_address());
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// Check the current limit that was set
	uint16_t current_limit_value;
	result = this->get_current_limit(&current_limit_value);
	if (verbose)
	{
		if (result)
			std::cout << "POLOLU SMC G2 DRIVER: get current limit returned: " << current_limit_value << ", for I2C address " << static_cast<int>(this->get_i2c_address());
		else
			std::cout << "POLOLU SMC G2 DRIVER: FAILED - get current limit NOT successful for I2C address " << static_cast<int>(this->get_i2c_address());
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// Short sleep
	usleep(1000);

	// Send the max duty cycle limit
	int max_duty_cycle_limit_response_code;
	result = this->set_motor_limit_max_duty_cycle(new_max_duty_cycle_limit, &max_duty_cycle_limit_response_code);
	if (verbose)
	{
		if (!result)
			std::cout << "POLOLU SMC G2 DRIVER: FAILED - set max duty cycle limit NOT successful with response code " << max_duty_cycle_limit_response_code << ", for I2C address " << static_cast<int>(this->get_i2c_address());
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// Check the max duty cycle limit that was set
	uint16_t max_duty_cycle_limit_value;
	result = this->get_max_duty_cycle_forward(&max_duty_cycle_limit_value);
	if (verbose)
	{
		if (result)
			std::cout << "POLOLU SMC G2 DRIVER: get max duty cycle limit returned: " << max_duty_cycle_limit_value << ", for I2C address " << static_cast<int>(this->get_i2c_address());
		else
			std::cout << "POLOLU SMC G2 DRIVER: FAILED - get max duty cycle limit NOT successful for I2C address " << static_cast<int>(this->get_i2c_address());
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// Short sleep
	usleep(1000);

	// Set the max acceleration limit
	int max_accel_limit_response_code;
	result = this->set_motor_limit_max_acceleration(new_max_accel_limit, &max_accel_limit_response_code);
	if (verbose)
	{
		if (!result)
			std::cout << "POLOLU SMC G2 DRIVER: FAILED - set max acceleration limit NOT successful with response code " << max_accel_limit_response_code << " for I2C address " << static_cast<int>(this->get_i2c_address());
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// Check the max duty cycle acceleration that was set
	uint16_t max_accel_limit_value;
	result = this->get_max_acceleration_forward(&max_accel_limit_value);
	if (verbose)
	{
		if (result)
			std::cout << "POLOLU SMC G2 DRIVER: get max acceleration limit returned: " << max_accel_limit_value << ", for I2C address " << static_cast<int>(this->get_i2c_address());
		else
			std::cout << "POLOLU SMC G2 DRIVER: FAILED - get max acceleration limit NOT successful for I2C address " << static_cast<int>(this->get_i2c_address());
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// Short sleep
	usleep(1000);

	// Set the max deceleration limit
	int max_decel_limit_response_code;
	result = this->set_motor_limit_max_deceleration(new_max_decel_limit, &max_decel_limit_response_code);
	if (verbose)
	{
		if (!result)
			std::cout << "POLOLU SMC G2 DRIVER: FAILED - set max deceleration limit NOT successful with response code " << max_decel_limit_response_code << " for I2C address " << static_cast<int>(this->get_i2c_address());
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// > Check the max duty cycle deceleration that was set
	uint16_t max_decel_limit_value;
	result = this->get_max_deceleration_forward(&max_decel_limit_value);
	if (verbose)
	{
		if (result)
			std::cout << "POLOLU SMC G2 DRIVER: get max deceleration limit returned: " << max_decel_limit_value << ", for I2C address " << static_cast<int>(this->get_i2c_address());
		else
			std::cout << "POLOLU SMC G2 DRIVER: FAILED - get max deceleration limit NOT successful for I2C address " << static_cast<int>(this->get_i2c_address());
	}
	// Update the "cumulative" result
	return_result = (return_result && result);

	// Short sleep
	usleep(1000);

	if (verbose)
	{
		std::cout << "POLOLU SMC G2 DRIVER: Finished setting up the Pololu SMC with I2C address " << static_cast<int>(this->get_i2c_address());
	}

	// Return the "cumulative" result
	return return_result;
}
