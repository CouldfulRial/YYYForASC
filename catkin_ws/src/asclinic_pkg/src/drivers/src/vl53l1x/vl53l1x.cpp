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
// I2C driver for the VL53L1X time-of-flight distance sensor
//
// ----------------------------------------------------------------------------

// NOTES - ON HOW THE API FROM ST IS USED:
// > The API provided by ST uses the variable "uint16_t dev" throughout the
//   code.
// > This "dev" variable is not actually used by any of the functions, it is
//   only passed along to every function that is called.
// > For example, a call to the following function, implemented in
//   VL53L1X_api.{h,c}:
//     "VL53L1X_ERROR VL53L1X_GetDistance(uint16_t dev, uint16_t *distance);"
//   will subsequently call the following function, implmented in
//   vl53l1_platform.{h,c}:
//     "int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata)"
//   which will subequently call the following function, also implemented in
//   vl53l1_platform.{h,c}:
//     "int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t register_address, uint8_t *write_data_array, uint16_t num_write_btyes);"
//   and this last function makes the calls to "ioctl(...)" to implement
//   writing over the I2C bus to the VL53L1X sensor.
// > The equivalent final function for reading data over the I2C bus from
//   the VL53L1X sensor is the following, also implemented in
//   vl53l1_platform.{h,c}:
//     "int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t register_address, uint8_t *read_data_array, uint16_t num_read_btyes);"
// > The "dev" variable is be used for passing the file descriptor of the
//   appropriate I2C device through to the functions within
//   vl53l1_platform.{h,c} that implement the calls to "ioctl(...)".
//
// NOTES - FOR MULTIPLE VL53L1X SENSORS ON THE SAME I2C BUS:
// > In order to manage multiple VL53L1X I2C sensors on the same I2C bus,
//   an I2C multiplexer is required.
// > The "dev" variable is used ...
//
// NOTES - ON THE TYPE "VL53L1X_ERROR"
// > The type "VL53L1X_ERROR" is defined in VL53L1X_api.h:
//     "typedef int8_t VL53L1X_ERROR;"
// > This is the return type used in most of the function 
//   implemented in vl53l1_platform.{h,c} and VL53L1X_api.{h,c}
// > The convention used is:
//   >  0 is success
//   > -1 is error
// > The convention used for functions in this class:
//   > true  is success
//   > false is error
//
// NOTES - ON READ/WRITE VIA THE "m_i2c_driver" MEMBER VARIABLE
// > The following two function read to and write from the VL53L1X sensor
//   over the I2C bus via the "m_i2c_driver" member variable of this
//   class:
//     "bool VL53L1X::read_register(...)"
//     "bool VL53L1X::write_register(...)"
// > Use of these functions is discouraged because ST does not provide
//   documentation of the registers of the VL53L1X sensor.
// > These functions are included for completeness should a unique
//   circumstance arise that justifies bypassing of the API functions
//   provided by ST.


#include "vl53l1x/vl53l1x.h"





// ----------------------------------------------------------------------------
//   CCCC   OOO   N   N   SSSS  TTTTT  RRRR   U   U   CCCC  TTTTT   OOO   RRRR
//  C      O   O  NN  N  S        T    R   R  U   U  C        T    O   O  R   R
//  C      O   O  N N N   SSS     T    RRRR   U   U  C        T    O   O  RRRR
//  C      O   O  N  NN      S    T    R   R  U   U  C        T    O   O  R   R
//   CCCC   OOO   N   N  SSSS     T    R   R   UUU    CCCC    T     OOO   R   R
// ----------------------------------------------------------------------------
VL53L1X::VL53L1X(I2C_Driver * i2c_driver)
{
	this->m_i2c_address = VL53L1X_I2C_ADDRESS_DEFAULT;
	this->m_i2c_driver = i2c_driver;

	this->m_is_connected_to_TCA9548A_mux = false;
	this->m_mux_channel = 0;
	this->m_mux_i2c_address = TCA9548A_I2C_ADDRESS_DEFAULT;
}

VL53L1X::VL53L1X(I2C_Driver * i2c_driver, uint8_t address)
{
	// Check that the address is in the range [0,127]
	if (address > 127)
	{
		// Inform the user
		perror("Address supplied is greater than 127. Instead setting the address to the default of 0x29 (decimal 41).");
		// Default the address
		address = VL53L1X_I2C_ADDRESS_DEFAULT;
	}

	this->m_i2c_address = address;
	this->m_i2c_driver = i2c_driver;

	this->m_is_connected_to_TCA9548A_mux = false;
	this->m_mux_channel = 0;
	this->m_mux_i2c_address = TCA9548A_I2C_ADDRESS_DEFAULT;
}

VL53L1X::VL53L1X(I2C_Driver * i2c_driver, uint8_t address, uint8_t mux_channel, uint8_t mux_i2c_address)
{
	// Check that the address is in the range [0,127]
	if (address > 127)
	{
		// Inform the user
		perror("I2C address supplied is greater than 127. Instead setting the I2C address to the default of 0x29 (decimal 41).");
		// Default the address
		address = VL53L1X_I2C_ADDRESS_DEFAULT;
	}

	this->m_i2c_address = address;
	this->m_i2c_driver = i2c_driver;

	// Check that the mux channel is in the range [0,7]
	// and the mux I2C address is in the range [0,127]
	if (mux_channel<=7 && mux_i2c_address<=127)
	{
		this->m_is_connected_to_TCA9548A_mux = true;
		this->m_mux_channel = mux_channel;
		this->m_mux_i2c_address = mux_i2c_address;
	}
	else
	{
		// Inform the user
		if (mux_channel > 7)
			perror("Mux channel supplied in greater than 7. Instead setting that a mux is NOT being used.");
		if (mux_i2c_address > 127)
			perror("Mux I2C address supplied in greater than 127. Instead setting that a mux is NOT being used.");
		// Default the mux to not in use
		this->m_is_connected_to_TCA9548A_mux = false;
		this->m_mux_channel = 0;
		this->m_mux_i2c_address = TCA9548A_I2C_ADDRESS_DEFAULT;
	}
}





// ------------------------------------------------------
//   GGGG  EEEEE  TTTTT       &&&      SSSS  EEEEE  TTTTT
//  G      E        T        &        S      E        T
//  G  GG  EEE      T        && &      SSS   EEE      T
//  G   G  E        T       &  &          S  E        T
//   GGGG  EEEEE    T        && &     SSSS   EEEEE    T
// ------------------------------------------------------

uint8_t VL53L1X::get_i2c_address()
{
	return this->m_i2c_address;
}

bool VL53L1X::set_i2c_address(uint8_t new_address)
{
	if (new_address<0 || new_address>127)
	{
		return false;
	}
	this->m_i2c_address = new_address;
	return true;
}



uint8_t VL53L1X::get_mux_channel()
{
	return this->m_mux_channel;
}

bool VL53L1X::set_mux_channel(uint8_t new_channel)
{
	if (new_channel<0 || new_channel>7)
	{
		return false;
	}
	this->m_mux_channel = new_channel;
	return true;
}

uint8_t VL53L1X::get_mux_i2c_address()
{
	return this->m_mux_i2c_address;
}

bool VL53L1X::set_mux_i2c_address(uint8_t new_address)
{
	if (new_address<0 || new_address>127)
	{
		return false;
	}
	this->m_mux_i2c_address = new_address;
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
bool VL53L1X::switch_mux_channel_to_this_sensor()
{
	// Check if connected to a mux
	if (this->m_is_connected_to_TCA9548A_mux)
	{
		// Convert the mux channel to the regiter value
		uint8_t mux_register_value = (0x01 << (this->m_mux_channel));
		// Put the mux register value into a uint8 array
		uint8_t write_array[] = { mux_register_value };
		// Specify the number of bytes in the array
		int num_write_bytes = sizeof(write_array);

		// Call the i2c_driver function
		bool wasSuccessful = this->m_i2c_driver->write_data(this->m_mux_i2c_address, num_write_bytes, write_array);
		// Check the status
		if (wasSuccessful)
		{
			// Return flag that the operation was successful
			return true;
		}
		else
		{
			// Inform the user
			//perror("FAILED to set mux channel.");
			// Return flag that the operation was unsuccessful
			return false;
		}
	}
	else
	{
		return true;
	}
}

// PRIVATE FUNCTION:
bool VL53L1X::read_register(uint16_t dev, uint8_t register_address, uint16_t * value)
{
	// Set the mux channel
	bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
	if (!set_mux_wasSuccessful)
		return false;
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
bool VL53L1X::write_register(uint16_t dev, uint8_t register_address, uint16_t value)
{
	// Set the mux channel
	bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
	if (!set_mux_wasSuccessful)
		return false;
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
		//perror("FAILED to write register.");
		// Return flag that the operation was unsuccessful
		return false;
	}
}





// PRIVATE FUNCTION

bool VL53L1X::read_byte(uint16_t index, uint8_t *data)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L1 API function
		uint16_t dev = i2c_fd;
		VL53L1X_ERROR result_of_call = VL53L1_RdByte(dev, index, data);
		if (result_of_call==0)
			return true;
		else
			return false;
	}
	else
	{
		return false;
	}
}

bool VL53L1X::read_word(uint16_t index, uint16_t *data)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L1 API function
		uint16_t dev = i2c_fd;
		VL53L1X_ERROR result_of_call = VL53L1_RdWord(dev, index, data);
		if (result_of_call==0)
			return true;
		else
			return false;
	}
	else
	{
		return false;
	}
}

bool VL53L1X::read_dword(uint16_t index, uint32_t *data)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L1 API function
		uint16_t dev = i2c_fd;
		VL53L1X_ERROR result_of_call = VL53L1_RdDWord(dev, index, data);
		if (result_of_call==0)
			return true;
		else
			return false;
	}
	else
	{
		return false;
	}
}

bool VL53L1X::boot_state(uint8_t *state)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L1 API function
		uint16_t dev = i2c_fd;
		VL53L1X_ERROR result_of_call = VL53L1X_BootState(dev, state);
		if (result_of_call==0)
			return true;
		else
			return false;
	}
	else
	{
		return false;
	}
}

bool VL53L1X::sensor_init()
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L1 API function
		uint16_t dev = i2c_fd;
		VL53L1X_ERROR result_of_call = VL53L1X_SensorInit(dev);
		if (result_of_call==0)
			return true;
		else
			return false;
	}
	else
	{
		return false;
	}
}

bool VL53L1X::set_distance_mode(uint16_t dist_mode)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L1 API function
		uint16_t dev = i2c_fd;
		VL53L1X_ERROR result_of_call = VL53L1X_SetDistanceMode(dev,dist_mode);
		if (result_of_call==0)
			return true;
		else
			return false;
	}
	else
	{
		return false;
	}
}


bool VL53L1X::start_ranging()
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L1 API function
		uint16_t dev = i2c_fd;
		VL53L1X_ERROR result_of_call = VL53L1X_StartRanging(dev);
		if (result_of_call==0)
			return true;
		else
			return false;
	}
	else
	{
		return false;
	}
}

bool VL53L1X::check_for_data_ready(uint8_t *is_data_ready)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L1 API function
		uint16_t dev = i2c_fd;
		VL53L1X_ERROR result_of_call = VL53L1X_CheckForDataReady(dev, is_data_ready);
		if (result_of_call==0)
			return true;
		else
			return false;
	}
	else
	{
		return false;
	}
}


bool VL53L1X::get_result(VL53L1X_Result_t *pointer_to_results)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L1 API function
		uint16_t dev = i2c_fd;
		VL53L1X_ERROR result_of_call = VL53L1X_GetResult(dev, pointer_to_results);
		if (result_of_call==0)
			return true;
		else
			return false;
	}
	else
	{
		return false;
	}
}
// NOTE:
// The "VL53L1X_Result_t" struct is defined in "VL53L1X_api.h"
// as follows:
//		typedef struct {
//			uint8_t Status;		/*!< ResultStatus */
//			uint16_t Distance;	/*!< ResultDistance */
//			uint16_t Ambient;	/*!< ResultAmbient */
//			uint16_t SigPerSPAD;/*!< ResultSignalPerSPAD */
//			uint16_t NumSPADs;	/*!< ResultNumSPADs */
//		} VL53L1X_Result_t;


bool VL53L1X::clear_interrupt()
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L1 API function
		uint16_t dev = i2c_fd;
		VL53L1X_ERROR result_of_call = VL53L1X_ClearInterrupt(dev);
		if (result_of_call==0)
			return true;
		else
			return false;
	}
	else
	{
		return false;
	}
}



bool VL53L1X::initialise(uint16_t distance_mode)
{
	// Initialise a flag for whether the VL53L1X distance
	// sensor is successfully intialised
	bool is_initialised = false;

	// Call the "sensor_init" fuction
	bool success_sensor_init = this->sensor_init();

	// Return if not successful
	if (!success_sensor_init)
	{
		//printf("Function \"sensor_init()\" return ERROR for the VL53L1X distance sensor.\n");
		// Return NOT success
		return false;
	}

	// Read the model ID and module type
	uint16_t model_id;
	uint8_t module_type;
	bool read_success_model_id    = this->read_word(0x010F, &model_id);
	bool read_success_module_type = this->read_byte(0x010F, &module_type);

	// Set the ranging specifications
	// > Set the Distance Mode:
	//   1 = short distance
	//   2 = long distance
	bool success_set_distance_mode = this->set_distance_mode(distance_mode);

	// Return if not successful
	if (
		!read_success_model_id ||
		!read_success_module_type ||
		!success_set_distance_mode
	)
	{
		// Display which step failed
		if (!read_success_model_id)
			printf("Function \"VL53L1X::initialise()\" FAILED to read model ID of the VL53L1X distance sensor.\n");
		if (!read_success_module_type)
			printf("Function \"VL53L1X::initialise()\" FAILED to read module type of the VL53L1X distance sensor.\n");
		if (!success_set_distance_mode)
			printf("Function \"VL53L1X::initialise()\" FAILED to set distance mode for VL53L1X sensor.\n");
		// Return NOT success
		return false;
	}

	// Display the model ID and module type
	//printf("VL53L1X distance sensor has model ID: %X, and module type: %X\n", model_id, module_type);

	// Display the ranging specifications
	//printf("VL53L1X ranging specifications set to, distance mode: %d\n", distance_mode);

	// Start ranging
	bool success_start_ranging = this->start_ranging();
	if (success_start_ranging)
	{
		//printf("Function \"VL53L1X::initialise()\" started ranging for VL53L1X distance sensor.\n");
		// Return success
		return true;
	}
	else
	{
		printf("Function \"VL53L1X::initialise()\" FAILED to started ranging for the VL53L1X distance sensor.\n");
		// Return NOT success
		return false;
	}
}

bool VL53L1X::initialise_and_start_ranging(uint16_t distance_mode)
{
	// Call the initialise function
	bool success_initilise = this->initialise(distance_mode);

	if (!success_initilise)
	{
		printf("Function \"VL53L1X::initialise_and_start_ranging()\" FAILED returned by call to \"VL53L1X::initialise\".\n");
		// Return NOT success
		return false;
	}

	// Start ranging
	bool success_start_ranging = this->start_ranging();
	if (success_start_ranging)
	{
		//printf("Function \"VL53L1X::initialise_and_start_ranging()\" started ranging for VL53L1X distance sensor.\n");
		// Set the flag that the sensor is available
		// Return success
		return true;
	}
	else
	{
		printf("Function \"VL53L1X::initialise()\" FAILED to started ranging for the VL53L1X distance sensor.\n");
		// Return NOT success
		return false;
	}
}

bool VL53L1X::get_distance_measurement(VL53L1X_Result_t *pointer_to_results)
{
	// > Wait until data is ready
	uint8_t is_data_ready = 0;
	uint16_t data_wait_counter = 0;
	while ( (is_data_ready == 0) && (data_wait_counter<10) ) {
		bool bool_status = this->check_for_data_ready(&is_data_ready);
		data_wait_counter++;
		usleep(1);
	}
	// > Get the data
	bool get_success_result = this->get_result(pointer_to_results);

	// Trigger the next ranging
	bool success_clear_interrupt = this->clear_interrupt();

	if (!success_clear_interrupt)
		printf("Function \"VL53L1X::get_distance_measurement\" FAILED to clear interrupt for VL53L1X distance sensor.\n");

	// Return the result of the call to get_result(...)
	return get_success_result;
}

