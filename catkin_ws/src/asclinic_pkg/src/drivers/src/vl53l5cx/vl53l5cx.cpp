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
// I2C driver for the VL53L5CX time-of-flight distance sensor
//
// ----------------------------------------------------------------------------

// NOTES - ON HOW THE API FROM ST IS USED:
// > The API provided by ST uses the variable "uint16_t dev" throughout the
//   code.
// > This "dev" variable is not actually used by any of the functions, it is
//   only passed along to every function that is called.
// > For example, a call to the following function, implemented in
//   VL53L5CX_api.{h,c}:
//     "VL53L5CX_ERROR VL53L5CX_GetDistance(uint16_t dev, uint16_t *distance);"
//   will subsequently call the following function, implmented in
//   vl53l5cx_platform.{h,c}:
//     "int8_t VL53L5CX_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata)"
//   which will subsequently call the following function, also implemented in
//   vl53l5cx_platform.{h,c}:
//     "int8_t VL53L5CX_WriteMulti( uint16_t dev, uint16_t register_address, uint8_t *write_data_array, uint16_t num_write_btyes);"
//   and this last function makes the calls to "ioctl(...)" to implement
//   writing over the I2C bus to the VL53L5CX sensor.
// > The equivalent final function for reading data over the I2C bus from
//   the VL53L5CX sensor is the following, also implemented in
//   vl53l5cx_platform.{h,c}:
//     "int8_t VL53L5CX_ReadMulti(uint16_t dev, uint16_t register_address, uint8_t *read_data_array, uint16_t num_read_btyes);"
// > The "dev" variable is be used for passing the file descriptor of the
//   appropriate I2C device through to the functions within
//   vl53l5cx_platform.{h,c} that implement the calls to "ioctl(...)".
//
// NOTES - FOR MULTIPLE VL53L5CX SENSORS ON THE SAME I2C BUS:
// > In order to manage multiple VL53L5CX I2C sensors on the same I2C bus,
//   an I2C multiplexer is required.
// > The "dev" variable is used ...
//
// NOTES - ON THE TYPE "VL53L5CX_ERROR"
// > The type "VL53L5CX_ERROR" is defined in VL53L5CX_api.h:
//     "typedef int8_t VL53L5CX_ERROR;"
// > This is the return type used in most of the function 
//   implemented in vl53l5cx_platform.{h,c} and VL53L5CX_api.{h,c}
// > The convention used is:
//   >  0 is success
//   > -1 is error
// > The convention used for functions in this class:
//   > true  is success
//   > false is error
//
// NOTES - ON READ/WRITE VIA THE "m_i2c_driver" MEMBER VARIABLE
// > The following two function read to and write from the VL53L5CX sensor
//   over the I2C bus via the "m_i2c_driver" member variable of this
//   class:
//     "bool VL53L5CX::read_register(...)"
//     "bool VL53L5CX::write_register(...)"
// > Use of these functions is discouraged because ST does not provide
//   documentation of the registers of the VL53L5CX sensor.
// > These functions are included for completeness should a unique
//   circumstance arise that justifies bypassing of the API functions
//   provided by ST.


#include "vl53l5cx/vl53l5cx.h"





// ----------------------------------------------------------------------------
//   CCCC   OOO   N   N   SSSS  TTTTT  RRRR   U   U   CCCC  TTTTT   OOO   RRRR
//  C      O   O  NN  N  S        T    R   R  U   U  C        T    O   O  R   R
//  C      O   O  N N N   SSS     T    RRRR   U   U  C        T    O   O  RRRR
//  C      O   O  N  NN      S    T    R   R  U   U  C        T    O   O  R   R
//   CCCC   OOO   N   N  SSSS     T    R   R   UUU    CCCC    T     OOO   R   R
// ----------------------------------------------------------------------------
VL53L5CX::VL53L5CX(I2C_Driver * i2c_driver)
{
	this->m_i2c_address = VL53L5CX_I2C_ADDRESS_DEFAULT;
	this->m_i2c_driver = i2c_driver;

	this->m_is_connected_to_TCA9548A_mux = false;
	this->m_mux_channel = 0;
	this->m_mux_i2c_address = TCA9548A_I2C_ADDRESS_DEFAULT;

	this->m_vl53l5cx_configuration.platform.address = this->m_i2c_address;
	this->m_vl53l5cx_configuration.platform.i2c_fd  = static_cast<int16_t>(i2c_driver->get_file_descriptor());

	// Initialise the "VL53L5CX::State" as uninitialised
	this->m_state = VL53L5CX::State::uninitialised;

}

VL53L5CX::VL53L5CX(I2C_Driver * i2c_driver, uint8_t address)
{
	// Check that the address is in the range [0,127]
	if (address > 127)
	{
		// Inform the user
		perror("Address supplied is greater than 127. Instead setting the address to the default of 0x29 (decimal 41).");
		// Default the address
		address = VL53L5CX_I2C_ADDRESS_DEFAULT;
	}

	this->m_i2c_address = address;
	this->m_i2c_driver = i2c_driver;

	this->m_is_connected_to_TCA9548A_mux = false;
	this->m_mux_channel = 0;
	this->m_mux_i2c_address = TCA9548A_I2C_ADDRESS_DEFAULT;

	this->m_vl53l5cx_configuration.platform.address = this->m_i2c_address;
	this->m_vl53l5cx_configuration.platform.i2c_fd  = static_cast<int16_t>(i2c_driver->get_file_descriptor());

	// Initialise the "VL53L5CX::State" as uninitialised
	this->m_state = VL53L5CX::State::uninitialised;
}

VL53L5CX::VL53L5CX(I2C_Driver * i2c_driver, uint8_t address, uint8_t mux_channel, uint8_t mux_i2c_address)
{
	// Check that the address is in the range [0,127]
	if (address > 127)
	{
		// Inform the user
		perror("I2C address supplied is greater than 127. Instead setting the I2C address to the default of 0x29 (decimal 41).");
		// Default the address
		address = VL53L5CX_I2C_ADDRESS_DEFAULT;
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

	this->m_vl53l5cx_configuration.platform.address = this->m_i2c_address;
	this->m_vl53l5cx_configuration.platform.i2c_fd  = static_cast<int16_t>(i2c_driver->get_file_descriptor());

	// Initialise the "VL53L5CX::State" as uninitialised
	this->m_state = VL53L5CX::State::uninitialised;
}





// ------------------------------------------------------
//   GGGG  EEEEE  TTTTT       &&&      SSSS  EEEEE  TTTTT
//  G      E        T        &        S      E        T
//  G  GG  EEE      T        && &      SSS   EEE      T
//  G   G  E        T       &  &          S  E        T
//   GGGG  EEEEE    T        && &     SSSS   EEEEE    T
// ------------------------------------------------------

uint8_t VL53L5CX::get_i2c_address()
{
	return this->m_i2c_address;
}

bool VL53L5CX::set_i2c_address(uint8_t new_address)
{
	if (new_address<0 || new_address>127)
	{
		return false;
	}
	this->m_i2c_address = new_address;
	return true;
}



uint8_t VL53L5CX::get_mux_channel()
{
	return this->m_mux_channel;
}

bool VL53L5CX::set_mux_channel(uint8_t new_channel)
{
	if (new_channel<0 || new_channel>7)
	{
		return false;
	}
	this->m_mux_channel = new_channel;
	return true;
}

uint8_t VL53L5CX::get_mux_i2c_address()
{
	return this->m_mux_i2c_address;
}

bool VL53L5CX::set_mux_i2c_address(uint8_t new_address)
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
bool VL53L5CX::switch_mux_channel_to_this_sensor()
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


bool VL53L5CX::read_register(uint16_t dev, uint16_t register_address, uint16_t * value)
{
	// Set the mux channel
	bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
	if (!set_mux_wasSuccessful)
		return false;
	// Convert the "register_address" into its two
	// separate bytes
	uint8_t reg_addr_byte_msb = register_address >> 8;
	uint8_t reg_addr_byte_lsb = register_address & 0xFF;
	// Put the "register address" into a uint8 array
	uint8_t write_array[] = { reg_addr_byte_msb , reg_addr_byte_lsb };
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
bool VL53L5CX::write_register(uint16_t dev, uint16_t register_address, uint16_t value)
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

	// Convert the "register_address" into its two
	// separate bytes
	uint8_t reg_addr_byte_msb = register_address >> 8;
	uint8_t reg_addr_byte_lsb = register_address & 0xFF;

	// Put the "register address" and the "value"
	// into a uint8 array
	uint8_t write_array[] = { reg_addr_byte_msb , reg_addr_byte_lsb , value_byte_1 , value_byte_2 };
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





// PUBLIC FUNCTION

bool VL53L5CX::sensor_is_alive(uint8_t * is_alive)
{	
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_is_alive(&this->m_vl53l5cx_configuration, is_alive);
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

bool VL53L5CX::sensor_init()
{
	// Check whether this sensor class is
	// already in the initialised state
	if (this->m_state==VL53L5CX::State::idle || this->m_state==VL53L5CX::State::ranging)
	{
		printf("[VL53L5CX DRIVER] \"sensor_init\" called when sensor class is already in an initialised state.");
		return true;
	}

	// Set the state to indicate that the
	// initialisation is running
	// > Note that initialisation can take
	//   upto 10 seconds.
	this->m_state = VL53L5CX::State::initialising;

	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
		{
			// Set state back to uninitialised
			this->m_state = VL53L5CX::State::uninitialised;
			return false;
		}
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_init(&this->m_vl53l5cx_configuration);
		if (result_of_call==0)
		{
			// Set state to idle
			this->m_state = VL53L5CX::State::idle;
			return true;
		}
		else
		{
			// Set state back to uninitialised
			this->m_state = VL53L5CX::State::uninitialised;
			return false;
		}
	}
	else
	{
		// Set state back to uninitialised
		this->m_state = VL53L5CX::State::uninitialised;
		return false;
	}
}

bool VL53L5CX::get_power_mode(uint8_t * power_mode)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_get_power_mode(&this->m_vl53l5cx_configuration,power_mode);
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


bool VL53L5CX::set_power_mode(uint8_t power_mode)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_set_power_mode(&this->m_vl53l5cx_configuration,power_mode);
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

bool VL53L5CX::start_ranging()
{
	// Check whether this sensor class still
	// needs to be initialised
	if (this->m_state==VL53L5CX::State::uninitialised || this->m_state==VL53L5CX::State::initialising)
	{
		printf("[VL53L5CX DRIVER] \"start_ranging\" called when sensor class is not yet initialised.");
		return false;
	}

	// Check whether this sensor class is
	// already ranging
	if (this->m_state==VL53L5CX::State::ranging)
	{
		printf("[VL53L5CX DRIVER] \"start_ranging\" called when sensor class is already in the ranging state.");
		return true;
	}

	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_start_ranging(&this->m_vl53l5cx_configuration);
		if (result_of_call==0)
		{
			// Update the state to be ranging
			this->m_state = VL53L5CX::State::ranging;
			return true;
		}
		else
			return false;
	}
	else
	{
		return false;
	}
}

bool VL53L5CX::stop_ranging()
{
	// Check whether this sensor class is
	// not even in the ranging state
	if (this->m_state!=VL53L5CX::State::ranging)
	{
		printf("[VL53L5CX DRIVER] \"stop_ranging\" called when sensor class is not in the ranging state.");
		return true;
	}

	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_stop_ranging(&this->m_vl53l5cx_configuration);
		if (result_of_call==0)
		{
			this->m_state!=VL53L5CX::State::idle;
			return true;
		}
		else
			return false;
	}
	else
	{
		return false;
	}
}

bool VL53L5CX::check_data_ready(uint8_t * is_ready)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_check_data_ready(&this->m_vl53l5cx_configuration, is_ready);
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


bool VL53L5CX::get_ranging_data(VL53L5CX_ResultsData * results_data)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_get_ranging_data(&this->m_vl53l5cx_configuration, results_data);
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
// The "VL53L5CX_ResultData" struct is defined in "vl53l5cx_api.h"
// as follows:
//		typedef struct
//		{
//			/* Internal sensor silicon temperature */
//			int8_t silicon_temp_degc;//		
//
//			/* Ambient noise in kcps/spads */
//			uint32_t ambient_per_spad[VL53L5CX_RESOLUTION_8X8];
//
//			/* Number of valid target detected for 1 zone */
//			uint8_t nb_target_detected[VL53L5CX_RESOLUTION_8X8];
//
//			/* Number of spads enabled for this ranging */
//			uint32_t nb_spads_enabled[VL53L5CX_RESOLUTION_8X8];
//
//			/* Signal returned to the sensor in kcps/spads */
//			uint32_t signal_per_spad[(VL53L5CX_RESOLUTION_8X8 *VL53L5CX_NB_TARGET_PER_ZONE)];
//
//			/* Sigma of the current distance in mm */
//			uint16_t range_sigma_mm[(VL53L5CX_RESOLUTION_8X8 *VL53L5CX_NB_TARGET_PER_ZONE)];
//
//			/* Measured distance in mm */
//			int16_t distance_mm[(VL53L5CX_RESOLUTION_8X8 *VL53L5CX_NB_TARGET_PER_ZONE)];
//
//			/* Estimated reflectance in percent */
//			uint8_t reflectance[(VL53L5CX_RESOLUTION_8X8 *VL53L5CX_NB_TARGET_PER_ZONE)];
//
//			/* Status indicating the measurement validity (5 & 9 means ranging OK)*/
//			uint8_t target_status[(VL53L5CX_RESOLUTION_8X8 *VL53L5CX_NB_TARGET_PER_ZONE)];
//
//			/* Motion detector results */
//			struct
//			{
//				uint32_t global_indicator_1;
//				uint32_t global_indicator_2;
//				uint8_t	 status;
//				uint8_t	 nb_of_detected_aggregates;
//				uint8_t	 nb_of_aggregates;
//				uint8_t	 spare;
//				uint32_t motion[32];
//			} motion_indicator;
//
//		} VL53L5CX_ResultsData;



bool VL53L5CX::get_resolution(uint8_t * resolution)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_get_resolution(&this->m_vl53l5cx_configuration, resolution);
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

bool VL53L5CX::set_resolution(uint8_t resolution)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_set_resolution(&this->m_vl53l5cx_configuration, resolution);
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

bool VL53L5CX::get_ranging_frequency_hz(uint8_t * frequency_hz)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_get_ranging_frequency_hz(&this->m_vl53l5cx_configuration, frequency_hz);
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

bool VL53L5CX::set_ranging_frequency_hz(uint8_t frequency_hz)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_set_ranging_frequency_hz(&this->m_vl53l5cx_configuration, frequency_hz);
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

bool VL53L5CX::get_integration_time_ms(uint32_t * integration_time_ms)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_get_integration_time_ms(&this->m_vl53l5cx_configuration, integration_time_ms);
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

bool VL53L5CX::set_integration_time_ms(uint32_t integration_time_ms)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_set_integration_time_ms(&this->m_vl53l5cx_configuration, integration_time_ms);
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

bool VL53L5CX::get_sharpener_percent(uint8_t * sharpener_percent)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_get_sharpener_percent(&this->m_vl53l5cx_configuration, sharpener_percent);
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

bool VL53L5CX::set_sharpener_percent(uint8_t sharpener_percent)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_set_sharpener_percent(&this->m_vl53l5cx_configuration, sharpener_percent);
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

bool VL53L5CX::get_target_order(uint8_t * target_order)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_get_target_order(&this->m_vl53l5cx_configuration, target_order);
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

bool VL53L5CX::set_target_order(uint8_t target_order)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_set_target_order(&this->m_vl53l5cx_configuration, target_order);
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

bool VL53L5CX::get_ranging_mode(uint8_t * ranging_mode)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_get_ranging_mode(&this->m_vl53l5cx_configuration, ranging_mode);
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

bool VL53L5CX::set_ranging_mode(uint8_t ranging_mode)
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_set_ranging_mode(&this->m_vl53l5cx_configuration, ranging_mode);
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

bool VL53L5CX::enable_internal_cp()
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_enable_internal_cp(&this->m_vl53l5cx_configuration);
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

bool VL53L5CX::disable_internal_cp()
{
	int i2c_fd = this->m_i2c_driver->get_file_descriptor();
	if (i2c_fd>=0)
	{
		// Set the mux channel
		bool set_mux_wasSuccessful = this->switch_mux_channel_to_this_sensor();
		if (!set_mux_wasSuccessful)
			return false;
		// Call the VL53L5CX API function
		uint8_t result_of_call = vl53l5cx_disable_internal_cp(&this->m_vl53l5cx_configuration);
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


// PUBLIC FUNCTIONS
// CONVENIENCE FUNCTIONS

/**
 * @brief This function sets the operating parameters.
 * @param (uint8_t) resolution : Use macro VL53L5CX_RESOLUTION_4X4 or VL53L5CX_RESOLUTION_8X8 to set the resolution.
 * @param (uint8_t) ranging_frequency_hz : Contains the ranging frequency in Hz.
 * - For 4x4, min and max allowed values are : [1;60]
 * - For 8x8, min and max allowed values are : [1;15]
 * @param (uint32_t) time_ms : Contains the integration time in ms.
 * - For all resolutions and frequency, the minimum value is 2ms, and the maximum is 1000ms.
 * @param (uint32_t) sharpener_percent : Value between 0 (disabled) and 99%.
 * - Default is 5%.
 * @param (uint8_t) target_order : Use macros VL53L5CX_TARGET_ORDER_STRONGEST and VL53L5CX_TARGET_ORDER_CLOSEST to define the new output order.
 * - By default, the sensor is configured with the strongest output.
 * @param (uint8_t) ranging_mode : Use macros VL53L5CX_RANGING_MODE_CONTINUOUS and VL53L5CX_RANGING_MODE_AUTONOMOUS.
 * - The default mode is Autonomous
 * @return (bool) status : true if all parameters set correctly
 */

bool VL53L5CX::initialise(uint8_t resolution, uint8_t ranging_frequency_hz, uint8_t integration_time_ms, uint8_t sharpener_percent, uint8_t target_order, uint8_t ranging_mode)
{
	// Initialise a flag for whether the VL53L5CX distance
	// sensor is successfully intialised
	bool is_initialised = false;

	// Check that there is a VL53L5CX sensor connected
	// at the I2C address of this object
	uint8_t is_alive_result;
	bool success_is_alive_call = this->sensor_is_alive(&is_alive_result);
	if (!success_is_alive_call)
	{
		printf("[VL53L5CX DRIVER] I2C communication failed while check if the sensor \"VL53L5CX::sensor_is_alive\"\n");
	}
	if(!is_alive_result)
	{
		printf("[VL53L5CX DRIVER] the \"VL53L5CX::sensor_is_alive\" call did not detect a VL53L5CX sensor at address %d\n", static_cast<int>(this->m_vl53l5cx_configuration.platform.address));
		return false;
	}

	// Call the "sensor_init" fuction
	bool success_sensor_init = this->sensor_init();

	// Return if not successful
	if (!success_sensor_init)
	{
		printf("[VL53L5CX DRIVER] Function \"VL53L5CX::sensor_init()\" returned ERROR.\n");
		// Return NOT success
		return false;
	}

	// Set the resolution:
	//   VL53L5CX_RESOLUTION_4X4 = 16
	//   VL53L5CX_RESOLUTION_8X8 = 64
	bool success_set_resolution = this->set_resolution(resolution);

	// Return if not successful
	if (!success_set_resolution)
	{
		printf("[VL53L5CX DRIVER] Function \"VL53L5CX::set_resolution()\" FAILED.\n");
		// Return NOT success
		return false;
	}

	// Set the ranging frequency:
	//   For 4x4, allowed values are: [1;60]
 	//   For 8x8, allowed values are: [1;15]
	bool success_set_ranging_frequency = this->set_ranging_frequency_hz(ranging_frequency_hz);

	// Return if not successful
	if (!success_set_ranging_frequency)
	{
		printf("[VL53L5CX DRIVER] Function \"VL53L5CX::set_ranging_frequency_hz()\" FAILED.\n");
		// Return NOT success
		return false;
	}

	// Set the integration time
	//   Allowed values are: [2;1000] ms
	//   Only used for the "Autonomous"ranging mode
	bool success_set_integration_time = this->set_integration_time_ms(integration_time_ms);

	// Return if not successful
	if (!success_set_integration_time)
	{
		printf("[VL53L5CX DRIVER] Function \"VL53L5CX::set_integration_time_ms()\" FAILED.\n");
		// Return NOT success
		return false;
	}

	// Set the sharpener percent
	//   Allowed values are: [0;99] %
	//   0% means disabled, 5% is the default
	bool success_set_sharpener_percent = this->set_sharpener_percent(sharpener_percent);

	// Return if not successful
	if (!success_set_sharpener_percent)
	{
		printf("[VL53L5CX DRIVER] Function \"VL53L5CX::set_sharpener_percent()\" FAILED.\n");
		// Return NOT success
		return false;
	}

	// Set the target order:
	//   VL53L5CX_TARGET_ORDER_STRONGEST = 2 <default>
	//   VL53L5CX_TARGET_ORDER_CLOSEST   = 1
	bool success_set_target_order = this->set_target_order(target_order);

	// Return if not successful
	if (!success_set_target_order)
	{
		printf("[VL53L5CX DRIVER] Function \"VL53L5CX::set_target_order()\" FAILED.\n");
		// Return NOT success
		return false;
	}

	// Set the ranging mode:
	//   VL53L5CX_RANGING_MODE_CONTINUOUS = 1
	//   VL53L5CX_RANGING_MODE_AUTONOMOUS = 3 <default>
	bool success_set_ranging_mode = this->set_ranging_mode(ranging_mode);

	// Return if not successful
	if (!success_set_ranging_mode)
	{
		printf("[VL53L5CX DRIVER] Function \"VL53L5CX::set_ranging_mode()\" FAILED.\n");
		// Return NOT success
		return false;
	}

	// If the code reaches this point,
	// then the initialisation was successful.
	is_initialised = true;
	return is_initialised;
}

bool VL53L5CX::initialise_and_start_ranging(uint8_t resolution, uint8_t ranging_frequency_hz, uint8_t integration_time_ms, uint8_t sharpener_percent, uint8_t target_order, uint8_t ranging_mode)
{
	// Call the initialise function
	bool success_initilise = this->initialise(resolution, ranging_frequency_hz, integration_time_ms, sharpener_percent, target_order, ranging_mode);

	if (!success_initilise)
	{
		printf("[VL53L5CX DRIVER] Function \"VL53L5CX::initialise()\" FAILED.\n");
		// Return NOT success
		return false;
	}

	// Start ranging
	bool success_start_ranging = this->start_ranging();
	if (success_start_ranging)
	{
		//printf("[VL53L5CX DRIVER] Function \"VL53L5CX::start_ranging()\" successful.\n");
		// Return success
		return true;
	}
	else
	{
		printf("[VL53L5CX DRIVER] Function \"VL53L5CX::start_ranging()\" FAILED.\n");
		// Return NOT success
		return false;
	}
}

bool VL53L5CX::get_distance_measurement(VL53L5CX_ResultsData * results_data)
{
	// > Get the data
	bool success_get_ranging_data = this->get_ranging_data(results_data);

	if (!success_get_ranging_data)
		printf("[VL53L5CX DRIVER] Function \"VL53L5CX::get_ranging_data\" FAILED.\n");

	// Return the result of the call to get_ranging_data(...)
	return success_get_ranging_data;
}

bool VL53L5CX::check_data_ready_multiple_then_get_distance_measurement(VL53L5CX_ResultsData * results_data, uint16_t interval_between_checks_usec, uint16_t number_of_checks)
{
	// > Wait until data is ready
	uint8_t is_data_ready = 0;
	uint16_t data_wait_counter = 0;
	while ( (is_data_ready == 0) && (data_wait_counter<number_of_checks) )
	{
		bool success_check_data_ready = this->check_data_ready(&is_data_ready);
		if (success_check_data_ready)
		{
			if (is_data_ready==1)
				break;
		}
		data_wait_counter++;
		usleep(interval_between_checks_usec);
	}

	// > Get the data
	bool success_get_ranging_data = this->get_ranging_data(results_data);

	if (!success_get_ranging_data)
		printf("[VL53L5CX DRIVER] Function \"VL53L5CX::get_ranging_data\" FAILED.\n");

	// Return the result of the call to get_ranging_data(...)
	return success_get_ranging_data;
}

