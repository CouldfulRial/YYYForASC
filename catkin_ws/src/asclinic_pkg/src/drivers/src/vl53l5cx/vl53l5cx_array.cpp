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





#include "vl53l5cx/vl53l5cx_array.h"





// ----------------------------------------------------------------------------
//   CCCC   OOO   N   N   SSSS  TTTTT  RRRR   U   U   CCCC  TTTTT   OOO   RRRR
//  C      O   O  NN  N  S        T    R   R  U   U  C        T    O   O  R   R
//  C      O   O  N N N   SSS     T    RRRR   U   U  C        T    O   O  RRRR
//  C      O   O  N  NN      S    T    R   R  U   U  C        T    O   O  R   R
//   CCCC   OOO   N   N  SSSS     T    R   R   UUU    CCCC    T     OOO   R   R
// ----------------------------------------------------------------------------
VL53L5CX_Array::VL53L5CX_Array()
{
	// Initialise the "VL53L5CX_Array::State" as uninitialised
	this->m_state = VL53L5CX_Array::State::uninitialised;
}

VL53L5CX_Array::VL53L5CX_Array(I2C_Driver * i2c_driver, uint8_t mux_i2c_address)
{
	// Set the driver and address
	this->set_i2c_driver_and_mux_i2c_address(i2c_driver, mux_i2c_address);

	// Initialise the "VL53L5CX_Array::State" as uninitialised
	this->m_state = VL53L5CX_Array::State::uninitialised;
}





// ------------------------------------------------------
//   GGGG  EEEEE  TTTTT       &&&      SSSS  EEEEE  TTTTT
//  G      E        T        &        S      E        T
//  G  GG  EEE      T        && &      SSS   EEE      T
//  G   G  E        T       &  &          S  E        T
//   GGGG  EEEEE    T        && &     SSSS   EEEEE    T
// ------------------------------------------------------

uint8_t VL53L5CX_Array::get_mux_i2c_address()
{
	return this->m_mux_i2c_address;
}

bool VL53L5CX_Array::set_mux_i2c_address(uint8_t new_address)
{
	if (new_address<0 || new_address>127)
	{
		return false;
	}
	this->m_mux_i2c_address = new_address;
	return true;
}

void VL53L5CX_Array::set_i2c_driver_and_mux_i2c_address(I2C_Driver * i2c_driver, uint8_t mux_i2c_address)
{
	// Check that the address is in the range [0,127]
	if (mux_i2c_address > 127)
	{
		// Inform the user
		perror("Address supplied is greater than 127. Instead setting the mux address to the default of 0x70 (decimal 112).");
		// Default the address
		mux_i2c_address = TCA9548A_I2C_ADDRESS_DEFAULT;
	}

	this->m_i2c_driver = i2c_driver;

	this->m_mux_i2c_address = mux_i2c_address;

	// Initialise the "VL53L5CX_Array::State" as uninitialised
	this->m_state = VL53L5CX_Array::State::uninitialised;
}





// ------------------------------------------------------------
//  FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N   SSSS
//  F      U   U  NN  N  C        T     I   O   O  NN  N  S
//  FFF    U   U  N N N  C        T     I   O   O  N N N   SSS
//  F      U   U  N  NN  C        T     I   O   O  N  NN      S
//  F       UUU   N   N   CCCC    T    III   OOO   N   N  SSSS
// ------------------------------------------------------------

// PUBLIC FUNCTION:
bool VL53L5CX_Array::add_sensor(uint8_t mux_channel){
	if (this->m_state == VL53L5CX_Array::State::uninitialised)
	{
		// Instantiate an object
		VL53L5CX vl53l5cx_object (this->m_i2c_driver, VL53L5CX_I2C_ADDRESS_DEFAULT, mux_channel, this->m_mux_i2c_address);

		// Check if a VL53L5CX sensor is connected at this channel
		// > i.e., check is the sensor "is alive"
		uint8_t is_alive_result;
		bool success_is_alive_call = vl53l5cx_object.sensor_is_alive(&is_alive_result);
		if (!success_is_alive_call)
		{
			printf("[VL53L5CX ARRAY DRIVER] I2C communication failed while running \"VL53L5CX::sensor_is_alive\" for VL53L5CX address %d, and mux addres %d\n", static_cast<int>(vl53l5cx_object.get_i2c_address()), static_cast<int>(vl53l5cx_object.get_mux_i2c_address()) );
			return false;
		}

		// Add the sensor to the array (if it is "alive")
		if(is_alive_result)
		{
			// Add the sensor object to the array
			this->m_sensor_array.push_back(vl53l5cx_object);
			// Increment the counter
			this->m_num_sensors++;
			return true;
		}
		else
		{
			printf("[VL53L5CX ARRAY DRIVER] the \"VL53L5CX::sensor_is_alive\" call did not detect a VL53L5CX sensor at mux channel %d\n", static_cast<int>(mux_channel));
			return false;
		}
	}
	else
	{
		printf("[VL53L5CX ARRAY DRIVER] the \"VL53L5CX_Array::add_sensor\" only adds sensors when the array state is unitialised\n");
		return false;
	}
}

bool VL53L5CX_Array::initialise_then_start_ranging_in_a_separate_thread()
{
	// Check that there are 1 or more sensors in the array
	m_results_mutex.lock();
	uint8_t num_sensors = this->m_num_sensors;
	m_results_mutex.unlock();
	if (num_sensors == 0)
	{
		printf("[VL53L5CX ARRAY DRIVER] The array currently has 0 sensors, hence NOT starting initialisation NOR ranging.\n");
		return false;
	}

	bool return_result;
	this->m_state_mutex.lock();
	switch (this->m_state)
	{
	case VL53L5CX_Array::State::uninitialised:
		{
			this->m_init_and_ranging_thread = std::thread { &VL53L5CX_Array::init_all_sensors_then_range, this };
			return_result = true;
			break;
		}
	case VL53L5CX_Array::State::initialising:
		{
			printf("[VL53L5CX ARRAY DRIVER] The array is currently initialising, hence NOT starting anything additional.\n");
			return_result = false;
			break;
		}
	case VL53L5CX_Array::State::idle:
		{
			printf("[VL53L5CX ARRAY DRIVER] The array is already initialise, hence starting ranging.\n");
			this->m_init_and_ranging_thread = std::thread { &VL53L5CX_Array::init_all_sensors_then_range, this };
			return_result = true;
			break;
		}
	case VL53L5CX_Array::State::ranging:
		{
			printf("[VL53L5CX ARRAY DRIVER] The array is current ranging, hence NOT starting anything additional.\n");
			return_result = false;
			break;
		}
	default:
		{
			printf("[VL53L5CX ARRAY DRIVER] Invalid value for \"this->m_state\"\n");
			return_result = false;
			break;
		}
	}
	this->m_state_mutex.unlock();
	
	return return_result;
}

bool VL53L5CX_Array::start_ranging()
{
	// Check that there are 1 or more sensors in the array
	m_results_mutex.lock();
	uint8_t num_sensors = this->m_num_sensors;
	m_results_mutex.unlock();
	if (num_sensors == 0)
	{
		printf("[VL53L5CX ARRAY DRIVER] The array currently has 0 sensors, hence NOT starting ranging.\n");
		return false;
	}

	bool return_result;
	m_state_mutex.lock();
	switch (this->m_state)
	{
	case VL53L5CX_Array::State::uninitialised:
		{
			printf("[VL53L5CX ARRAY DRIVER] The array is currently uninitialised, hence canNOT start ranging.\n");
			return_result = false;
			break;
		}
	case VL53L5CX_Array::State::initialising:
		{
			printf("[VL53L5CX ARRAY DRIVER] The array is currently initialising, hence canNOT start ranging.\n");
			return_result = false;
			break;
		}
	case VL53L5CX_Array::State::idle:
		{
			this->m_init_and_ranging_thread = std::thread { &VL53L5CX_Array::init_all_sensors_then_range, this };
			return_result = true;
			break;
		}
	case VL53L5CX_Array::State::ranging:
		{
			printf("[VL53L5CX ARRAY DRIVER] The array is current ranging, hence NOT starting anything additional.\n");
			return_result = false;
			break;
		}
	default:
		{
			printf("[VL53L5CX ARRAY DRIVER] Invalid value for \"this->m_state\"\n");
			return_result = false;
			break;
		}
	}
	m_state_mutex.unlock();
	
	return return_result;
}

bool VL53L5CX_Array::stop_ranging()
{
	// Set the variable to stop the ranging
	bool should_join_ranging_thread = false;
	m_state_mutex.lock();
	if (this->m_state == VL53L5CX_Array::State::ranging)
	{
		m_should_stop_ranging = true;
		should_join_ranging_thread = true;
	}
	else
	{
		printf("[VL53L5CX ARRAY DRIVER] Not currently in the ranging state, hence this call to \"VL53L5CX_Array::stop_ranging()\" does nothing.\n");
		should_join_ranging_thread = false;
	}
	m_state_mutex.unlock();

	// Join the thread
	if (should_join_ranging_thread)
	{
		// Wait for the thread to join
		this->m_init_and_ranging_thread.join();
		// Update the state to idle
		m_state_mutex.lock();
		this->m_state = VL53L5CX_Array::State::idle;
		m_state_mutex.unlock();
	}

	return true;
}

bool VL53L5CX_Array::get_distance_measurements(VL53L5CX_ResultsData * results_data_array)
{
	bool should_copy_across_measurements = true;
	m_state_mutex.lock();
	if (this->m_state != VL53L5CX_Array::State::ranging)
	{
		//printf("[VL53L5CX ARRAY DRIVER] The array is not initialised, hence no measurements available\n");
		should_copy_across_measurements = false;
	}
	m_state_mutex.unlock();

	if (should_copy_across_measurements)
	{
		// Copy across the results
		m_results_mutex.lock();
		uint8_t num_sensors = this->m_num_sensors;
		for (int i = 0; i < num_sensors; i++)
		{
			results_data_array[i] = this->m_results_data_array[i];
		}
		m_results_mutex.unlock();

		return true;
	}
	else
	{
		return false;
	}

}

// PRIVATE FUNCTION
void VL53L5CX_Array::init_all_sensors_then_range()
{
	bool should_init_array = false;
	m_state_mutex.lock();
	if (this->m_state == VL53L5CX_Array::State::uninitialised)
	{
		this->m_state = VL53L5CX_Array::State::initialising;
		should_init_array = true;
	}
	m_state_mutex.unlock();

	m_results_mutex.lock();
	uint8_t num_sensors = this->m_num_sensors;
	m_results_mutex.unlock();


	printf("[VL53L5CX ARRAY DRIVER] Starting initialisation of the array of %d sensors\n", static_cast<int>(num_sensors));

	// ARRAY INTIALISATION
	if (should_init_array)
	{
		bool cumulative_init_success = true;
		for (std::size_t i = 0; i < num_sensors; i++)
		{
			printf("[VL53L5CX ARRAY DRIVER] Starting initialisation of sensor at index %d\n", static_cast<int>(i));

			uint8_t resolution           = VL53L5CX_ARRAY_RESOLUTION_DEFAULT;
			uint8_t ranging_frequency_hz = VL53L5CX_ARRAY_RANGING_FREQUENCT_DEFAULT;
			uint8_t integration_time_ms  = VL53L5CX_ARRAY_INTEGRATION_TIME_MS_DEFAULT;
			uint8_t sharpener_percent    = VL53L5CX_ARRAY_SHARPENER_PERCENT_DEFAULT;
			uint8_t target_order         = VL53L5CX_ARRAY_TARGET_ORDER_DEFAULT;
			uint8_t ranging_mode         = VL53L5CX_ARRAY_RANGING_MODE_DEFAULT;
			bool this_init_success = this->m_sensor_array[i].initialise(resolution, ranging_frequency_hz, integration_time_ms, sharpener_percent, target_order, ranging_mode);
			cumulative_init_success &= this_init_success;

			if (!this_init_success)
			{
				printf("[VL53L5CX ARRAY DRIVER] Sensor initialisation failed for sensor at mux channel %d\n", static_cast<int>(this->m_sensor_array[i].get_mux_channel()));
			}
			else
			{
				printf("[VL53L5CX ARRAY DRIVER] Finished initialisation of sensor at index %d\n", static_cast<int>(i));
			}
		}

		if (cumulative_init_success)
		{
			m_state_mutex.lock();
			this->m_state = VL53L5CX_Array::State::ranging;
			m_state_mutex.unlock();
		}
		else
		{
			printf("[VL53L5CX ARRAY DRIVER] An error occurred during the initialisation step within \"VL53L5CX_Array::init_all_sensors_then_range\"\n");
		}
		
	}

	printf("[VL53L5CX ARRAY DRIVER] Finished initialising the array of %d sensors. Now starting ranging.\n", static_cast<int>(num_sensors));

	m_state_mutex.lock();
	this->m_state = VL53L5CX_Array::State::ranging;
	m_state_mutex.unlock();

	// Call the start ranging function for each sensor
	bool cumulative_success_start_ranging = true;
	for (std::size_t i = 0; i < num_sensors; i++)
	{
		bool success_start_ranging = this->m_sensor_array[i].start_ranging();
		if (!success_start_ranging)
		{
			printf("[VL53L5CX DRIVER] Function \"VL53L5CX::start_ranging()\" FAILED.\n");
		}
		cumulative_success_start_ranging &= success_start_ranging;
	}

	// CONTINUALLY RANGE
	bool should_stop_ranging = false;
	while (!should_stop_ranging)
	{
		// Initialise a local placeholder for the results
		VL53L5CX_ResultsData tof_result;

		// Iterate through the sensor array
		for (std::size_t i = 0; i < num_sensors; i++)
		{
			uint8_t is_data_ready = 0;
			bool success_check_data_ready = this->m_sensor_array[i].check_data_ready(&is_data_ready);
			if (success_check_data_ready)
			{
				if (is_data_ready==1)
				{
					// > Get the data
					m_results_mutex.lock();
					bool success_get_ranging_data = this->m_sensor_array[i].get_ranging_data(&(this->m_results_data_array[i]));
					m_results_mutex.unlock();

					if (!success_get_ranging_data)
					{
						printf("[VL53L5CX DRIVER] Function \"VL53L5CX::get_ranging_data\" FAILED for sensor at mux channel %d\n", static_cast<int>(this->m_sensor_array[i].get_mux_channel()));	
					}
				}
			}
			// Check whether to stop ranging
			m_state_mutex.lock();
			should_stop_ranging = this->m_should_stop_ranging;
			m_state_mutex.unlock();

			// Sleep briefly
			//usleep(interval_between_checks_usec);
		}
	}
}