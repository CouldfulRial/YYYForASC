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
// Class for managing an array of VL53L5CX time-of-flight distance sensors
//
// ----------------------------------------------------------------------------





#ifndef VL53L5CX_ARRAY_CPP_INTERFACE_H
#define VL53L5CX_ARRAY_CPP_INTERFACE_H





// #include <fcntl.h>
// #include <linux/i2c.h>
// #include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
// #include <sys/ioctl.h>
#include <unistd.h>

#include <vector>

#include <mutex>
#include <thread>

// #include "vl53l5cx/platform/vl53l5cx_platform.h"
// #include "vl53l5cx/VL53L5CX_ULD_API/inc/vl53l5cx_api.h"
// #include "vl53l5cx/VL53L5CX_ULD_API/inc/vl53l5cx_plugin_xtalk.h
// #include "vl53l5cx/VL53L5CX_ULD_API/inc/vl53l5cx_plugin_detection_thresholds.h
// #include "vl53l5cx/VL53L5CX_ULD_API/inc/vl53l5cx_plugin_motion_indicator.h
// #include "vl53l5cx/VL53L5CX_ULD_API/inc/vl53l5cx_buffers.h

#include "i2c_driver/i2c_driver.h"

//#include "vl53l5cx/VL53L5CX_ULD_API/inc/vl53l5cx_api.h"
#include "vl53l5cx/vl53l5cx.h"





#define VL53L5CX_ARRAY_RESOLUTION_DEFAULT             ((uint8_t) 16U) /**< Default VL53L5CX Resolution for the array */
#define VL53L5CX_ARRAY_RANGING_FREQUENCT_DEFAULT      10              /**< Default VL53L5CX Ranging frequency for the array */
#define VL53L5CX_ARRAY_INTEGRATION_TIME_MS_DEFAULT    50              /**< Default VL53L5CX Integration time in milliseconds for the array */
#define VL53L5CX_ARRAY_SHARPENER_PERCENT_DEFAULT      20              /**< Default VL53L5CX Sharpener percent for the array */
#define VL53L5CX_ARRAY_TARGET_ORDER_DEFAULT           ((uint8_t) 1U) /**< Default VL53L5CX Target order for the array */
#define VL53L5CX_ARRAY_RANGING_MODE_DEFAULT           ((uint8_t) 1U) /**< Default VL53L5CX Ranging mode for the array */

// The following values are from the file:
// > "vl53l5cx/VL53L5CX_ULD_API/inc/vl53l5cx_api.h"
//VL53L5CX_RESOLUTION_4X4			((uint8_t) 16U)
//VL53L5CX_RESOLUTION_8X8			((uint8_t) 64U)

//VL53L5CX_TARGET_ORDER_CLOSEST		((uint8_t) 1U)
//VL53L5CX_TARGET_ORDER_STRONGEST	((uint8_t) 2U)

//VL53L5CX_RANGING_MODE_CONTINUOUS	((uint8_t) 1U)
//VL53L5CX_RANGING_MODE_AUTONOMOUS	((uint8_t) 3U)





// ----------------------------------------------------------
//   CCCC  L        A     SSSS   SSSS     DDDD   EEEEE  FFFFF
//  C      L       A A   S      S         D   D  E      F
//  C      L      A   A   SSS    SSS      D   D  EEE    FFF
//  C      L      AAAAA      S      S     D   D  E      F
//   CCCC  LLLLL  A   A  SSSS   SSSS      DDDD   EEEEE  F
// ----------------------------------------------------------

class VL53L5CX_Array
{
	// ----------------------------------
	//  EEEEE  N   N  U   U  M   M   SSSS
	//  E      NN  N  U   U  MM MM  S
	//  EEE    N N N  U   U  M M M   SSS
	//  E      N  NN  U   U  M   M      S
	//  EEEEE  N   N   UUU   M   M  SSSS
	// ----------------------------------

	enum class State : int
	{
		uninitialised = 0,
		initialising = 1,
		idle = 2,
		ranging = 3,
	};





	// ------------------------------------------------------------
	//  V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
	//  V   V   A A   R   R   I    A A   B   B  L      E      S
	//  V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
	//   V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
	//    V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
	// ------------------------------------------------------------

private:
	I2C_Driver * m_i2c_driver;

	uint8_t m_mux_i2c_address;
	
	VL53L5CX_Array::State m_state;

	std::vector<VL53L5CX> m_sensor_array;
	uint8_t m_num_sensors = 0;

	VL53L5CX_ResultsData m_results_data_array[8];

	std::mutex m_state_mutex;
	std::mutex m_results_mutex;

	std::thread m_init_and_ranging_thread;

	bool m_should_stop_ranging = false;




	// ----------------------------------------------------------------------------
	//   CCCC   OOO   N   N   SSSS  TTTTT  RRRR   U   U   CCCC  TTTTT   OOO   RRRR
	//  C      O   O  NN  N  S        T    R   R  U   U  C        T    O   O  R   R
	//  C      O   O  N N N   SSS     T    RRRR   U   U  C        T    O   O  RRRR
	//  C      O   O  N  NN      S    T    R   R  U   U  C        T    O   O  R   R
	//   CCCC   OOO   N   N  SSSS     T    R   R   UUU    CCCC    T     OOO   R   R
	// ----------------------------------------------------------------------------
public:
	VL53L5CX_Array();
	VL53L5CX_Array(I2C_Driver * i2c_driver, uint8_t mux_i2c_address);





	// ------------------------------------------------------
	//   GGGG  EEEEE  TTTTT       &&&      SSSS  EEEEE  TTTTT
	//  G      E        T        &        S      E        T
	//  G  GG  EEE      T        && &      SSS   EEE      T
	//  G   G  E        T       &  &          S  E        T
	//   GGGG  EEEEE    T        && &     SSSS   EEEEE    T
	// ------------------------------------------------------
public:
	uint8_t get_mux_i2c_address();
	bool set_mux_i2c_address(uint8_t new_address);

	void set_i2c_driver_and_mux_i2c_address(I2C_Driver * i2c_driver, uint8_t mux_i2c_address);





	// ------------------------------------------------------------
	//  FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N   SSSS
	//  F      U   U  NN  N  C        T     I   O   O  NN  N  S
	//  FFF    U   U  N N N  C        T     I   O   O  N N N   SSS
	//  F      U   U  N  NN  C        T     I   O   O  N  NN      S
	//  F       UUU   N   N   CCCC    T    III   OOO   N   N  SSSS
	// ------------------------------------------------------------

public:
	bool add_sensor(uint8_t mux_channel);

	bool initialise_then_start_ranging_in_a_separate_thread();

	bool start_ranging();

	bool stop_ranging();

	bool get_results_data_array();

	bool get_distance_measurements(VL53L5CX_ResultsData * results_data);

private:
	void init_all_sensors_then_range();

}; // END OF CLASS DEFINITION





#endif // VL53L5CX_ARRAY_CPP_INTERFACE_H
