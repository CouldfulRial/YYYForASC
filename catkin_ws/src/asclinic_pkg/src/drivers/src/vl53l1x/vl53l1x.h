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





#ifndef VL53L1X_CPP_INTERFACE_H
#define VL53L1X_CPP_INTERFACE_H





#include <fcntl.h>
// #include <linux/i2c.h>
// #include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
// #include <sys/ioctl.h>
#include <unistd.h>

#include "vl53l1x/platform/vl53l1_platform.h"
#include "vl53l1x/core/VL53L1X_api.h"
//#include "vl53l1x/core/VL53L1X_calibration.h"

#include "i2c_driver/i2c_driver.h"





// I2C ADDRESS
#define VL53L1X_I2C_ADDRESS_DEFAULT     0x29 /**< Default VL53L1X I2C Slave Address */
#define TCA9548A_I2C_ADDRESS_DEFAULT    0x70 /**< Default TCA9548A I2C Slave Address */





// ----------------------------------------------------------
//   CCCC  L        A     SSSS   SSSS     DDDD   EEEEE  FFFFF
//  C      L       A A   S      S         D   D  E      F
//  C      L      A   A   SSS    SSS      D   D  EEE    FFF
//  C      L      AAAAA      S      S     D   D  E      F
//   CCCC  LLLLL  A   A  SSSS   SSSS      DDDD   EEEEE  F
// ----------------------------------------------------------

class VL53L1X
{
	// ------------------------------------------------------------
	//  V   V    A    RRRR   III    A    BBBB   L      EEEEE   SSSS
	//  V   V   A A   R   R   I    A A   B   B  L      E      S
	//  V   V  A   A  RRRR    I   A   A  BBBB   L      EEE     SSS
	//   V V   AAAAA  R  R    I   AAAAA  B   B  L      E          S
	//    V    A   A  R   R  III  A   A  BBBB   LLLLL  EEEEE  SSSS
	// ------------------------------------------------------------

private:
	uint8_t m_i2c_address;
	I2C_Driver * m_i2c_driver;

	bool m_is_connected_to_TCA9548A_mux;
	uint8_t m_mux_channel;
	uint8_t m_mux_i2c_address;





	// ----------------------------------------------------------------------------
	//   CCCC   OOO   N   N   SSSS  TTTTT  RRRR   U   U   CCCC  TTTTT   OOO   RRRR
	//  C      O   O  NN  N  S        T    R   R  U   U  C        T    O   O  R   R
	//  C      O   O  N N N   SSS     T    RRRR   U   U  C        T    O   O  RRRR
	//  C      O   O  N  NN      S    T    R   R  U   U  C        T    O   O  R   R
	//   CCCC   OOO   N   N  SSSS     T    R   R   UUU    CCCC    T     OOO   R   R
	// ----------------------------------------------------------------------------
public:
	VL53L1X(I2C_Driver * i2c_driver);
	VL53L1X(I2C_Driver * i2c_driver, uint8_t address);
	VL53L1X(I2C_Driver * i2c_driver, uint8_t address, uint8_t mux_channel, uint8_t mux_i2c_address);





	// ------------------------------------------------------
	//   GGGG  EEEEE  TTTTT       &&&      SSSS  EEEEE  TTTTT
	//  G      E        T        &        S      E        T
	//  G  GG  EEE      T        && &      SSS   EEE      T
	//  G   G  E        T       &  &          S  E        T
	//   GGGG  EEEEE    T        && &     SSSS   EEEEE    T
	// ------------------------------------------------------
public:
	uint8_t get_i2c_address();
	bool set_i2c_address(uint8_t new_address);

	uint8_t get_mux_channel();
	bool set_mux_channel(uint8_t new_channel);

	uint8_t get_mux_i2c_address();
	bool set_mux_i2c_address(uint8_t new_address);

	// ------------------------------------------------------------
	//  FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N   SSSS
	//  F      U   U  NN  N  C        T     I   O   O  NN  N  S
	//  FFF    U   U  N N N  C        T     I   O   O  N N N   SSS
	//  F      U   U  N  NN  C        T     I   O   O  N  NN      S
	//  F       UUU   N   N   CCCC    T    III   OOO   N   N  SSSS
	// ------------------------------------------------------------

public:
	bool switch_mux_channel_to_this_sensor();

	bool read_register(uint16_t dev, uint8_t register_address, uint16_t * value);

	bool write_register(uint16_t dev, uint8_t register_address, uint16_t value);

public:

	bool read_byte(uint16_t index, uint8_t *data);
	bool read_word(uint16_t index, uint16_t *data);
	bool read_dword(uint16_t index, uint32_t *data);
	
	bool boot_state(uint8_t *state);
	
	bool sensor_init();
	
	// /* status += VL53L1X_SetInterruptPolarity(Dev, 0); */

	bool set_distance_mode(uint16_t dist_mode);

	// status += VL53L1X_SetTimingBudgetInMs(Dev, 100);
	// status += VL53L1X_SetInterMeasurementInMs(Dev, 100);

	bool start_ranging();

	bool check_for_data_ready(uint8_t *is_data_ready);

	// status = VL53L1X_UltraLite_WaitForInterrupt(ST_TOF_IOCTL_WFI);

	bool get_result(VL53L1X_Result_t *pointer_to_results);

	bool clear_interrupt();


// Convenience Functions
public:
	bool initialise(uint16_t distance_mode);

	bool initialise_and_start_ranging(uint16_t distance_mode);

	bool get_distance_measurement(VL53L1X_Result_t *pointer_to_results);


}; // END OF CLASS DEFINITION





#endif // VL53L1X_CPP_INTERFACE_H
