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

// NOTES PROVIDED WITH THE POLOLU SOFWARE
// > The SMC's input mode must be "Serial / I2C / USB".
// > For reliable operation on a Raspberry Pi, enable the i2c-gpio
//   overlay and use the I2C device it provides (usually /dev/i2c-3).
// Pololu Simple Motor Controller (SMC) G2 example taken from:
//   https://www.pololu.com/docs/0J77/8.11





#ifndef POLOLU_SMC_G2_H
#define POLOLU_SMC_G2_H





#include <fcntl.h>
// #include <linux/i2c.h>
// #include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
// #include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>
#include <bitset>
#include <cmath>

#include "pololu_smc_g2/pololu_smc_g2_constants.h"

#include "i2c_driver/i2c_driver.h"





// ----------------------------------------------------------
//   CCCC  L        A     SSSS   SSSS     DDDD   EEEEE  FFFFF
//  C      L       A A   S      S         D   D  E      F
//  C      L      A   A   SSS    SSS      D   D  EEE    FFF
//  C      L      AAAAA      S      S     D   D  E      F
//   CCCC  LLLLL  A   A  SSSS   SSSS      DDDD   EEEEE  F
// ----------------------------------------------------------

class Pololu_SMC_G2
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





	// ----------------------------------------------------------------------------
	//   CCCC   OOO   N   N   SSSS  TTTTT  RRRR   U   U   CCCC  TTTTT   OOO   RRRR
	//  C      O   O  NN  N  S        T    R   R  U   U  C        T    O   O  R   R
	//  C      O   O  N N N   SSS     T    RRRR   U   U  C        T    O   O  RRRR
	//  C      O   O  N  NN      S    T    R   R  U   U  C        T    O   O  R   R
	//   CCCC   OOO   N   N  SSSS     T    R   R   UUU    CCCC    T     OOO   R   R
	// ----------------------------------------------------------------------------
public:
	Pololu_SMC_G2();
	Pololu_SMC_G2(I2C_Driver * i2c_driver);
	Pololu_SMC_G2(I2C_Driver * i2c_driver, uint8_t address);





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





	// ------------------------------------------------------------
	//  FFFFF  U   U  N   N   CCCC  TTTTT  III   OOO   N   N   SSSS
	//  F      U   U  NN  N  C        T     I   O   O  NN  N  S
	//  FFF    U   U  N N N  C        T     I   O   O  N N N   SSS
	//  F      U   U  N  NN  C        T     I   O   O  N  NN      S
	//  F       UUU   N   N   CCCC    T    III   OOO   N   N  SSSS
	// ------------------------------------------------------------

private:
	bool get_variable(uint8_t variable_id, uint16_t * value);

	bool set_motor_limit(uint8_t limit_id, uint16_t value, int * response_code);

public:
	// DIRECT COMMANDS:
	bool exit_safe_start();

	bool motor_forward_3200(int target_duty_cycle);
	bool motor_forward_percent(int target_duty_cycle);
	bool motor_forward_7bit(int target_duty_cycle);

	bool motor_reverse_3200(int target_duty_cycle);
	bool motor_reverse_percent(int target_duty_cycle);
	bool motor_reverse_7bit(int target_duty_cycle);

	bool motor_brake(int brake_amount);

	bool stop_motor();

	// CONVENIENCE FUNCTIONS FOR SETTING
	// A SIGNED MOTOR TARGET DUTY CYCLE
	bool set_motor_target_duty_cycle_3200(int target_duty_cycle);
	bool set_motor_target_duty_cycle_percent(int target_duty_cycle);
	bool set_motor_target_duty_cycle_percent(float target_duty_cycle);
	bool set_motor_target_duty_cycle_7bit(int target_duty_cycle);

	// SET MOTOR LIMITS
	// > For setting forward and reverse limits at the same time
	bool set_motor_limit_max_duty_cycle(int new_max_duty_cycle, int * response_code);
	bool set_motor_limit_max_acceleration(int new_max_acceleration, int * response_code);
	bool set_motor_limit_max_deceleration(int new_max_deceleration, int * response_code);
	bool set_motor_limit_max_brake_duration(int new_max_brake_duration, int * response_code);
	// > For setting forward limits
	bool set_motor_limit_max_duty_cycle_forward(int new_max_duty_cycle, int * response_code);
	bool set_motor_limit_max_acceleration_forward(int new_max_acceleration, int * response_code);
	bool set_motor_limit_max_deceleration_forward(int new_max_deceleration, int * response_code);
	bool set_motor_limit_max_brake_duration_forward(int new_max_brake_duration, int * response_code);
	// > For setting reverse limits
	bool set_motor_limit_max_duty_cycle_reverse(int new_max_duty_cycle, int * response_code);
	bool set_motor_limit_max_acceleration_reverse(int new_max_acceleration, int * response_code);
	bool set_motor_limit_max_deceleration_reverse(int new_max_deceleration, int * response_code);
	bool set_motor_limit_max_brake_duration_reverse(int new_max_brake_duration, int * response_code);

	// SET THE CURRENT LIMIT
	bool set_current_limit_in_internal_units(int new_current_limit);
	bool set_current_limit_in_milliamps(int new_current_limit);

	// GET THE FIRMWARE VERSION
	bool get_firmware_version(uint16_t * product_id, uint8_t * firmware_major , uint8_t * firmware_minor);

	// GET VARIABLES
	// > For the status flag registers
	bool get_error_status(uint16_t * value);
	bool get_error_occurred(uint16_t * value);
	bool get_serial_erros_occurred(uint16_t * value);
	bool get_limit_status(uint16_t * value);
	bool get_reset_flags(uint16_t * value);
	// > For the RC channels
	bool get_rc1_unlimited_raw_value(uint16_t * value);
	bool get_rc1_raw_value(uint16_t * value);
	bool get_rc1_scaled_value(int16_t * value);
	bool get_rc2_unlimited_raw_value(uint16_t * value);
	bool get_rc2_raw_value(uint16_t * value);
	bool get_rc2_scaled_value(int16_t * value);
	// > For the analog channels
	bool get_an1_unlimited_raw_value(uint16_t * value);
	bool get_an1_raw_value(uint16_t * value);
	bool get_an1_scaled_value(int16_t * value);
	bool get_an2_unlimited_raw_value(uint16_t * value);
	bool get_an2_raw_value(uint16_t * value);
	bool get_an2_scaled_value(int16_t * value);
	// > For diagnostic variables
	bool get_target_duty_cycle_3200(int16_t * value);
	bool get_duty_cycle_3200(int16_t * value);
	bool get_brake_amount(uint16_t * value);
	bool get_input_voltage_in_volts(float * value);
	bool get_temperature_a(float * value);
	bool get_temperature_b(float * value);
	bool get_rc_period_in_seconds(float * value);
	bool get_baud_rate_register_in_bps(float * value);
	bool get_up_time_low(uint16_t * value);
	bool get_up_time_high(uint16_t * value);
	bool get_up_time_in_seconds(float * value);
	// > For motor duty cycle limits (forward)
	bool get_max_duty_cycle_forward(uint16_t * value);
	bool get_max_acceleration_forward(uint16_t * value);
	bool get_max_deceleration_forward(uint16_t * value);
	bool get_brake_duration_forward(uint16_t * value);
	bool get_brake_duration_forward_in_seconds(float * value);
	bool get_starting_duty_cycle_forward(uint16_t * value);
	// > For motor duty cycle limits (reverse)
	bool get_max_duty_cycle_reverse(uint16_t * value);
	bool get_max_acceleration_reverse(uint16_t * value);
	bool get_max_deceleration_reverse(uint16_t * value);
	bool get_brake_duration_reverse(uint16_t * value);
	bool get_brake_duration_reverse_in_seconds(float * value);
	bool get_starting_duty_cycle_reverse(uint16_t * value);
	// > For current limiting and measurement
	bool get_current_limit(uint16_t * value);
	bool get_raw_current(uint16_t * value);
	bool get_current_in_milliamps(uint16_t * value);
	bool get_current_limiting_consecutive_count(uint16_t * value);
	bool get_current_limiting_occurrence_count(uint16_t * value);


// Convenience Functions
public:
	bool initialise_with_limits(int new_current_limit_in_milliamps, int new_max_duty_cycle_limit, int new_max_accel_limit, int new_max_decel_limit, bool verbose);

}; // END OF CLASS DEFINITION





#endif // POLOLU_SMC_G2_H
