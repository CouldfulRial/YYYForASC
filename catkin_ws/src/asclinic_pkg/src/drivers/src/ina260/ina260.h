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

// NOTES TAKEN FROM THE TEXAS INSTRUMENTS DOCUMENTATION
// > This documentation can be downloaded found here:
//   https://www.ti.com/product/INA260
// > The Adafruit guide can be found here:
//   https://learn.adafruit.com/adafruit-ina260-current-voltage-power-sensor-breakout
// > The Adafruit Arduino library can be found here:
//   https://github.com/adafruit/Adafruit_INA260





#ifndef INA260_H
#define INA260_H





#include <fcntl.h>
// #include <linux/i2c.h>
// #include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
// #include <sys/ioctl.h>
#include <unistd.h>

#include "ina260/ina260_constants.h"

#include "i2c_driver/i2c_driver.h"





// ----------------------------------------------------------
//   CCCC  L        A     SSSS   SSSS     DDDD   EEEEE  FFFFF
//  C      L       A A   S      S         D   D  E      F
//  C      L      A   A   SSS    SSS      D   D  EEE    FFF
//  C      L      AAAAA      S      S     D   D  E      F
//   CCCC  LLLLL  A   A  SSSS   SSSS      DDDD   EEEEE  F
// ----------------------------------------------------------

class INA260
{

public:
	// ----------------------------------
	//  EEEEE  N   N  U   U  M   M   SSSS
	//  E      NN  N  U   U  MM MM  S
	//  EEE    N N N  U   U  M M M   SSS
	//  E      N  NN  U   U  M   M      S
	//  EEEEE  N   N   UUU   M   M  SSSS
	// ----------------------------------

	/**
	 * @brief Operating mode options.
	 *
	 * The operating mode is specified as the 3 least significant
	 * bits of the configuration register.
	 * There are 7 possible operating modes, of which 6 modes
	 * select continuous or triggered measurements of the shunt
	 * current and/or bus voltage. The other possible mode is the
	 * power down (or shutdown) mode.
	 *
	 * The 3 bit operating modes are:
	 * 000 = power-down (or shutdown)
	 * 001 = shunt current, triggered
	 * 010 = bus voltage, triggered
	 * 011 = shunt current and bus voltage, triggered
	 * 100 = power-down (or shutdown)
	 * 101 = shunt current, continuous
	 * 110 = bus voltage, continuous
	 * 111 = shunt current and bus voltage, continuous
	 *
	 * On start-up or reset, the default operating mode in the
	 * configuration register is 111, i.e., by default the INA260
	 * performs continuous shunt current and bus voltage
	 * measurements.
	 *
	 * Some further details about the modes:
	 * Power-down mode: this mode minimize the quiescient current
	 * and turns off current into the device inputs. To exit the
	 * power-down mode, set the configuration register to another
	 * operating mode.
	 * Triggered modes: these modes trigger a one-shot measurement
	 * of the current and/or voltage. Each measurement is triggered
	 * by setting the configuration register, even if it is set to
	 * the same operating mode that it was already set to.
	 */
	enum class Operating_Mode : uint16_t
	{
		power_down                  = 0x0000, /**< Power-down (or shutdown) **/
		current_triggered           = 0x0001, /**< Shunt current, triggered **/
		voltage_triggered           = 0x0002, /**< Bus voltage, triggered **/
		current_voltage_triggered   = 0x0003, /**< Shunt current and bus voltage, triggered **/
		power_down_alt              = 0x0004, /**< Power-down (or shutdown) alternative**/
		current_continuous          = 0x0005, /**< Shunt current, continuous **/
		voltage_continuous          = 0x0006, /**< Bus voltage, continuous **/
		current_voltage_continuous  = 0x0007, /**< Shunt current and bus voltage, continuous (default) **/
	};



	/**
	 * @brief Conversion time options.
	 *
	 * The conversation times are sepcified separately for the
	 * shunt current and bus voltage measurements. Both use the
	 * same encoding and hence we declare only one enumeration to
	 * cover both.
	 *
	 * As stated in the data sheet:
	 * > The conversion time settings, along with the programmable
	 *   averaging mode, allow the device to be configured to optimize
	 *   the available timing requirements in a given application.
	 * > The conversion times selected can have an impact on the
	 *   measurement accuracy, with longer conversion time results in
	 *   low bias of the measurement.
	 * > To achieve the highest accuracy measurement possible, use a
	 *   combination of the longest allowable conversion times and
	 *   highest number of averages, based on the timing requirements
	 *   of the system.
	 *
	 * The conversion time for the bus voltage measurement is
	 * stored in bits 8-6 of the configuration register.
	 *
	 * The conversion time for the shunt current measurement is
	 * stored in bits 5-3 of the configuration register.
	 */
	enum class Conversion_Time : uint16_t
	{
		t_0140_us  = 0x0000, /**< Measurement time: 140 micro-seconds */
		t_0204_us  = 0x0001, /**< Measurement time: 204 micro-seconds */
		t_0332_us  = 0x0002, /**< Measurement time: 332 micro-seconds */
		t_0558_us  = 0x0003, /**< Measurement time: 558 micro-seconds */
		t_1100_us  = 0x0004, /**< Measurement time: 1.100 milli-seconds (default) */
		t_2116_us  = 0x0005, /**< Measurement time: 2.116 milli-seconds */
		t_4156_us  = 0x0006, /**< Measurement time: 4.156 milli-seconds */
		t_8244_us  = 0x0007, /**< Measurement time: 8.224 milli-seconds */
	};



	/**
	 * @brief Averaging mode options.
	 *
	 * The averaging modes are sepcified separately for the shunt
	 * current and bus voltage measurements. Both use the same
	 * encoding and hence we declare only one enumeration to
	 * cover both.
	 *
	 * As stated in the data sheet:
	 * > The conversion time settings, along with the programmable
	 *   averaging mode, allow the device to be configured to optimize
	 *   the available timing requirements in a given application.
	 * > The averaging feature can significantly improve the measurement
	 *   accuracy by effectively filtering the signal. This approach
	 *   allows the device to reduce any noise in the measurement that
	 *   may be caused by noise coupling into the signal. A greater
	 *   number of averages enables the device to be more effective in
	 *   reducing the noise component of the measurement.
	 * > To achieve the highest accuracy measurement possible, use a
	 *   combination of the longest allowable conversion times and
	 *   highest number of averages, based on the timing requirements
	 *   of the system.
	 *
	 * The averaging mode is stored in bits 11-9 of the configuration
	 * register.
	 */
	enum class Averaging_Mode : uint16_t
	{
		samples_0001  = 0x0000, /**< Number of averages:    1 (default) */
		samples_0004  = 0x0001, /**< Number of averages:    4 */
		samples_0016  = 0x0002, /**< Number of averages:   16 */
		samples_0064  = 0x0003, /**< Number of averages:   64 */
		samples_0128  = 0x0004, /**< Number of averages:  128 */
		samples_0256  = 0x0005, /**< Number of averages:  256 */
		samples_0512  = 0x0006, /**< Number of averages:  512 */
		samples_1024  = 0x0007, /**< Number of averages: 1024 */
	};



	/**
	 * @brief Alert functions.
	 *
	 * The alert functions specify under which circumstances the
	 * ALERT pin is toggled to indicate an alert. 
	 *
	 * As stated in the data sheet:
	 * > the ALERT pin to be programmed to respond to a single
	 *   user-defined event or to a Conversion Ready notification
	 *   if desired.
	 * > Based on the function being monitored, enter a value into
	 *   the Alert Limit Register to set the corresponding threshold
	 *   value that asserts the ALERT pin.
	 * > Only one of the alert functions can be enabled and monitored
	 *   at a time. If multiple alert functions are enabled, the
	 *   selected function in the highest significant bit position takes
	 *   priority and responds to the Alert Limit Register value.
	 *
	 * The alert functions are stored in bits 15-11 of the
	 * mask/enable register.
	 */
	enum class Alert_Functions : uint16_t
	{
		limits_disabled      = 0x0000, /**< No alerts are raised (default) */
		over_current_limit   = 0x8000, /**< Alert raised when current exceeds limit */
		under_current_limit  = 0x4000, /**< Alert raised when current drops below limit */
		over_voltage_limit   = 0x2000, /**< Alert raised when voltage exceeds limit */
		under_voltage_limit  = 0x1000, /**< Alert raised when voltage drops below limit */
		over_power_limit     = 0x0800, /**< Alert raised when power exceeds limit */
		on_conversion_ready  = 0x0400, /**< Alert raised when a conversion is ready */
		polarity_active_high = 0x0002, /**< Specifies alert polarity is active-high */
		enable_alert_latch   = 0x0001, /**< Specifies that the ALERT pin and Alert Flag bit remains active following a fault until the Mask/Enable Register has been read. */
	};



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
	INA260(I2C_Driver * i2c_driver);
	INA260(I2C_Driver * i2c_driver, uint8_t address);





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
	bool read_register(uint8_t register_address, uint16_t * value);

	bool write_register(uint8_t register_address, uint16_t value);

	bool set_alert_configuration_and_limit(uint16_t limit_value, uint16_t alert_function_bits, bool should_alert_when_conversion_ready, bool should_use_alert_active_high, bool should_enable_alert_latch);

	bool set_alert_current_limit_value_in_amps(float limit_value, uint16_t alert_function_bits, bool should_alert_when_conversion_ready, bool should_use_alert_active_high, bool should_enable_alert_latch);
	bool set_alert_voltage_limit_value_in_volts(float limit_value, uint16_t alert_function_bits, bool should_alert_when_conversion_ready, bool should_use_alert_active_high, bool should_enable_alert_latch);

public:
	// SETTING THE ALERT CONFIGURATION AND ALERT LIMIT REGISTERS
	bool set_alert_over_current_limit_value_in_amps(float limit_value, bool should_alert_when_conversion_ready, bool should_use_alert_active_high, bool should_enable_alert_latch);
	bool set_alert_under_current_limit_value_in_amps(float limit_value, bool should_alert_when_conversion_ready, bool should_use_alert_active_high, bool should_enable_alert_latch);
	bool set_alert_over_voltage_limit_value_in_volts(float limit_value, bool should_alert_when_conversion_ready, bool should_use_alert_active_high, bool should_enable_alert_latch);
	bool set_alert_under_voltage_limit_value_in_volts(float limit_value, bool should_alert_when_conversion_ready, bool should_use_alert_active_high, bool should_enable_alert_latch);
	bool set_alert_over_power_limit_value_in_watts(float limit_value, bool should_alert_when_conversion_ready, bool should_use_alert_active_high, bool should_enable_alert_latch);

	// SETTING THE CONFIGURATION REGISTER
	bool set_configuration(INA260::Operating_Mode op_mode, INA260::Conversion_Time conv_time_current, INA260::Conversion_Time conv_time_voltage, INA260::Averaging_Mode avg_mode);

	// GETTING THE CONFIGURATION REGISTER VALUES
	bool get_configuration(INA260::Operating_Mode * op_mode, INA260::Conversion_Time * conv_time_current, INA260::Conversion_Time * conv_time_voltage, INA260::Averaging_Mode * avg_mode);

	// GETTING THE MEASUREMENT VALUES
	bool get_current_measurement_in_amps(float * value);

	bool get_current_measurement_as_uint16(uint16_t * value);

	bool get_voltage_measurement_in_volts(float * value);

	bool get_power_measurement_in_watts(float * value);

	// GETTING THE ALERT CONFIGURATION
	bool get_alert_configuration_register(uint16_t * value);

	bool get_alert_limit_value_register(uint16_t * value);

	// GETTING THE UNIQUE IDs
	bool get_manufacturer_uid(uint16_t * value);

	bool get_die_and_revision_uid(uint16_t * value);

	// CONVENIENCE FUNCTIONS
	uint16_t conversion_time_enum_to_micro_seconds(INA260::Conversion_Time conv_time_as_emum);

	bool conversion_time_uint16_to_enum(uint16_t conv_time_as_uint16, INA260::Conversion_Time * conv_time_as_emum);

	uint16_t averaging_mode_enum_to_samples(INA260::Averaging_Mode avg_mode_as_emum);

	bool averaging_mode_uint16_to_enum(uint16_t avg_mode_as_uint16, INA260::Averaging_Mode * avg_mode_as_emum);

	bool operating_mode_uint16_to_enum(uint16_t op_mode_as_uint16, INA260::Operating_Mode * op_mode_as_emum);

	float current_uint16_to_float_in_amps(uint16_t current_as_uint16);

	uint16_t current_float_in_amps_to_uint16(float current_as_float);

	float voltage_uint16_to_float_in_volts(uint16_t voltage_as_uint16);

	uint16_t voltage_float_in_volts_to_uint16(float voltage_as_float);

	float power_uint16_to_float_in_watts(uint16_t power_as_uint16);

	uint16_t power_float_in_watts_to_uint16(float power_as_float);

}; // END OF CLASS DEFINITION





#endif // INA260_H
