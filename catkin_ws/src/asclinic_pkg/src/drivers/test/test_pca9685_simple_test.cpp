#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include <iostream>
#include <bitset>

#include "i2c_driver/i2c_driver.h"
#include "pca9685/pca9685.h"

int main(int argc, char *argv[])
{
	printf("\n\n");
	printf("===================================\n");
	printf("CHECK COMMAND LINE FOR  PULSE WIDTH\n");

	// Set the default pulse width
	uint16_t pulse_width_in_us = 1500;
	// Get the input argument if supplied
	if (argc==2)
	{
		uint16_t pulse_width_arg = atoi( argv[1] );
		if ( (500 < pulse_width_arg) && (pulse_width_arg < 2500) )
		{
			pulse_width_in_us = pulse_width_arg;
		}
	}
	printf("Pulse width = %d us\n", pulse_width_in_us);

	printf("\n\n");
	printf("===================\n");
	printf("OPEN THE I2C DEVICE\n");

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


	// Initialise an object for the PCA9685 PWM Servo Driver
	const uint8_t pca9685_address = 0x40;
	PCA9685 pca9685 (&i2c_driver, pca9685_address);


	// Initialise a boolean for the result of each call
	bool result;


	printf("\n\n");
	printf("==================================\n");
	printf("SET THE MODE 1 and MODE 2 DEFAULTS\n");


	// > Set the mode 1 defaults
	result = pca9685.set_mode1_defaults();
	if (result)
	{
		printf("PCA9685 - set mode 1 defaults successful, for I2C address %d\n", pca9685.get_i2c_address() );
	}
	else
	{
		printf("FAILED - PCA9685 - set mode 1 defaults NOT successful for I2C address %d\n", pca9685.get_i2c_address() );
	}


	// > Set the mode 2 defaults
	result = pca9685.set_mode2_defaults_for_driving_servos();
	if (result)
	{
		printf("PCA9685 - set mode 2 defaults successful, for I2C address %d\n", pca9685.get_i2c_address() );
	}
	else
	{
		printf("FAILED - PCA9685 - set mode 2 defaults NOT successful for I2C address %d\n", pca9685.get_i2c_address() );
	}

	// > Call the wakeup function
	result = pca9685.wakeup();
	if (result)
	{
		printf("PCA9685 - wakeup successful, for I2C address %d\n", pca9685.get_i2c_address() );
	}
	else
	{
		printf("FAILED - PCA9685 - wakeup NOT successful for I2C address %d\n", pca9685.get_i2c_address() );
	}


	printf("\n\n");


	// > Get the current mode 1 configuration
	uint8_t current_mode1;
	result = pca9685.get_mode1(&current_mode1);
	if (result)
	{
		std::cout << "PCA9685 - get mode 1 returned: " << std::bitset<8>(current_mode1) << ", for I2C address " << static_cast<int>(pca9685.get_i2c_address()) << "\n";
		std::cout << "                     expected: " << "00100000" << "\n";
	}
	else
	{
		printf("FAILED - PCA9685 - get mode 1 NOT successful for I2C address %d\n", pca9685.get_i2c_address() );
	}


	// > Get the current mode 2 configuration
	uint8_t current_mode2;
	result = pca9685.get_mode2(&current_mode2);
	if (result)
	{
		std::cout << "PCA9685 - get mode 2 returned: " << std::bitset<8>(current_mode2) << ", for I2C address " << static_cast<int>(pca9685.get_i2c_address()) << "\n";
		std::cout << "                     expected: " << "00000100" << "\n";
	}
	else
	{
		printf("FAILED - PCA9685 - get mode 2 NOT successful for I2C address %d\n", pca9685.get_i2c_address() );
	}


	usleep(10000);
	printf("\n\n");
	printf("=====================\n");
	printf("SET THE PWM FREQUENCY\n");


	// > Set the frequency
	float pwm_freq = 50.0;
	result = pca9685.set_pwm_frequency_in_hz(pwm_freq);
	if (result)
	{
		printf("PCA9685 - set the PWM frequency successfully, for I2C address %d\n", pca9685.get_i2c_address() );
	}
	else
	{
		printf("FAILED - PCA9685 - set the PWM frequency NOT successful for I2C address %d\n", pca9685.get_i2c_address() );
	}

	printf("\n\n");

	// > Get the frequency to double check
	float pwm_freq_retrieved;
	uint8_t pre_scale_retrieved;
	result = pca9685.get_pwm_frequency_in_hz_and_prescale(&pwm_freq_retrieved, &pre_scale_retrieved);
	if (result)
	{
		printf("PCA9685 - get PWM frequency returned:\n> freq = %f\n> pre scale = %d\nfor I2C address %d\n", pwm_freq_retrieved, pre_scale_retrieved, pca9685.get_i2c_address() );
	}
	else
	{
		printf("FAILED - PCA9685 - get PWM frequency NOT successful for I2C address %d\n", pca9685.get_i2c_address() );
	}


	usleep(10000);
	printf("\n\n");
	printf("=========================\n");
	printf("SET CHANNEL 0 TO FULL OFF\n");


	// > Set the frequency
	int channel_to_set = 0;
	result = pca9685.set_pwm_full_on_or_full_off(channel_to_set,false);
	if (result)
	{
		printf("PCA9685 - set channel %d full off successfully, for I2C address %d\n", channel_to_set, pca9685.get_i2c_address() );
	}
	else
	{
		printf("FAILED - PCA9685 - set channel %d full off NOT successful for I2C address %d\n", channel_to_set, pca9685.get_i2c_address() );
	}

	printf("\n\n");

	// > Get the PWM pulse setting
	uint16_t channel_0_on_count;
	uint16_t channel_0_off_count;
	result = pca9685.get_pwm_pulse(channel_to_set,&channel_0_on_count, &channel_0_off_count);
	if (result)
	{
		std::cout << "PCA9685 - get channel " << channel_to_set << " pulse returned:\n";
		std::cout << "channel on  = " << std::bitset<16>(channel_0_on_count) << "\n";
		std::cout << "    expected: " << "0000000000000000" << "\n";
		std::cout << "channel off = " << std::bitset<16>(channel_0_off_count) << "\n";
		std::cout << "    expected: " << "0001000000000000" << "\n";
	}
	else
	{
		printf("FAILED - PCA9685 - get channel %d pulse NOT successful for I2C address %d\n", channel_to_set, pca9685.get_i2c_address() );
	}


	usleep(20000);
	printf("\n\n");
	printf("========================\n");
	printf("SET CHANNEL 0 TO 1500 us\n");


	// > Set the frequency
	//uint16_t pulse_width_in_us = 1500;
	result = pca9685.set_pwm_pulse_in_microseconds(channel_to_set,pulse_width_in_us);
	if (result)
	{
		printf("PCA9685 - set channel %d to %d micro seconds successfully, for I2C address %d\n", channel_to_set, pulse_width_in_us, pca9685.get_i2c_address() );
	}
	else
	{
		printf("FAILED - PCA9685 - set channel %d pulse NOT successful for I2C address %d\n", channel_to_set, pca9685.get_i2c_address() );
	}
	// Convert the pulse width to a count
	float micro_seconds_per_count = (1000000.0 / 4096.0) / pwm_freq;
	uint16_t expected_off_count = static_cast<uint16_t>( static_cast<float>(pulse_width_in_us) / micro_seconds_per_count );

	printf("\n\n");

	// > Get the PWM pulse setting
	//uint16_t channel_0_on_count;
	//uint16_t channel_0_off_count;
	result = pca9685.get_pwm_pulse(channel_to_set,&channel_0_on_count, &channel_0_off_count);
	if (result)
	{
		std::cout << "PCA9685 - get channel " << channel_to_set << " pulse returned:\n";
		std::cout << "channel on  = " << std::bitset<16>(channel_0_on_count) << "\n";
		std::cout << "    expected: " << "0000000000000000" << "\n";
		std::cout << "channel off = " << std::bitset<16>(channel_0_off_count) << "\n";
		std::cout << "    expected: " << std::bitset<16>(expected_off_count)  << "\n";
	}
	else
	{
		printf("FAILED - PCA9685 - get channel %d pulse NOT successful for I2C address %d\n", channel_to_set, pca9685.get_i2c_address() );
	}


	usleep(1000000);
	printf("\n\n");
	printf("============================\n");
	printf("SET ALL CHANNELS TO FULL OFF\n");


	result = pca9685.set_all_channels_full_off();
	if (result)
	{
		printf("PCA9685 - set all channels to full off successfully, for I2C address %d\n", pca9685.get_i2c_address() );
	}
	else
	{
		printf("FAILED - PCA9685 - set all channels to full off NOT successful for I2C address %d\n", pca9685.get_i2c_address() );
	}

	printf("\n\n");

	// > Get the PWM pulse setting
	//uint16_t channel_0_on_count;
	//uint16_t channel_0_off_count;
	result = pca9685.get_pwm_pulse(channel_to_set,&channel_0_on_count, &channel_0_off_count);
	if (result)
	{
		std::cout << "PCA9685 - get channel " << channel_to_set << " pulse returned:\n";
		std::cout << "channel on  = " << std::bitset<16>(channel_0_on_count) << "\n";
		std::cout << "    expected: " << "0000000000000000" << "\n";
		std::cout << "channel off = " << std::bitset<16>(channel_0_off_count) << "\n";
		std::cout << "    expected: " << "0001000000000000" << "\n";
	}
	else
	{
		printf("FAILED - PCA9685 - get channel %d pulse NOT successful for I2C address %d\n", channel_to_set, pca9685.get_i2c_address() );
	}


	usleep(10000);
	printf("\n\n");
	printf("====================\n");
	printf("CLOSE THE i2C DEVICE\n");


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
