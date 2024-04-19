#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include "i2c_driver/i2c_driver.h"
//#include "pololu_smc_g2.h"

int main()
{
	// Choose the I2C device.
	const char * i2c_device_name = "/dev/i2c-1";

	// Initialise an driver for the I2C device
	I2C_Driver i2c_driver (i2c_device_name);

	// Initialise a object for each of the Pololu
	// simple motor controllers
	// const uint8_t pololu_smc_address1 = 70;
	// Pololu_SMC_G2 pololu_smc1 = Pololu_SMC_G2::Pololu_SMC_G2(i2c_driver, uint8_t pololu_smc_address1);
	// const uint8_t pololu_smc_address2 = 13;
	// Pololu_SMC_G2 pololu_smc2 = Pololu_SMC_G2::Pololu_SMC_G2(i2c_driver, uint8_t pololu_smc_address2);

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

	

	// int result = smc_exit_safe_start(fd, address);
	// if (result) { return 1; }

	// uint16_t error_status;
	// result = smc_get_error_status(fd, address, &error_status);
	// if (result) { return 1; }
	// printf("Error status: 0x%04x\n", error_status);

	// uint16_t up_time_low;
	// result = smc_get_up_time_low(fd, address, &up_time_low);
	// if (result) { return 1; }
	// uint16_t up_time_high;
	// result = smc_get_up_time_high(fd, address, &up_time_high);
	// if (result) { return 1; }
	// printf("Up time (high , low) is (%d , %d)\n", up_time_high, up_time_low);

	// int16_t target_speed;
	// result = smc_get_target_speed(fd, address, &target_speed);
	// if (result) { return 1; }
	// printf("Target speed is %d.\n", target_speed);

	// int16_t temperature_a;
	// result = smc_get_temperature_a(fd, address, &temperature_a);
	// if (result) { return 1; }
	// int16_t temperature_b;
	// result = smc_get_temperature_b(fd, address, &temperature_b);
	// if (result) { return 1; }
	// printf("Temperature A is %f.\n", float(temperature_a)/10.0);
	// printf("Temperature B is %f.\n", float(temperature_b)/10.0);

	//int16_t new_speed = (target_speed <= 0) ? 3200 : -3200;
	//printf("Setting target speed to %d.\n", new_speed);
	//result = smc_set_target_speed(fd, address, new_speed);
	//if (result) { return 1; }

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