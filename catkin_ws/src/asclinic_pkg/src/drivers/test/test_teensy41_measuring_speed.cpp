#include <stdio.h>

#include <thread>

#include "i2c_driver/i2c_driver.h"
#include "pololu_smc_g2/pololu_smc_g2.h"
#include "teensy41/teensy41.h"

typedef struct
{
	int duration_micro_seconds;
	int mode; // 0 is for rotation; 1 is for driving forward;
	int speed;
} test_config_t;

void open_i2c_device(I2C_Driver *i2c_driver)
{
	bool openSuccess = i2c_driver->open_i2c_device();
	if (!openSuccess)
	{
		printf("FAILED to open I2C device.\n\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		printf("I2C Device successfully opened with file descriptor = %d\n\n",
			i2c_driver->get_file_descriptor());
	}
}

void close_i2c_device(I2C_Driver *i2c_driver)
{
	bool closeSuccess = i2c_driver->close_i2c_device();
	if (!closeSuccess)
	{
		printf("FAILED to close I2C device.\n\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		printf("I2C Device successfully closed.\n\n");
	}
}

bool setup_and_check_pololu_status(Pololu_SMC_G2 *pololu_smc)
{
	const int new_current_limit_in_milliamps = 5000;
	const int new_max_speed_limit = 3200;
	const int new_max_accel_limit = 5;
	const int new_max_decel_limit = 5;
	const bool verbose = true;
	return pololu_smc->initialise_with_limits(new_current_limit_in_milliamps,
		new_max_speed_limit, new_max_accel_limit, new_max_decel_limit, verbose);
}

void drive_one_wheel(Pololu_SMC_G2 *pololu_smc, int speed)
{
	if (pololu_smc->set_motor_target_duty_cycle_percent(speed))
	{
		printf("Pololu SMC - motor percent set to: %d, for I2C address %d\n",
			speed, pololu_smc->get_i2c_address());
	}
	else
	{
		printf("FAILED - Pololu SMC - set motor percent NOT successful for I2C "
			   "address %d\n",
			pololu_smc->get_i2c_address());
	}
}

void stop_one_wheel(Pololu_SMC_G2 *pololu_smc)
{
	if (pololu_smc->stop_motor())
	{
		printf("Pololu SMC - stop motor command sent to I2C address %d\n",
			pololu_smc->get_i2c_address());
	}
	else
	{
		printf(
			"FAILED - Pololu SMC - stop motor command NOT successful for I2C "
			"address %d\n",
			pololu_smc->get_i2c_address());
	}
	printf("\n");
}

void drive_wheels(Pololu_SMC_G2 *pololu_smc1, Pololu_SMC_G2 *pololu_smc2,
	test_config_t *test_config)
{
	// const int speed = 8;
	// const int delta_t_micro_seconds = 3700000;
	// const int speed = 20;
	// const int delta_t_micro_seconds = 1380000;
	// const int speed = 30;
	// const int delta_t_micro_seconds = 900000;
	int speed1 = test_config->speed;
	int speed2 = test_config->mode == 0 ? test_config->speed : -(test_config->speed);
	drive_one_wheel(pololu_smc1, speed1);
	drive_one_wheel(pololu_smc2, speed2);
	usleep(test_config->duration_micro_seconds);
	stop_one_wheel(pololu_smc1);
	stop_one_wheel(pololu_smc2);
}

void listen_to_teensy(Teensy41 *teensy, bool *finished_testing)
{
	float speed1, speed2;
	while (!*finished_testing)
	{
		//if (teensy->get_motors_speed_rpm(&speed1, &speed2, 100000))
		if (teensy->get_motors_speed_rpm_directly_from_teensy(&speed1, &speed2))
		{
			printf("Motor1's speed = %.3f\n", speed1);
			printf("Motor2's speed = %.3f\n\n", speed2);
		}
		else
		{
			printf("ERROR!!!\n\n");
		}
		usleep(100000);
	}
}

int main(int argc, char *argv[])
{
	const char *i2c_device_name = "/dev/i2c-1";
	const uint8_t pololu_smc_address1 = 14;
	const uint8_t pololu_smc_address2 = 13;
	bool finished_testing = false;
	test_config_t test_config;
	test_config.duration_micro_seconds = 60000000;
	test_config.mode = 0;
	test_config.speed = 80;

	I2C_Driver i2c_driver(i2c_device_name);
	printf("Now opening i2c device with name = %s\n",
		i2c_driver.get_device_name());

	open_i2c_device(&i2c_driver);

	Pololu_SMC_G2 pololu_smc1(&i2c_driver, pololu_smc_address1);
	Pololu_SMC_G2 pololu_smc2(&i2c_driver, pololu_smc_address2);

	setup_and_check_pololu_status(&pololu_smc1);
	setup_and_check_pololu_status(&pololu_smc2);

	Teensy41 teensy = Teensy41();
	printf("Device name: %s\n", teensy.get_device_name());

	std::thread listener_thread(listen_to_teensy, &teensy, &finished_testing);
	std::thread actuator_thread(drive_wheels, &pololu_smc1, &pololu_smc2, &test_config);

	actuator_thread.join();
	finished_testing = true;
	listener_thread.join();

	close_i2c_device(&i2c_driver);
	return 0;
}
