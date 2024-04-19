#include <assert.h>
#include <stdio.h>
#include <time.h>

#include <fstream>
#include <thread>

#include "i2c_driver/i2c_driver.h"
#include "pololu_smc_g2/pololu_smc_g2.h"
#include "teensy41/teensy41.h"

typedef struct
{
	long duration_micro_seconds;
	int mode; // 0 is for rotation; 1 is for driving forward;
	int speed;
	int polling_delta_t; // also in microseconds
	bool bulk_log;
} test_config_t;

typedef struct
{
	int disp1;
	int disp2;
	float delta_t_sec;
}
event_log_t;

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

void listen_to_teensy(Teensy41 *teensy, test_config_t *test_config,
	bool *finished_testing, event_log_t *all_events, int *total_number_of_events)
{
	int disp1, disp2;
	struct timespec prev_time, curr_time;
	bool first_time = true;
	*total_number_of_events = 0;
	while (!*finished_testing)
	{
		//if (teensy->get_motors_speed_rpm(&speed1, &speed2, 100000))
		clock_gettime(CLOCK_REALTIME, &curr_time);
		if (teensy->get_motors_displacement(&disp1, &disp2))
		{
			if (!first_time && all_events)
			{
				all_events[*total_number_of_events].disp1 = disp1;
				all_events[*total_number_of_events].disp2 = disp2;
				// Compute the time difference in seconds
				float this_diff_sec =
					float(curr_time.tv_nsec - prev_time.tv_nsec) /
					1000000000;
				if (curr_time.tv_sec > prev_time.tv_sec)
				{
					this_diff_sec += curr_time.tv_sec - prev_time.tv_sec;
				}
				all_events[*total_number_of_events].delta_t_sec = this_diff_sec;
				(*total_number_of_events)++;
			}
			// printf("Motor1's displacement = %.3d\n", disp1);
			// printf("Motor2's displacement = %.3d\n\n", disp2);
			first_time = false;
		}
		else
		{
			printf("ERROR!!!\n\n");
			return;
		}
		prev_time = curr_time;
		usleep(test_config->polling_delta_t);
	}
}

void config_test_according_to_args(test_config_t *test_config, int argc, char *argv[])
{
	if (argc >= 3)
	{
		test_config->speed = atoi(argv[1]);
		int duration_in_seconds = atoi(argv[2]);
		test_config->duration_micro_seconds = duration_in_seconds * 1000000;
	}
	if (argc >= 4)
	{
		// test_mode == 0: rotation
		// test_mode == 1: going forward
		test_config->mode = atoi(argv[3]);
		assert(test_config->mode == 0 || test_config->mode == 1);
	}
	if (argc >= 5)
	{
		test_config->polling_delta_t = atoi(argv[4]);
	}
	if (argc == 6)
	{
		int arg = atoi(argv[5]);
		assert(arg == 0 || arg == 1);
		if (arg)
			test_config->bulk_log = true;
	}
}

event_log_t *allocate_memory_to_log_data(test_config_t *test_config)
{
	event_log_t *all_events;
	int array_size = (int)(test_config->duration_micro_seconds / (double)test_config->polling_delta_t);
	// printf("Array size = %d\n", array_size);
	all_events = (event_log_t *)malloc(
		array_size * sizeof(*all_events) * 1.1 // Just in case there are extra data
	);
	assert(all_events);
	return all_events;
}

void print_test_results(event_log_t *all_events, int total_number_of_events)
{
	for (int i = 0; i < total_number_of_events; i++)
	{
		printf("disp1 = %4d, disp2 = %4d, delta_t = %.3fs\n",
			all_events[i].disp1,
			all_events[i].disp2,
			all_events[i].delta_t_sec);
	}
}

void write_json_test_report(test_config_t *test_config,
	event_log_t *all_events, int total_number_of_events)
{
	std::ofstream logfile;
	logfile.open("log/logfile_060721.json");
	logfile << "{ ";
	logfile << "\"duration\": " << test_config->duration_micro_seconds << ", ";
	logfile << "\"speed\": " << test_config->speed << ", ";
	logfile << "\"mode\": " << test_config->mode << ", ";
	logfile << "\"pollingDeltaT\": " << test_config->polling_delta_t << ", ";
	logfile << "\"results\": ";
	logfile << "[";
	for (int i = 0; i < total_number_of_events; i++)
	{
		logfile << "{ ";
		logfile << "\"displacement1\": " << all_events[i].disp1 << ", ";
		logfile << "\"displacement2\": " << all_events[i].disp2 << ", ";
		logfile << "\"deltaT\": " << all_events[i].delta_t_sec;
		logfile << " }";
		if (i < total_number_of_events - 1)
			logfile << ", ";
	}
	logfile << "]";
	logfile << " }\n";
	logfile.close();
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
	test_config.polling_delta_t = 100000;
	test_config.bulk_log = false;
	event_log_t *all_events = NULL;
	int total_number_of_events;

	config_test_according_to_args(&test_config, argc, argv);
	if (test_config.bulk_log)
		all_events = allocate_memory_to_log_data(&test_config);

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

	std::thread listener_thread(
		listen_to_teensy, &teensy, &test_config, &finished_testing,
		all_events, &total_number_of_events
	);
	std::thread actuator_thread(drive_wheels, &pololu_smc1, &pololu_smc2, &test_config);

	actuator_thread.join();
	finished_testing = true;
	listener_thread.join();

	teensy.close_device();
	close_i2c_device(&i2c_driver);

	// print_test_results(all_events, total_number_of_events);
	write_json_test_report(&test_config, all_events, total_number_of_events);

	free(all_events);
	return 0;
}
