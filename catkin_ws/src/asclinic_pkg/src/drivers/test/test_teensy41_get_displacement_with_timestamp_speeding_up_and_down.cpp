#include <assert.h>
#include <math.h>
#include <stdio.h>

#include <chrono>
#include <ctime>
#include <fstream>
#include <functional>
#include <mutex>
#include <thread>
#include <string>

#include "i2c_driver/i2c_driver.h"
#include "pololu_smc_g2/pololu_smc_g2.h"
#include "teensy41/teensy41.h"

typedef struct
{
	int target_distance; // in meters
	int mode; // 0 is for rotation; 1 is for driving forward;
	int speed;
	int polling_delta_t; // also in microseconds
	bool bulk_log;
} test_config_t;

typedef struct
{
	int disp1;
	int disp2;
	unsigned int timestamp;
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

void drive_one_wheel(Pololu_SMC_G2 *pololu_smc, int speed_in_percentage)
{
	int speed = (int)round(3200 * speed_in_percentage / 100.0);
	if (pololu_smc->set_motor_target_duty_cycle_3200(speed))
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
	test_config_t *test_config, bool *finished_testing)
{
	std::chrono::duration<double> sustain_duration (0.78);
	std::chrono::time_point<std::chrono::system_clock> sustain_start_time = std::chrono::system_clock::now();
	// int speed1 = test_config->speed;
	// int speed2 = test_config->mode == 0 ? test_config->speed : -(test_config->speed);
	const int speed_array_length = 10;
	int speeds[10] = { 10, 20, 30, 40, 50, 60, 50, 40, 30, 20 };
	int speed_array_index = 0;
	drive_one_wheel(pololu_smc1, speeds[speed_array_index]);
	drive_one_wheel(pololu_smc2, -speeds[speed_array_index] * 0.978); // compensating for the difference in speed
	while (!*finished_testing) {
		if (std::chrono::system_clock::now() - sustain_start_time > sustain_duration)
		{
			sustain_start_time = std::chrono::system_clock::now();
			if (speed_array_index < speed_array_length - 1)
			{
				speed_array_index += 1;
			}
			drive_one_wheel(pololu_smc1, speeds[speed_array_index]);
			drive_one_wheel(pololu_smc2, -speeds[speed_array_index] * 0.978);
		}
	}
	stop_one_wheel(pololu_smc1);
	stop_one_wheel(pololu_smc2);
}

void listen_to_teensy(Teensy41 *teensy, test_config_t *test_config,
	bool *finished_testing, int *total_number_of_events, std::fstream &logfile)
{
	int disp1, disp2;
	unsigned int timestamp;
	// struct timespec prev_time, curr_time;
	bool first_time = true, second_time = false;
	int target_displacement = test_config->target_distance / (0.1 * 3.1415926535 / 3200.0);
	printf("Target displacement = %d\n", target_displacement);
	int total_disp1 = 0, total_disp2 = 0;
	*total_number_of_events = 0;
	std::mutex mtx;
	bool start_final_countdown = false;
	std::chrono::time_point<std::chrono::system_clock> final_countdown_start_time;
	std::chrono::duration<double> countdown_duration (1);
	while (!start_final_countdown || (std::chrono::system_clock::now() - final_countdown_start_time < countdown_duration))
	{
		if ((abs(total_disp1) >= target_displacement || abs(total_disp2) >= target_displacement) && !start_final_countdown)
		{
			mtx.lock();
			*finished_testing = true;
			mtx.unlock();
			start_final_countdown = true;
			final_countdown_start_time = std::chrono::system_clock::now();
			printf("Total displacement of motor1 when stopping motors = %d\n", total_disp1);
			printf("Total displacement of motor2 when stopping motors = %d\n", total_disp2);
		}
		//if (teensy->get_motors_speed_rpm(&speed1, &speed2, 100000))
		// clock_gettime(CLOCK_REALTIME, &curr_time);
		if (teensy->get_motors_displacement_with_timestamp(&disp1, &disp2, &timestamp))
		{
			if (!first_time && test_config->bulk_log)
			{
				if (second_time)
					logfile << ", ";
				else
					second_time = true;
				logfile << "{ ";
				logfile << "\"displacement1\": " << disp1 << ", ";
				logfile << "\"displacement2\": " << disp2 << ", ";
				logfile << "\"timestamp\": " << timestamp;
				logfile << " }";
				(*total_number_of_events)++;
			}
			if (!test_config->bulk_log)
			{
				printf("Motor1's displacement = %4d\n", disp1);
				printf("Motor2's displacement = %4d\n", disp2);
				printf("Timestamp = %u\n\n", timestamp);
			}
			if (!first_time)
			{
				total_disp1 += disp1;
				total_disp2 += disp2;
			}
			first_time = false;
		}
		else
		{
			printf("ERROR!!!\n\n");
			return;
		}
		// prev_time = curr_time;
		usleep(test_config->polling_delta_t);
	}
	logfile << "]";
	logfile << " }\n";
	printf("Total displacement of motor 1 = %d\n", total_disp1);
	printf("Total displacement of motor 2 = %d\n", total_disp2);
}

void config_test_according_to_args(test_config_t *test_config, int argc, char *argv[])
{
	if (argc >= 2)
	{
		test_config->target_distance = atoi(argv[1]);
	}
	if (argc >= 3)
	{
		// test_mode == 0: rotation
		// test_mode == 1: going forward
		test_config->mode = atoi(argv[2]);
		assert(test_config->mode == 0 || test_config->mode == 1);
	}
	if (argc >= 4)
	{
		test_config->polling_delta_t = atoi(argv[3]);
	}
	if (argc == 5)
	{
		int arg = atoi(argv[4]);
		assert(arg == 0 || arg == 1);
		if (arg)
			test_config->bulk_log = true;
	}
}

std::string generate_log_file_name()
{
	std::time_t now = std::time(0);
	std::tm *ltm = localtime(&now);
	std::string year = std::to_string(1900 + ltm->tm_year);
	std::string month = std::to_string(1 + ltm->tm_mon);
	std::string day = std::to_string(ltm->tm_mday);
	std::string hour = std::to_string(ltm->tm_hour);
	std::string minute = std::to_string(ltm->tm_min);
	std::string second = std::to_string(ltm->tm_sec);
	std::string logfile_base ("log/log_teensy41_get_displacement_with_timestamp_speeding_up_and_down");
	std::string timestamp_sep ("_");
	std::string logfile_format (".json");
	return logfile_base + timestamp_sep + year + timestamp_sep + month + timestamp_sep
		+ day + timestamp_sep + hour + timestamp_sep + minute + timestamp_sep + second
		+ logfile_format;
}

void write_config_to_logfile(std::fstream &logfile, test_config_t *test_config)
{
	logfile << "{ ";
	logfile << "\"targetDistance\": " << test_config->target_distance << ", ";
	logfile << "\"speed\": " << test_config->speed << ", ";
	logfile << "\"mode\": " << test_config->mode << ", ";
	logfile << "\"pollingDeltaT\": " << test_config->polling_delta_t << ", ";
	logfile << "\"results\": ";
	logfile << "[";
}

int main(int argc, char *argv[])
{
	const char *i2c_device_name = "/dev/i2c-1";
	const uint8_t pololu_smc_address1 = 14;
	const uint8_t pololu_smc_address2 = 13;
	bool finished_testing = false;
	test_config_t test_config;
	test_config.target_distance = 1;
	test_config.mode = 0;
	test_config.speed = 80;
	test_config.polling_delta_t = 100000;
	test_config.bulk_log = false;
	int total_number_of_events;
	std::fstream logfile;

	config_test_according_to_args(&test_config, argc, argv);
	// if (test_config.bulk_log)
	if (test_config.bulk_log)
	{
		std::string logfile_name = generate_log_file_name();
		logfile.open(logfile_name, std::fstream::out);
		std::cout << "Opening Log file " << logfile_name << std::endl;
		write_config_to_logfile(logfile, &test_config);
	}

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

	// listen_to_teensy(&teensy, &test_config, &finished_testing, &total_number_of_events, logfile);

	std::thread listener_thread(
		listen_to_teensy, &teensy, &test_config, &finished_testing, &total_number_of_events, std::ref(logfile)
	);
	std::thread actuator_thread(drive_wheels, &pololu_smc1, &pololu_smc2, &test_config, &finished_testing);

	listener_thread.join();
	usleep(1000000);
	finished_testing = true;
	actuator_thread.join();

	teensy.close_device();
	close_i2c_device(&i2c_driver);

	if (logfile.is_open())
	{
		std::cout << "Closing log file" << std::endl;
		logfile.close();
	}
	return 0;
}
