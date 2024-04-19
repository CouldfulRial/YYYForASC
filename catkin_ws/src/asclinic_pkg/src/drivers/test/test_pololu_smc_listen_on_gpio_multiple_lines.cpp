#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include <gpiod.h>

#include <bitset>
#include <fstream>
#include <iostream>
#include <map>
#include <thread>

#include "i2c_driver/i2c_driver.h"
#include "pololu_smc_g2/pololu_smc_g2.h"

typedef struct
{
	int total_number_of_events;
	int minimum_missed_events;
} gpio_line_event_summary_t;

typedef struct
{
	time_t tv_sec;
	long int tv_nsec;
	bool isValid;
	int event_type;
} gpio_line_event_status_t;

typedef struct
{
	float delta_t_sec;
	int event_type;
} gpio_line_event_bulk_log_t;

typedef struct
{
	int duration_micro_seconds;
	int mode; // 0 is for rotation; 1 is for driving forward;
	int speed;
	bool bulk_log;
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

void get_ctxless_values(const char *gpio_chip_name, int *line_numbers)
{
	// TODO: substitute ctxless with something else (ctxless is deprecated)
	int value_motor_l_ch_a = gpiod_ctxless_get_value(
		gpio_chip_name, line_numbers[0], false, "foobar");
	int value_motor_l_ch_b = gpiod_ctxless_get_value(
		gpio_chip_name, line_numbers[1], false, "foobar");
	int value_motor_r_ch_a = gpiod_ctxless_get_value(
		gpio_chip_name, line_numbers[2], false, "foobar");
	int value_motor_r_ch_b = gpiod_ctxless_get_value(
		gpio_chip_name, line_numbers[3], false, "foobar");

	std::cout << "[TEMPLATE ENCODER MONITOR] On startup of node, chip "
			  << gpio_chip_name << " lines {" << line_numbers[0] << ", "
			  << line_numbers[1] << ", " << line_numbers[2] << ", "
			  << line_numbers[3] << "} returned values = {"
			  << value_motor_l_ch_a << ", " << value_motor_l_ch_b << ", "
			  << value_motor_r_ch_a << ", " << value_motor_r_ch_b << "}"
			  << std::endl;
}

void get_gpiod_line_and_add_to_bulk(struct gpiod_chip *chip, int *line_numbers,
	struct gpiod_line_bulk *line_bulk)
{
	struct gpiod_line *line;
	gpiod_line_bulk_init(line_bulk);
	for (int i = 0; i < 4; i++)
	{
		line = gpiod_chip_get_line(chip, line_numbers[i]);
		gpiod_line_bulk_add(line_bulk, line);
	}
}

void setup_gpio_listener(struct gpiod_chip **chip,
	struct gpiod_line_bulk *line_bulk, struct gpiod_line_event *event)
{
	const int line_number_motor_l_ch_a = 105;
	const int line_number_motor_l_ch_b = 106;
	const int line_number_motor_r_ch_a =  84;
	const int line_number_motor_r_ch_b = 130;
	int line_numbers[4] = {line_number_motor_l_ch_a, line_number_motor_l_ch_b,
		line_number_motor_r_ch_a, line_number_motor_r_ch_b};

	const char *gpio_chip_name = "/dev/gpiochip1";

	struct gpiod_line *lines[4];

	get_ctxless_values(gpio_chip_name, line_numbers);

	printf("[motor encoder]: event variable initialised as: %d timestamp: "
		   "[%8ld.%09ld]\n",
		event->event_type, event->ts.tv_sec, event->ts.tv_nsec);

	*chip = gpiod_chip_open(gpio_chip_name);
	get_gpiod_line_and_add_to_bulk(*chip, line_numbers, line_bulk);
	gpiod_line_request_bulk_both_edges_events(line_bulk, "foobar");
}

void event_summaries_init(gpio_line_event_summary_t *event_summaries)
{
	for (int i = 0; i < 4; i++)
	{
		event_summaries[i].total_number_of_events = 0;
		event_summaries[i].minimum_missed_events = 0;
	}
}

void gpio_line_event_status_init(
	gpio_line_event_status_t *event_statuses, int n)
{
	for (int i = 0; i < n; i++)
	{
		event_statuses[i].tv_sec = -1;
		event_statuses[i].tv_nsec = -1;
		event_statuses[i].isValid = false;
		event_statuses[i].event_type = -1;
	}
}

void listen_on_gpio(struct gpiod_line_bulk *line_bulk,
	struct gpiod_line_event *event, struct timespec *ts, bool *finished_testing,
	gpio_line_event_summary_t *event_summaries,
	gpio_line_event_bulk_log_t *
		*all_events, // this can be null if we are not logging all events
	int estimated_number_of_events)
{
	struct gpiod_line_bulk line_bulk_for_events;
	int rv; // returned value for waiting / reading gpiod event
	int num_events_waited;
	struct gpiod_line_event single_event;
	gpio_line_event_status_t event_statuses[4];
	gpio_line_event_status_init(event_statuses, 4);

	std::map<int, int> line_number_to_index = {
		{105, 0}, {106, 1}, {84, 2}, {130, 3}};
	int line_index;

	event_summaries_init(event_summaries);

	while (1)
	{
		do
		{
			rv = gpiod_line_event_wait_bulk(
				line_bulk, ts, &line_bulk_for_events);
		} while (rv <= 0 && !*finished_testing);
		if (*finished_testing)
			break;

		num_events_waited = gpiod_line_bulk_num_lines(&line_bulk_for_events);

		for (int i_line = 0; i_line < num_events_waited; i_line++)
		{
			struct gpiod_line *line_handle =
				gpiod_line_bulk_get_line(&line_bulk_for_events, i_line);
			unsigned int this_line_number = gpiod_line_offset(line_handle);
			rv = gpiod_line_event_read(line_handle, event);
			line_index = line_number_to_index[this_line_number];
			if (!rv)
			{
				gpio_line_event_status_t *prev_event_status =
					event_statuses + line_index;
				if (prev_event_status->isValid)
				{
					// Compute the time difference in seconds
					float this_diff_sec =
						float(event->ts.tv_nsec - prev_event_status->tv_nsec) /
						1000000000;
					if (event->ts.tv_sec > prev_event_status->tv_sec)
					{
						this_diff_sec = this_diff_sec +
							(event->ts.tv_sec - prev_event_status->tv_sec);
					}
					if (prev_event_status->event_type == event->event_type)
					{
						event_summaries[line_index].minimum_missed_events++;
					}
					// printf("[motor encoder]: line number = %4d, event type = "
					// 	   "%4d, time delta(sec) = %11.9f\n ",
					// 	this_line_number, event->event_type, this_diff_sec);
					if (all_events)
					{
						int t_index =
							event_summaries[line_index].total_number_of_events;
						if (t_index >= estimated_number_of_events)
						{
							printf(
								"%d %d\n", t_index, estimated_number_of_events);
						}
						assert(t_index < estimated_number_of_events);
						all_events[line_index][t_index].delta_t_sec =
							this_diff_sec;
						all_events[line_index][t_index].event_type =
							event->event_type;
					}
				}
				prev_event_status->tv_sec = event->ts.tv_sec;
				prev_event_status->tv_nsec = event->ts.tv_nsec;
				prev_event_status->event_type = event->event_type;
				prev_event_status->isValid = true;
				event_summaries[line_index].total_number_of_events++;
			}
		}
	}
}

void write_json_test_report(test_config_t *test_config, gpio_line_event_summary_t *event_summaries, gpio_line_event_bulk_log_t **all_events)
{
	int line_numbers[4] = {105, 106, 84, 130};
	std::ofstream logfile;
	logfile.open("log/logfile.json");
	logfile << "{ ";
	logfile << "\"duration\": " << test_config->duration_micro_seconds << ", ";
	logfile << "\"speed\": " << test_config->speed << ", ";
	logfile << "\"mode\": " << test_config->mode << ", ";
	logfile << "\"results\": ";
	logfile << "[";
	for (int i = 0; i < 4; i++)
	{
		logfile << "{ ";
		logfile << "\"lineNumber\": " << line_numbers[i] << ", ";
		logfile << "\"numberOfEvents\": " << event_summaries[i].total_number_of_events << ", ";
		logfile << "\"minNumberOfMissedEvents\": " << event_summaries[i].minimum_missed_events << ", ";

		logfile << "\"eventTypes\": [";
		logfile << all_events[i][0].event_type;
		for (int j = 1; j < event_summaries[i].total_number_of_events; j++)
		{
			logfile << "," << all_events[i][j].event_type;
		}
		logfile << "], ";

		logfile << "\"deltaT\": [";
		logfile << all_events[i][0].delta_t_sec;
		for (int j = 1; j < event_summaries[i].total_number_of_events; j++)
		{
			logfile << "," << all_events[i][j].delta_t_sec;
		}
		logfile << "]";
		logfile << " }";
		if (i < 3) logfile << ", ";
	}
	logfile << "]";
	logfile << " }\n";
	logfile.close();
}

void print_test_report(gpio_line_event_summary_t *event_summaries,
	gpio_line_event_bulk_log_t **all_events)
{
	int line_numbers[4] = {105, 106, 84, 130};
	for (int i = 0; i < 4; i++)
	{
		printf(
			"[Test results]: Test results for GPIO line %d\n", line_numbers[i]);
		printf("[Test results]: Total number of events: %d\n",
			event_summaries[i].total_number_of_events);
		printf("[Test results]: Minimum number of missed events: %d\n",
			event_summaries[i].minimum_missed_events);
		int estimated_number_of_events =
			event_summaries[i].total_number_of_events +
			event_summaries[i].minimum_missed_events;
		double perror = event_summaries[i].minimum_missed_events /
			(double)estimated_number_of_events;
		printf("[Test results]: Approximate percentage error: %.3f%%\n",
			perror * 100);
		if (all_events && event_summaries[i].total_number_of_events > 0)
		{
			double total_delta_t = 0.0;
			double min_delta_t =
				all_events[i][0].delta_t_sec; // just a big number in case there
											  // is no events
			double max_delta_t = all_events[i][0].delta_t_sec;
			for (int j = 0; j < event_summaries[i].total_number_of_events; j++)
			{
				total_delta_t += all_events[i][j].delta_t_sec;
				if (all_events[i][j].delta_t_sec < min_delta_t)
					min_delta_t = all_events[i][j].delta_t_sec;
				if (all_events[i][j].delta_t_sec > max_delta_t)
					max_delta_t = all_events[i][j].delta_t_sec;
			}
			printf("[Test results]: Average delta t: %.11f\n",
				total_delta_t / event_summaries[i].total_number_of_events);
			printf("[Test results]: Minimum delta t: %.11f\n", min_delta_t);
			printf("[Test results]: Maximum delta t: %.11f\n", max_delta_t);
		}
		printf("\n");
	}
}

void free_event_logs(
	gpio_line_event_bulk_log_t **all_events, int estimated_number_of_edges)
{
	assert(all_events != NULL);
	for (int i = 0; i < 4; i++)
	{
		free(all_events[i]);
	}
	free(all_events);
}

int main(int argc, char *argv[])
{
	const char *i2c_device_name = "/dev/i2c-1";
	const uint8_t pololu_smc_address1 = 14;
	const uint8_t pololu_smc_address2 = 13;
	bool finished_testing = false;
	struct gpiod_line_event event;
	struct gpiod_chip *chip;
	struct gpiod_line_bulk line_bulk; // For listening on multiple lines
	struct timespec ts = {0, 1000000};
	gpio_line_event_summary_t event_summaries[4];
	gpio_line_event_bulk_log_t **all_events = NULL;
	const int leeway_factor = 1.1; // extra amount of memory for the logs
	test_config_t test_config;
	test_config.bulk_log = false;
	test_config.speed = 8;
	test_config.duration_micro_seconds = 3700000;
	test_config.mode = 0;
	int estimated_number_of_edges = 0;

	if (argc >= 3)
	{
		test_config.speed = atoi(argv[1]);
		test_config.duration_micro_seconds = atoi(argv[2]);
	}
	if (argc >= 4)
	{
		// test_mode == 0: rotation
		// test_mode == 1: going forward
		test_config.mode = atoi(argv[3]);
		assert(test_config.mode == 0 || test_config.mode == 1);
	}
	if (argc == 5)
	{
		// are we logging all event information?
		int arg4 = atoi(argv[4]);
		assert(arg4 == 0 || arg4 == 1);
		if (arg4)
			test_config.bulk_log = true;
	}

	if (test_config.bulk_log)
	{
		estimated_number_of_edges =
			1600 * (test_config.speed / 8.0) * (test_config.duration_micro_seconds / 3700000.0);
		estimated_number_of_edges = (int)(estimated_number_of_edges * 1.1);
		printf("[DEBUG]::: speed = %d\n", test_config.speed);
		printf("[DEBUG]::: Duraion in micro seconds = %d\n",
			test_config.duration_micro_seconds);
		printf("[DEBUG]::: Estimated number of edges = %d\n",
			estimated_number_of_edges);
		all_events =
			(gpio_line_event_bulk_log_t **)malloc(4 * sizeof(**all_events));
		assert(all_events);
		for (int i = 0; i < 4; i++)
		{
			all_events[i] = (gpio_line_event_bulk_log_t *)malloc(
				estimated_number_of_edges * sizeof(*all_events[i]));
			assert(all_events[i]);
		}
	}

	I2C_Driver i2c_driver(i2c_device_name);
	printf("Now opening i2c device with name = %s\n",
		i2c_driver.get_device_name());

	open_i2c_device(&i2c_driver);

	Pololu_SMC_G2 pololu_smc1(&i2c_driver, pololu_smc_address1);
	Pololu_SMC_G2 pololu_smc2(&i2c_driver, pololu_smc_address2);

	setup_and_check_pololu_status(&pololu_smc1);
	setup_and_check_pololu_status(&pololu_smc2);

	setup_gpio_listener(&chip, &line_bulk, &event);

	std::thread listener_thread(listen_on_gpio, &line_bulk, &event, &ts,
		&finished_testing, event_summaries, all_events,
		estimated_number_of_edges);
	usleep(500000); // just to make sure everything is setup
	std::thread actuator_thread(drive_wheels, &pololu_smc1, &pololu_smc2, &test_config);

	actuator_thread.join();
	finished_testing = true;
	listener_thread.join();

	close_i2c_device(&i2c_driver);
	gpiod_chip_close(chip);

	print_test_report(event_summaries, all_events);
	if (test_config.bulk_log) write_json_test_report(&test_config, event_summaries, all_events);

	if (test_config.bulk_log)
	{
		free_event_logs(all_events, estimated_number_of_edges);
	}
	all_events = NULL;

	return 0;
}
