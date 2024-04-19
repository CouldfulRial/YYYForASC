#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include <gpiod.h>

#include <iostream>
#include <bitset>
#include <thread>

#include "i2c_driver/i2c_driver.h"
#include "pololu_smc_g2/pololu_smc_g2.h"

typedef struct {
	int total_number_of_events;
	int minimum_missed_events;
} test_report_t;

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
		printf("I2C Device successfully opened with file descriptor = %d\n\n", i2c_driver->get_file_descriptor());
	}
}

void close_i2c_device(I2C_Driver *i2c_driver)
{
	bool closeSuccess = i2c_driver->close_i2c_device();
	if (!closeSuccess)
	{
		printf("FAILED to close I2C device.\n\n" );
		exit(EXIT_FAILURE);
	}
	else
	{
		printf("I2C Device successfully closed.\n\n" );
	}
}

void check_pololu_status(Pololu_SMC_G2 *pololu_smc)
{
	uint16_t error_status;
	printf("Now checking status for Pololu SMC with I2C address %d\n",
			pololu_smc->get_i2c_address());

	// > Send the "exit safe start" command
	if (pololu_smc->exit_safe_start())
	{
		printf("Pololu SMC - exit safe start successful for I2C address %d\n",
				pololu_smc->get_i2c_address());
	}
	else
	{
		printf("FAILED - Pololu SMC - exit safe start NOT successful for I2C address %d\n",
				pololu_smc->get_i2c_address());
	}

	// > Check the status flag registers
	if (pololu_smc->get_error_status(&error_status))
	{
		std::cout << "Pololu SMC - get error status returned: "
				<< std::bitset<16>(error_status)
				<< ", for I2C address "
				<< pololu_smc->get_i2c_address()
				<< "\n";
	}
	else
	{
		printf("FAILED - Pololu SMC - get error status NOT successful for I2C address %d\n",
				pololu_smc->get_i2c_address());
	}

	// > Check the input voltage
	float input_voltage_value;
	if (pololu_smc->get_input_voltage_in_volts(&input_voltage_value))
	{
		printf("Pololu SMC - get input voltage value returned: %f [Volts], for I2C address %d\n",
				input_voltage_value,
				pololu_smc->get_i2c_address());
	}
	else
	{
		printf("FAILED - Pololu SMC - get input voltage value NOT successful for I2C address %d\n",
				pololu_smc->get_i2c_address());
	}
	
	printf("Finished checking status for Pololu SMC with I2C address %d\n\n",
			pololu_smc->get_i2c_address());
}

void drive_one_wheel(Pololu_SMC_G2 *pololu_smc, int speed)
{
	if (pololu_smc->set_motor_target_duty_cycle_percent(speed))
	{
		printf("Pololu SMC - motor percent set to: %d, for I2C address %d\n",
				speed,
				pololu_smc->get_i2c_address());
	}
	else
	{
		printf("FAILED - Pololu SMC - set motor percent NOT successful for I2C address %d\n",
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
		printf("FAILED - Pololu SMC - stop motor command NOT successful for I2C address %d\n",
				pololu_smc->get_i2c_address());
	}
	printf("\n");
}

void drive_wheels(
	Pololu_SMC_G2 *pololu_smc1,
	Pololu_SMC_G2 *pololu_smc2,
	int speed,
	int delta_t_micro_seconds,
	int drive_mode
)
{
	// const int speed = 8;
	// const int delta_t_micro_seconds = 3700000;
	// const int speed = 20;
	// const int delta_t_micro_seconds = 1380000;
	// const int speed = 30;
	// const int delta_t_micro_seconds = 900000;
	int speed1 = speed;
	int speed2 = drive_mode == 0 ? speed : -speed;
	drive_one_wheel(pololu_smc1, speed1);
	drive_one_wheel(pololu_smc2, speed2);
	usleep(delta_t_micro_seconds);
	stop_one_wheel(pololu_smc1);
	stop_one_wheel(pololu_smc2);
}

void setup_gpio_listener(struct gpiod_chip **chip, struct gpiod_line **line, struct gpiod_line_event *event)
{
	const int line_number = 105;
	const char * gpio_chip_name = "/dev/gpiochip1";

	// struct gpiod_line_event event;
	// struct gpiod_chip *chip;
	// struct gpiod_line *line;
	int value;

	printf("[motor encoder]: event variable initialised as: %d timestamp: [%8ld.%09ld]\n",
			event->event_type,
			event->ts.tv_sec,
			event->ts.tv_nsec);

	// TODO: substitute ctxless with something else (ctxless is deprecated)
	value = gpiod_ctxless_get_value("/dev/gpiochip1", line_number, false, "foobar");
	printf("[motor encoder]: value = %d\n", value);
	*chip = gpiod_chip_open(gpio_chip_name);
	*line = gpiod_chip_get_line(*chip,line_number);
	printf("[motor encoder]: Chip %s opened and line %d retrieved\n", gpio_chip_name, line_number);
	gpiod_line_request_both_edges_events(*line, "foobar");

}

void listen_on_gpio(
	struct gpiod_line *line,
	struct gpiod_line_event *event,
	struct timespec *ts,
	bool *finished_testing,
	test_report_t *test_report
)
{
	int rv;
	time_t prev_tv_sec    = -1;
	long int prev_tv_nsec = -1;
	bool prev_time_isValid = false;
	int prev_event_type = -1;
	test_report->total_number_of_events = 0;
	test_report->minimum_missed_events = 0;

	while (1)
	{
		do
		{
			rv = gpiod_line_event_wait(line, ts);
		}
		while (rv <= 0 && !*finished_testing);
		if (*finished_testing) break;

		rv = gpiod_line_event_read(line, event);
		if (!rv)
		{
			if (prev_time_isValid)
			{
				// Compute the time difference in seconds
				float this_diff_sec = float(event->ts.tv_nsec - prev_tv_nsec) / 1000000000;
				if (event->ts.tv_sec > prev_tv_sec)
				{
					this_diff_sec = this_diff_sec + (event->ts.tv_sec - prev_tv_sec);
				}

				// Convert the difference in nano seconds to a frequency in RPM
				// > Note: for the conversion to RPM couting a single edge: 1/(16*50) * 60[sec/min] = 0.075
				// > Note: for the conversion to RPM counting both edges:   1/(32*50) * 60[sec/min] = 0.0375
				float this_freq_in_rpm = 1.0 / this_diff_sec * 0.0375;

				// Print out the speed
				// printf("event type = %4d, time delta (sec) = %11.9f, RPM = %7.3f\n",
				// 		event->event_type,
				// 		this_diff_sec,
				// 		this_freq_in_rpm);
				if (prev_event_type == event->event_type)
				{
					test_report->minimum_missed_events++;
				}
			}

			// Update the previous time
			prev_tv_sec  = event->ts.tv_sec;
			prev_tv_nsec = event->ts.tv_nsec;
			prev_time_isValid = true;
			prev_event_type = event->event_type;
			test_report->total_number_of_events++;
		}
	}
}

void print_test_report(test_report_t *test_report)
{
	printf("[Test results]: Total number of events: %d\n", test_report->total_number_of_events);
	printf("[Test results]: Minimum number of missed events: %d\n", test_report->minimum_missed_events);
	int estimated_number_of_events = test_report->total_number_of_events + test_report->minimum_missed_events;
	double perror = test_report->minimum_missed_events / (double)estimated_number_of_events;
	printf("[Test results]: Approximate percentage error: %.3f%%\n", perror * 100);
	printf("\n");
}

int main(int argc, char *argv[])
{
	const char *i2c_device_name = "/dev/i2c-1";
	const uint8_t pololu_smc_address1 = 14;
	const uint8_t pololu_smc_address2 = 13;
	bool finished_testing = false;
	struct gpiod_line_event event;
	struct gpiod_chip *chip;
	struct gpiod_line *line;
	struct timespec ts = { 0, 1000000 };
	test_report_t test_report;
	int speed = 8;
	int delta_t_micro_seconds = 3700000;
	int test_mode = 0; // 0 is for rotation; 1 is for driving forward;

	if (argc >= 3)
	{
		speed = atoi(argv[1]);
		delta_t_micro_seconds = atoi(argv[2]);
	}
	if (argc == 4)
	{
		test_mode = atoi(argv[3]);
		assert(test_mode == 0 || test_mode == 1);
	}

	I2C_Driver i2c_driver (i2c_device_name);
	printf("Now opening i2c device with name = %s\n", i2c_driver.get_device_name());

	open_i2c_device(&i2c_driver);

	Pololu_SMC_G2 pololu_smc1 (&i2c_driver, pololu_smc_address1);
	Pololu_SMC_G2 pololu_smc2 (&i2c_driver, pololu_smc_address2);

	check_pololu_status(&pololu_smc1);
	check_pololu_status(&pololu_smc2);

	setup_gpio_listener(&chip, &line, &event);

	std::thread listener_thread (listen_on_gpio, line, &event, &ts, &finished_testing, &test_report);
	std::thread actuator_thread (drive_wheels, &pololu_smc1, &pololu_smc2, speed, delta_t_micro_seconds, test_mode);

	actuator_thread.join();
	finished_testing = true;
	listener_thread.join();

	close_i2c_device(&i2c_driver);
	gpiod_chip_close(chip);

	print_test_report(&test_report);

	return 0;
}
