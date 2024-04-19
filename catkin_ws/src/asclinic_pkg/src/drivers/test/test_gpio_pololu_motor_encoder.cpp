#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include <gpiod.h>

int main()
{
	// Choose the I2C device.
	//const char * i2c_device_name = "/dev/i2c-1";

//	::gpiod::chip chip("gpiochip1");
//	auto lines = chip.get_lines({ 161 });
//	lines.request({ "foobar", ::gpiod::line_request::EVENT_BOTH_EDGES, 0});
//
//	for (;;)
//	{
//		auto events = lines.event_wait(::std::chrono::nanoseconds(1000000000));
//		if (events)
//		{
//			for (auto&it: events)
//			{
//				print_event(it.event_read());
//			}
//		}
//	}

	// int line_number = 148;
	int line_number = 105;
	// int line_number = 130;
	const char * gpio_chip_name = "/dev/gpiochip1";

	int num_events_to_display = 20;

	struct timespec ts = { 0, 1000000 };
	struct gpiod_line_event event;
	struct gpiod_chip *chip;
	struct gpiod_line *line;
	int rv, value;

	time_t prev_tv_sec    = -1;
	long int prev_tv_nsec = -1;
	bool prev_time_isValid = false;

	printf("event variable initialised as: %d timestamp: [%8ld.%09ld]\n", event.event_type, event.ts.tv_sec, event.ts.tv_nsec);

	value = gpiod_ctxless_get_value("/dev/gpiochip1", line_number, false, "foobar");

	printf("value = %d\n", value);

	chip = gpiod_chip_open(gpio_chip_name);
	
	line = gpiod_chip_get_line(chip,line_number);

	printf("Chip %s opened and line %d retrieved\n", gpio_chip_name, line_number);

	//gpiod_line_request_rising_edge_events(line, "foobar");
	gpiod_line_request_both_edges_events(line, "foobar");

	for (int i_event=0; i_event<num_events_to_display; i_event++)
	{
		/**
		* @brief Wait for an event on a single line.
		* @param line GPIO line object.
		* @param timeout Wait time limit.
		* @return 0 if wait timed out, -1 if an error occurred, 1 if an event
		*         occurred.
		*/
		do {
			rv = gpiod_line_event_wait(line,&ts);
		}
		while (rv <= 0);

		/**
 		* @brief Read the last event from the GPIO line.
		* @param line GPIO line object.
		* @param event Buffer to which the event data will be copied.
		* @return 0 if the event was read correctly, -1 on error.
		* @note This function will block if no event was queued for this line.
 		*/
		rv = gpiod_line_event_read(line,&event);

		if (!rv)
		{
			//printf("event: %d timestamp: [%8ld.%09ld]\n", event.event_type, event.ts.tv_sec, event.ts.tv_nsec);
			
			if (prev_time_isValid)
			{
				// Compute the time difference in seconds
				float this_diff_sec = float(event.ts.tv_nsec - prev_tv_nsec) / 1000000000;
				if (event.ts.tv_sec > prev_tv_sec)
				{
					this_diff_sec = this_diff_sec + (event.ts.tv_sec - prev_tv_sec);
				}

				// Convert the difference in nano seconds to a frequency in RPM
				// > Note: for the conversion to RPM couting a single edge: 1/(16*50) * 60[sec/min] = 0.075
				// > Note: for the conversion to RPM counting both edges:   1/(32*50) * 60[sec/min] = 0.0375
				float this_freq_in_rpm = 1.0 / this_diff_sec * 0.0375;

				// Print out the speed
				printf("event type = %4d, time delta (sec) = %11.9f, RPM = %7.3f\n", event.event_type, this_diff_sec, this_freq_in_rpm);
			}

			// Update the previous time
			prev_tv_sec  = event.ts.tv_sec;
			prev_tv_nsec = event.ts.tv_nsec;
			prev_time_isValid = true;
		}
	}
	
	gpiod_chip_close(chip);

}
