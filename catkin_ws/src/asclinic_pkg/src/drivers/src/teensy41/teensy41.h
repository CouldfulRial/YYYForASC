#ifndef TEENSY41_H
#define TEENSY41_H

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>


class Teensy41
{

private:
	// private variables
	static const int TEENSY_MAX_DEVICE_NAME_LENGTH = 20;
	static const int TEENSY_RETURNED_BUFFER_SIZE = 50;
	const char TEENSY_REQUEST_DISPLACEMENT_BYTE = '*';
	const char TEENSY_REQUEST_SPEED_BYTE = 's';
	const char TEENSY_REQUEST_DISPLACEMENT_WITH_TIMESTAMP_BYTE = 't';
	char device_name[TEENSY_MAX_DEVICE_NAME_LENGTH];
	int file_descriptor;
	char returned_buffer[TEENSY_RETURNED_BUFFER_SIZE];

private:
	// private functions
	void open_device();

public:
	// public constructors
	Teensy41();
	Teensy41(const char *device_name);

public:
	// public functions
	const char *get_device_name();
	bool close_device();
	bool get_motors_displacement(int *motor1, int *motor2);
	bool get_motors_displacement_with_timestamp(int *disp1, int *disp2, unsigned int *timestamp);
	bool get_motors_speed_rpm(float *speed1, float *speed2, int delta_t_in_us);
	bool get_motors_speed_rpm_directly_from_teensy(float *speed1, float *speed2);
};

#endif // TEENSY41_H
