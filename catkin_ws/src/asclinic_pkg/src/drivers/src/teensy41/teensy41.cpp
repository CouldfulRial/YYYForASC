#include "teensy41/teensy41.h"

Teensy41::Teensy41()
{
	strcpy(this->device_name, "/dev/ttyACM0");
	this->open_device();
}

Teensy41::Teensy41(const char *device_name)
{
	if ((int)strlen(device_name) < TEENSY_MAX_DEVICE_NAME_LENGTH)
	{
		strcpy(this->device_name, device_name);
	}
	else
	{
		// Inform the user
		printf("FAILED to initialise driver becuse provided device_name is too long.\n");
		printf("> device_name = %s\n", device_name);
		printf("> strlen(device_name) = %d, MAX_DEVICE_NAME_LENGTH = %d\n",
			(int)strlen(device_name),
			TEENSY_MAX_DEVICE_NAME_LENGTH);
		printf("> Defaulting instead to /dev/ttyACM0\n");
		// Set the device name to a default
		strcpy(this->device_name, "/dev/ttyACM0");
	}
	this->open_device();
}

void Teensy41::open_device()
{
	this->file_descriptor = open(this->device_name, O_RDWR);
	if (this->file_descriptor == -1)
	{
		perror(this->device_name);
	}
}

bool Teensy41::close_device()
{
	if (close(this->file_descriptor) == -1)
	{
		perror(this->device_name);
		return false;
	}
	return true;
}

bool Teensy41::get_motors_displacement(int *motor1, int *motor2)
{
	write(this->file_descriptor, &TEENSY_REQUEST_DISPLACEMENT_BYTE, 1);
	size_t returned_packet_size = read(
		this->file_descriptor,
		this->returned_buffer,
		TEENSY_RETURNED_BUFFER_SIZE
	);
	this->returned_buffer[returned_packet_size] = '\0';
	if (sscanf(this->returned_buffer, "<%d,%d>", motor1, motor2) != 2)
	{
		return false;
	}
	return true;
}

bool Teensy41::get_motors_displacement_with_timestamp(
	int *disp1, int *disp2, unsigned int *timestamp)
{
	write(this->file_descriptor, &TEENSY_REQUEST_DISPLACEMENT_WITH_TIMESTAMP_BYTE, 1);
	size_t returned_packet_size = read(
		this->file_descriptor,
		this->returned_buffer,
		TEENSY_RETURNED_BUFFER_SIZE
	);
	this->returned_buffer[returned_packet_size] = '\0';
	if (sscanf(this->returned_buffer, "{%d,%d,%u}", disp1, disp2, timestamp) != 3)
	{
		return false;
	}
	return true;
}

bool Teensy41::get_motors_speed_rpm(float *speed1, float *speed2, int delta_t_in_us)
{
	int disp1, disp2;
	// The first ``get_motors_displacement`` set the wheel position to 0
	if (!this->get_motors_displacement(&disp1, &disp2))
	{
		return false;
	}
	usleep(delta_t_in_us);
	if (!this->get_motors_displacement(&disp1, &disp2))
	{
		return false;
	}
	// speed = disp1 / 3200.0 / (delta_t_in_us / 60000000);
	*speed1 = ((float)disp1) * 0.0003125 / (delta_t_in_us * 1.667e-8);
	*speed2 = ((float)disp2) * 0.0003125 / (delta_t_in_us * 1.667e-8);
	return true;
}

bool Teensy41::get_motors_speed_rpm_directly_from_teensy(float *speed1, float *speed2)
{
	write(this->file_descriptor, &TEENSY_REQUEST_SPEED_BYTE, 1);
	size_t returned_packet_size = read(
		this->file_descriptor,
		this->returned_buffer,
		TEENSY_RETURNED_BUFFER_SIZE
	);
	this->returned_buffer[returned_packet_size] = '\0';
	if (sscanf(this->returned_buffer, "[%f,%f]", speed1, speed2) != 2)
	{
		return false;
	}
	return true;
}

const char *Teensy41::get_device_name()
{
	return this->device_name;
}
