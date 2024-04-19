#include <stdio.h>
#include <unistd.h>

#include "teensy41/teensy41.h"

int main(int argc, char *argv[])
{
	Teensy41 teensy = Teensy41();
	printf("Device name: %s\n", teensy.get_device_name());
	teensy.close_device();
	return 0;
}
