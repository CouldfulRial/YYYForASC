
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/

#include "vl53l1x/platform/vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>

#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
//#include <string.h>
//#include <iostream>
#include <sys/ioctl.h>
#include <unistd.h>

//#include <sys/types.h>
//#include <sys/stat.h>
#include <arpa/inet.h>




// DECLARE A GLOBAL WRITE ARRAY FOR BUFFERING
// I2C COMMUNICATIONS
#define VL53L1_MAX_I2C_XFER_SIZE 512
static uint8_t vl53l1x_write_buffer[VL53L1_MAX_I2C_XFER_SIZE + 2];



int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t register_address, uint8_t *write_data_array, uint16_t num_write_btyes)
{
	//printf("DEBUGGING: VL53L1_WriteMulti function called with dev = %d\n", dev);

	// Hardcode the address of the VL53L1X for now
	uint8_t i2c_address = 0x29;

	// Convert the "register_address" into its two
	// separate bytes
	// > The first data byte written contains the
	//   most significant eight bits of the register
	//   address
	// > The second data byte written contains the
	//   least significant eight bits of the register
	//   address
	uint8_t reg_addr_byte_msb = register_address >> 8;
	uint8_t reg_addr_byte_lsb = register_address & 0xFF;

	// Put the "register address" into the uint8
	// write array
	vl53l1x_write_buffer[0] = reg_addr_byte_msb;
	vl53l1x_write_buffer[1] = reg_addr_byte_lsb;

	// Copy the values pointed to by "write_data_array"
	// into the uint8 write array
	memcpy(&vl53l1x_write_buffer[2], write_data_array, num_write_btyes);
	// Specify the number of bytes in the array
	uint16_t num_write_bytes_total = num_write_btyes + 2;

	// Create an array of "i2c_msg structs" with:
	// > One message for the data to write
	struct i2c_msg message = { i2c_address, 0, num_write_bytes_total, vl53l1x_write_buffer };

	// Create the struct for using the ioctl interface
	struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };

	// Call the ioctl interface
	int result = ioctl(dev, I2C_RDWR, &ioctl_data);

	// Check the result of the ioctl call
	if (result != 1)
	{
		// Inform the user
		//perror("FAILED result from call to ioctl.\n");
		// Return flag that ioctl was unsuccessful
		return -1;
	}
	// Return flag that ioctl was successful
	//printf("SUCCESS result from call to ioctl.\n");
	return 0;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t register_address, uint8_t *read_data_array, uint16_t num_read_btyes)
{
	//printf("DEBUGGING: VL53L1_ReadMulti function called with dev = %d\n", dev);

	// Hardcode the address of the VL53L1X for now
	uint8_t i2c_address = 0x29;

	// Convert the "register_address" into its two
	// separate bytes
	// > The first data byte written contains the
	//   most significant eight bits of the register
	//   address
	// > The second data byte written contains the
	//   least significant eight bits of the register
	//   address
	uint8_t reg_addr_byte_msb = register_address >> 8;
	uint8_t reg_addr_byte_lsb = register_address & 0xFF;

	// Put the "register address" into the uint8
	// write array
	vl53l1x_write_buffer[0] = reg_addr_byte_msb;
	vl53l1x_write_buffer[1] = reg_addr_byte_lsb;

	// Specify the number of bytes in the array
	uint16_t num_write_bytes_total = 2;

	// Create an array of "i2c_msg structs" with:
	// > First message for the data to write
	// > Second message for reading data
	struct i2c_msg messages[] = {
		{ i2c_address, 0       , num_write_bytes_total, vl53l1x_write_buffer },
		{ i2c_address, I2C_M_RD, num_read_btyes , read_data_array  },
	};

	// Create the struct for using the ioctl interface
	struct i2c_rdwr_ioctl_data ioctl_data = { messages, 2 };

	// Call the ioctl interface
	int result = ioctl(dev, I2C_RDWR, &ioctl_data);

	// Check the result of the ioctl call
	if (result != 2)
	{
		// Inform the user
		//perror("FAILED result from call to ioctl.\n");
		// Return flag that ioctl was unsuccessful
		return -1;
	}
	// Return flag that ioctl was successful
	//printf("SUCCESS result from call to ioctl.\n");
	return 0;
}





// SOME NOTES ABOUT THE FUNCTIONS BELOW:
// > THE FUNCTIONS BELOW ARE CONVENIENCE FUNCTIONS 
//   FOR READING AND WRITING DATA OF LENGTHS 1, 2,
//   OR 4 BYTES.
// > TO IMPLEMENT THE READING AND WRITING, THE
//   FUNCTIONS BELOW CALL THE "WriteMulti" AND
//   "ReadMulti" FUNCTIONS ABOVE.
// > THESE FUNCTIONS BELOW ARE WHAT THE "vl53l1x_api"
//   FUNCTIONS CALL.





// WRITE FUNCTIONS
// WRITE FUNCTIONS
// WRITE FUNCTIONS
// WRITE FUNCTIONS
// WRITE FUNCTIONS

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
	// Call the "Write Multi" for 1 byte of data
	return VL53L1_WriteMulti(dev, index, (uint8_t *) &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
	// Convert the 2 btyes of data from host byte
	// order to network byte order
	data = htons(data);
	// Call the "Write Multi" for 2 bytes of data
	return VL53L1_WriteMulti(dev, index, (uint8_t *) &data, 2);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
	// Convert the 4 btyes of data from host byte
	// order to network byte order
	data = htonl(data);
	// Call the "Write Multi" for 4 bytes of data
	return VL53L1_WriteMulti(dev, index, (uint8_t *) &data, 4);
}



// READ FUNCTIONS
// READ FUNCTIONS
// READ FUNCTIONS
// READ FUNCTIONS
// READ FUNCTIONS

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data)
{
	// Call the "Read Multi" for 1 byte of data
	uint16_t num_read_btyes = 1;
	return VL53L1_ReadMulti(dev, index, data, num_read_btyes);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data)
{
	// Initialise local variables
	int8_t read_status = 0;
	uint16_t data_from_read_multi;
	// Call the "Read Multi" for 2 bytes of data
	uint16_t num_read_btyes = 2;
	read_status = VL53L1_ReadMulti(dev, index, (uint8_t *) &data_from_read_multi, num_read_btyes);
	// Convert the 2 btyes of data from network byte
	// order to host byte order
	uint16_t data_in_host_byte_order = ntohs(data_from_read_multi);
	// Put the data into the return variable
	*data = data_in_host_byte_order;
	// Return the status
	return read_status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data)
{
	// Initialise local variables
	int8_t read_status = 0;
	uint32_t data_from_read_multi;
	// Call the "Read Multi" for 4 bytes of data
	uint16_t num_read_btyes = 4;
	read_status = VL53L1_ReadMulti(dev, index, (uint8_t *) &data_from_read_multi, num_read_btyes);
	// Convert the 4 btyes of data from network byte
	// order to host byte order
	uint16_t data_in_host_byte_order = ntohl(data_from_read_multi);
	// Put the data into the return variable
	*data = data_in_host_byte_order;
	// Return the status
	return read_status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
	//(void)dev;
	usleep(wait_ms * 1000);
	return 0;
}
