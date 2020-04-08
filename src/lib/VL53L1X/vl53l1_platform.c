
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

/**
 * @brief Platform dependent implementation for I2C, used by the VL54L1-library to communicate with the sensor. 
 * This file implements I2C on Linux and is tested on a Raspberry Pi
 * 
 * The implementation is heavily inspired by Pimoronis Python library 
 * (https://github.com/pimoroni/vl53l1x-python/blob/505fd6be452f5e9172fa833da1be3b8e10c5b5eb/api/platform/vl53l1_platform.c)
 * 
 * @author Jannled
 */

#include "vl53l1_platform.h"
#include "vl53l1_error_codes.h"

#include <string.h>
#include <time.h>
#include <math.h>
#include <unistd.h>

#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port

// sudo apt install libi2c-dev i2c-tools

#include <i2c/smbus.h>
#include <linux/i2c-dev.h>

VL53L1_Dev_t VL53L1_DEV = {
	address: 0x00,
	i2cFileHandle: -1,
	i2c_mutex: PTHREAD_MUTEX_INITIALIZER
};

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) 
{
	pthread_mutex_lock(&VL53L1_DEV.i2c_mutex);
	__s32 writtenBytes = -1;

	openI2C(dev);
	writtenBytes = i2c_smbus_write_block_data(VL53L1_DEV.i2cFileHandle, index, count, pdata);
	pthread_mutex_unlock(&VL53L1_DEV.i2c_mutex);

	return writtenBytes;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	pthread_mutex_lock(&VL53L1_DEV.i2c_mutex);
	__s32 actualCount = -1;

	// SMBus only allows a maximum of 32 Bytes, since there is no way of specifying the maximum amount of returned values
	// we need to take the maximum size to keep memory in bounds.
	unsigned char readBytes[32] = {0};

	openI2C(dev);
	actualCount = i2c_smbus_read_block_data(VL53L1_DEV.i2cFileHandle, index, readBytes);

	if(actualCount != count)
		fprintf(stderr, "Warning: Requested %d bytes, but got %d!\n", count, actualCount);

	for(int i=0; i < ((count < actualCount) ? count : actualCount); i++)
		pdata[i] = readBytes[i];

	pthread_mutex_unlock(&VL53L1_DEV.i2c_mutex);
	return actualCount;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) 
{	
	pthread_mutex_lock(&VL53L1_DEV.i2c_mutex);
	__s32 writtenBytes = -1;

	openI2C(dev);
	writtenBytes = i2c_smbus_write_byte_data(VL53L1_DEV.i2cFileHandle, index, data);

	pthread_mutex_unlock(&VL53L1_DEV.i2c_mutex);
	return writtenBytes;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) 
{
	uint8_t buf[4];
	buf[1] = data>>0&0xFF;
	buf[0] = data>>8&0xFF;
	return VL53L1_WriteMulti(dev, index, buf, 2);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) 
{
	uint8_t buf[4];
	buf[3] = data>>0&0xFF;
	buf[2] = data>>8&0xFF;
	buf[1] = data>>16&0xFF;
	buf[0] = data>>24&0xFF;
	return VL53L1_WriteMulti(dev, index, buf, 4);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) 
{
	int ret = VL53L1_ReadMulti(dev, index, data, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) 
{
	uint8_t buf[2];
	int ret = VL53L1_ReadMulti(dev, index, buf, 2);
	uint16_t tmp = 0;
	tmp |= buf[1]<<0;
	tmp |= buf[0]<<8;
	*data = tmp;

	return ret;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) 
{
	uint8_t buf[4];
	int ret = VL53L1_ReadMulti(dev, index, buf, 4);
	uint32_t tmp = 0;
	tmp |= buf[3]<<0;
	tmp |= buf[2]<<8;
	tmp |= buf[1]<<16;
	tmp |= buf[0]<<24;
	*data = tmp;
	return ret;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
	return usleep(wait_ms * 1000);
}

int8_t openI2C(uint16_t address)
{
	if(VL53L1_DEV.i2cFileHandle < 0)
	{
		if((VL53L1_DEV.i2cFileHandle = open(I2C_SYSPATH, O_RDWR)) < 0)
		{
			perror("Failed to open I2C device file!\n");
			return VL53L1_ERROR_CONTROL_INTERFACE;
		}
	}

	if(VL53L1_DEV.address != address)
	{
		if(ioctl(VL53L1_DEV.i2cFileHandle, I2C_SLAVE, address) < 0)
		{
			perror("Failed to acquire bus access and/or talk to slave!\n");
			return VL53L1_ERROR_CONTROL_INTERFACE;
		}

		VL53L1_DEV.address = address;
	}
}

int8_t closeI2C()
{
	pthread_mutex_lock(&VL53L1_DEV.i2c_mutex);
	close(VL53L1_DEV.i2cFileHandle);
	pthread_mutex_unlock(&VL53L1_DEV.i2c_mutex);
}