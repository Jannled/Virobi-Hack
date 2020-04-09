
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
	char buf[] = {index>>8&0xFF, index>>0&0xFF};
	char buf[count +1];
	buf[0] = index;
	for(int i=0; i<count; i++)
		buf[i+1] = pdata[i];

	openI2C(dev);
	
	pthread_mutex_lock(&VL53L1_DEV.i2c_mutex);
	uint32_t retval = write(VL53L1_DEV.i2cFileHandle, buf, count+1);
	pthread_mutex_unlock(&VL53L1_DEV.i2c_mutex);

	if(retval != count+1)
	{
		fprintf(stderr, "Written %ud bytes insted of the requested %ud!", retval-1, count);
		return VL53L1_ERROR_CONTROL_INTERFACE;
	}
	else
		return count;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
	openI2C(dev);

	pthread_mutex_lock(&VL53L1_DEV.i2c_mutex);
	int writeval = write(VL53L1_DEV.i2cFileHandle, &index, 1);
	int readVal = read(VL53L1_DEV.i2cFileHandle, pdata, count);
	pthread_mutex_unlock(&VL53L1_DEV.i2c_mutex);

	if(writeval == 1 && readVal == count)
		return count;
	else
		return VL53L1_ERROR_CONTROL_INTERFACE;	
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) 
{	
	char buf[] = {index, data};

	openI2C(dev);

	pthread_mutex_lock(&VL53L1_DEV.i2c_mutex);
	int retval = write(VL53L1_DEV.i2cFileHandle, buf, 2);
	pthread_mutex_unlock(&VL53L1_DEV.i2c_mutex);

	if(retval != 2)
	{
		fprintf(stderr, "Actually written %d bytes instead of a single one!", retval);
		return VL53L1_ERROR_CONTROL_INTERFACE;
	}
	else
		return 1;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) 
{
	char buf[] = {index, data>>8&0xFF, data>>0&0xFF};

	openI2C(dev);

	pthread_mutex_lock(&VL53L1_DEV.i2c_mutex);
	int retval = write(VL53L1_DEV.i2cFileHandle, buf, 3);
	pthread_mutex_unlock(&VL53L1_DEV.i2c_mutex);

	if(retval != 3)
	{
		fprintf(stderr, "Actually written %d bytes instead of a word!", retval);
		return VL53L1_ERROR_CONTROL_INTERFACE;
	}
	else
		return 2;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) 
{
	char buf[] = {index, data>>24&0xFF, data>>16&0xFF, data>>8&0xFF, data>>0&0xFF};

	openI2C(dev);

	pthread_mutex_lock(&VL53L1_DEV.i2c_mutex);
	int retval = write(VL53L1_DEV.i2cFileHandle, buf, 5);
	pthread_mutex_unlock(&VL53L1_DEV.i2c_mutex);

	if(retval != 5)
	{
		fprintf(stderr, "Actually written %d bytes instead of a dword!", retval);
		return VL53L1_ERROR_CONTROL_INTERFACE;
	}
	else
		return 4;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) 
{
	openI2C(dev);

	unsigned char regAddr = (unsigned char) index;

	pthread_mutex_lock(&VL53L1_DEV.i2c_mutex);
	int writeval = write(VL53L1_DEV.i2cFileHandle, &regAddr, 1);
	int readVal = read(VL53L1_DEV.i2cFileHandle, data, 1);
	pthread_mutex_unlock(&VL53L1_DEV.i2c_mutex);

	if(writeval == 1 && readVal == 1)
		return 1;
	else
		return VL53L1_ERROR_CONTROL_INTERFACE;	
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) 
{
	openI2C(dev);
	uint8_t buf[2];

	pthread_mutex_lock(&VL53L1_DEV.i2c_mutex);
	int writeval = write(VL53L1_DEV.i2cFileHandle, &index, 1);
	int readVal = read(VL53L1_DEV.i2cFileHandle, buf, 2);
	pthread_mutex_unlock(&VL53L1_DEV.i2c_mutex);

	uint16_t tmp = 0;
	tmp |= buf[1]<<0;
	tmp |= buf[0]<<8;
	*data = tmp;

	if(writeval == 1 && readVal == 1)
		return 1;
	else
		return VL53L1_ERROR_CONTROL_INTERFACE;	

}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) 
{
	openI2C(dev);
	uint8_t buf[4];

	pthread_mutex_lock(&VL53L1_DEV.i2c_mutex);
	int writeval = write(VL53L1_DEV.i2cFileHandle, &index, 1);
	int readVal = read(VL53L1_DEV.i2cFileHandle, buf, 2);
	pthread_mutex_unlock(&VL53L1_DEV.i2c_mutex);

	uint32_t tmp = 0;
	tmp |= buf[3]<<0;
	tmp |= buf[2]<<8;
	tmp |= buf[1]<<16;
	tmp |= buf[0]<<24;

	if(writeval == 1 && readVal == 1)
		return 1;
	else
		return VL53L1_ERROR_CONTROL_INTERFACE;	
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
	return usleep(wait_ms * 1000);
}

int8_t openI2C(uint16_t address)
{
	pthread_mutex_lock(&VL53L1_DEV.i2c_mutex);

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

	pthread_mutex_unlock(&VL53L1_DEV.i2c_mutex);
}

int8_t closeI2C()
{
	pthread_mutex_lock(&VL53L1_DEV.i2c_mutex);
	close(VL53L1_DEV.i2cFileHandle);
	pthread_mutex_unlock(&VL53L1_DEV.i2c_mutex);
}