#include "ISensorDistance.h"

#include <stdio.h>

#include "lib/VL53L1X/VL53L1X_api.h"

SensorDistance::SensorDistance(uint16_t address)
{
	this->address = address;
	this->initialized = false;
}

bool SensorDistance::init()
{
	uint16_t stat;
	VL53L1_RdWord(address, VL53L1_IDENTIFICATION__MODEL_ID, &stat);
	if(stat != 0xEACC)
	{
		fprintf(stderr, "[VL53L1X] Invalid device signature got 0x%4x, expected 0xEACC\n", stat);
		return false;
	}

	//Soft resetting just to be sure
	printf("[VL53L1X] Soft resetting sensor at addr 0x%2x\n", address);
	int reval = VL53L1_WrByte(address, SOFT_RESET, 0x00);
	VL53L1_WaitMs(address, 100);
	reval = VL53L1_WrByte(address, SOFT_RESET, 0x01);
	VL53L1_WaitMs(address, 1000);

	uint8_t status;
	VL53L1X_BootState(address, &status);
	if(status & 0x1)
		printf("[VL53L1X] Connected to laser ranger!\n");
	else
		fprintf(stderr, "[VL53L1X] Laser sensor not up...\n");
	

	/*
	//Set communication voltage to 2v8
	uint8_t m2v8 = 0;
	VL53L1_RdByte(address, PAD_I2C_HV__EXTSUP_CONFIG, &m2v8);
	VL53L1_WrByte(address, PAD_I2C_HV__EXTSUP_CONFIG, m2v8 | 0x01);
*/
	this->initialized = true;
	printf("[VL53L1X] Successfully connected to VL53L1X distance sensor with address 0x%2X.\n", address);
	return true;
}

bool SensorDistance::startRanging()
{
	if(!initialized)
	{
		fprintf(stderr, "[VL53L1X] Sensor not initialized!\n");
		return false;
	}

	//Start ranging
	VL53L1X_ERROR reval = VL53L1X_StartRanging(address);
	if(reval == VL53L1_ERROR_NONE)
	{
		printf("[VL53L1X] Started measurement.\n");
		return true;
	}

	fprintf(stderr, "[VL53L1X] Error starting measurement %4X!\n", reval);
	return false;
}

bool SensorDistance::stopRanging()
{
	if(!initialized)
	{
		fprintf(stderr, "[VL53L1X] Sensor not initialized!\n");
		return false;
	}

	//Stop ranging
	VL53L1X_ERROR reval = VL53L1X_StopRanging(address);
	if(reval == VL53L1_ERROR_NONE)
	{
		printf("[VL53L1X] Stopped measurement.\n");
		return true;
	}

	fprintf(stderr, "[VL53L1X] Error stopping measurement %4X!\n", reval);
	return false;
}

bool SensorDistance::dataReady()
{
	uint8_t dataReady = 0;
	VL53L1X_CheckForDataReady(address, &dataReady);

	return (dataReady & 0x1);
}

int SensorDistance::getMillimeters()
{
	uint16_t distance = 0;

	if(dataReady())
	{
		VL53L1X_GetDistance(address, &distance);
		VL53L1X_ClearInterrupt(address);
		return distance;
	}

	return DS_DATA_NOT_READY;
}

int SensorDistance::waitMillimeters()
{
	while (!dataReady())
		VL53L1_WaitMs(address, 50);
	
	return getMillimeters();
}

SensorDistance::~SensorDistance()
{

}