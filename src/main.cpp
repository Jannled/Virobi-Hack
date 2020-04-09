#include <stdio.h>

#include "lib/VL53L1X/VL53L1X_api.h"
#include "lib/joystick.h"

#define RANGING_SENSOR_ADDR 0x29

int main(int argc, char** argv) 
{
	int reval = VL53L1_WrByte(RANGING_SENSOR_ADDR, SOFT_RESET, 0x00);
	VL53L1_WaitMs(RANGING_SENSOR_ADDR, 100);
	reval = VL53L1_WrByte(RANGING_SENSOR_ADDR, SOFT_RESET, 0x01);
	
	VL53L1_WaitMs(RANGING_SENSOR_ADDR, 1000);

	uint16_t stat;
	VL53L1_RdWord(RANGING_SENSOR_ADDR, VL53L1_IDENTIFICATION__MODEL_ID, &stat);
	printf("Model ID: 0x%4X %s\n", stat, stat==0xEACC ? "Check!" : "Not the Laser Ranger!");

	uint8_t status = 0;
	int retVal = VL53L1X_BootState(RANGING_SENSOR_ADDR, &status);
	if(status & 0x1)
		printf("Connected to laser ranger!\n");
	else
		fprintf(stderr, "Ranging sensor is not connected %d 0x%X \n", retVal, status);

	VL53L1_WaitMs(RANGING_SENSOR_ADDR, 1000);
	printf("Initializing Sensor...\n");
	printf("Sensor Init (%d)\n", VL53L1X_SensorInit(RANGING_SENSOR_ADDR));

	//Start ranging
	reval = VL53L1X_StartRanging(RANGING_SENSOR_ADDR);
	if(reval != VL53L1_ERROR_NONE)
		fprintf(stderr, "Error starting measurement");

	uint8_t dataReady = 0;

	for(int i=0; i<300; i++)
	{
		do
		{
			VL53L1_WaitMs(RANGING_SENSOR_ADDR, 50);
			VL53L1X_CheckForDataReady(RANGING_SENSOR_ADDR, &dataReady);
		} while (!(dataReady & 0x1));
		
		uint16_t distance = 0;
		VL53L1X_GetDistance(RANGING_SENSOR_ADDR, &distance);
		printf("%3d Distance: %d\n", distance);
	}

	//Stop Ranging
	reval = VL53L1X_StopRanging(RANGING_SENSOR_ADDR);
}

#if 0
int main(int argc, char *argv[])
{
	const char *device;
	int js;
	struct js_event event;
	struct axis_state axes[3] = {0};
	size_t axis;

	if (argc > 1)
		device = argv[1];
	else
		device = "/dev/input/js0";

	js = open(device, O_RDONLY);

	if (js == -1)
		perror("Could not open joystick");

	/* This loop will exit if the controller is unplugged. */
	while (read_event(js, &event) == 0)
	{
		switch (event.type)
		{
			case JS_EVENT_BUTTON:
				printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
				break;
			case JS_EVENT_AXIS:
				axis = get_axis_state(&event, axes);
				if (axis < 3)
					printf("Axis %zu at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);
				break;
			default:
				/* Ignore init events. */
				break;
		}
		
		fflush(stdout);
	}

	close(js);
	return 0;
}
#endif