#include <stdio.h>
#include <signal.h>

#include "lib/VL53L1X/VL53L1X_api.h"
#include "lib/joystick.h"

#include "sensors/ISensorDistance.h"

#define RANGING_SENSOR_ADDR 0x29

SensorDistance sensorDistance(RANGING_SENSOR_ADDR);

volatile bool running = true;
void requestClose(int arg)
{
	running = false;
}

int main(int argc, char** argv) 
{
	signal(SIGINT, requestClose);

	sensorDistance.init();
	sensorDistance.startRanging();

	while(running)
	{
		VL53L1_WaitMs(RANGING_SENSOR_ADDR, 50);
		
		if(sensorDistance.dataReady())
			printf("Distance: %4.1fcm\n", (float) sensorDistance.getMillimeters());
	}

	sensorDistance.stopRanging();
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