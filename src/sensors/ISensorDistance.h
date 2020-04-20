#ifndef I_SENSOR_DISTANCE
#define I_SENSOR_DISTANCE

#include "stdint.h"

#define DS_DATA_TIMEOUT -31
#define DS_DATA_NOT_READY -32

#ifndef DISTANCE_SENSOR_TIMEOUT
#define DISTANCE_SENSOR_TIMEOUT 1000
#endif

class SensorDistance
{
	public:
		SensorDistance(uint16_t address);
		virtual ~SensorDistance();

		int getMillimeters();
		int waitMillimeters();
		bool dataReady();

		bool init();
		bool startRanging();
		bool stopRanging();

	private:
		uint16_t address;
		bool initialized;
};

#endif // I_SENSOR_DISTANCE