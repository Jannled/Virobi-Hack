#include <stdio.h>

#include "lib/VL53L1X/VL53L1X_api.h"
#include "lib/joystick.h"

#define RANGING_SENSOR_ADDR 0x29

#define IDENTIFICATION__MODEL_ID 0x010F

int main(int argc, char** argv) 
{
    int reval = VL53L1_WrByte(RANGING_SENSOR_ADDR, SOFT_RESET, 0x00);
    printf("Reval: %d\n", reval);
    VL53L1_WaitMs(RANGING_SENSOR_ADDR, 100);
    reval = VL53L1_WrByte(RANGING_SENSOR_ADDR, SOFT_RESET, 0x01);
    printf("Reval: %d\n", reval);

    VL53L1_WaitMs(RANGING_SENSOR_ADDR, 10000);

    uint16_t stat;
    VL53L1_RdWord(RANGING_SENSOR_ADDR, IDENTIFICATION__MODEL_ID, &stat);
    if (stat != 0xEACC)
        printf("Model ID: %4X\n", stat);

    uint8_t status = 0;
    int retVal = VL53L1X_BootState(RANGING_SENSOR_ADDR, &status);
    if(status == 1)
        printf("Connected to laser ranger!");
    else
        fprintf(stderr, "Ranging sensor is not connected %d\n", retVal);

    //printf("Sensor Init (%d)\n", VL53L1X_SensorInit(RANGING_SENSOR_ADDR));
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