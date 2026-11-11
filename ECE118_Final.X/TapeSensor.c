/*
 * Initializes and reads the tape sensor inputs
 */

#include <xc.h>
#include <IO_Ports.h>
#include "TapeSensor.h"

char TapeSensor_Init(void) {
    // Set the 7 sensor pins as inputs
    FRONT_LEFT_TRIS = 1;
    FRONT_CENTER_TRIS = 1;
    FRONT_RIGHT_TRIS = 1;
    REAR_LEFT_TRIS = 1;
    REAR_CENTER_TRIS = 1;
    REAR_RIGHT_TRIS = 1;
    TOP_TRIS = 1;

    return SUCCESS;
}

char TapeSensor_ReadValues(unsigned int readings[6]) {
    // Check for an array
    if (readings == 0) {
        return ERROR;
    }

    // Read the 6 ground sensors
    readings[0] = FRONT_LEFT;
    readings[1] = FRONT_CENTER;
    readings[2] = FRONT_RIGHT;
    readings[3] = REAR_LEFT;
    readings[4] = REAR_CENTER;
    readings[5] = REAR_RIGHT;

    return SUCCESS;
}

char TapeSensor_CheckTape(unsigned int readings[6], unsigned char onTape[6]) {
    // Check that both are arrays
    if (readings == 0 || onTape == 0) {
        return ERROR;
    }

    for (unsigned int sensor = 0; sensor < 6; sensor++) {
        if (readings[sensor] == 1) {
            //  HIGH = sensor sees black tape
            onTape[sensor] = 1;
        } else {
            // LOW = sensor sees white floor
            onTape[sensor] = 0;
        }
    }

    return SUCCESS;
}

unsigned char TapeSensor_CheckBanner(void) {
    // Return 1 if HIGH and 0 if LOW
    return TOP;
}
