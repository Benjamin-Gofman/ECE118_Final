#include <xc.h>
#include "Flywheel.h"

void Flywheel_Init(void) {
    // Starts off 
    Flywheel_Stop();
    LEFT_FLYWHEEL_TRIS = 0;
    RIGHT_FLYWHEEL_TRIS = 0;
}

void Flywheel_Start(void) {
    // Start motors
    LEFT_FLYWHEEL_LAT = 1;
    RIGHT_FLYWHEEL_LAT = 1;
}

void Flywheel_Stop(void) {
    // Stop motors
    LEFT_FLYWHEEL_LAT = 0;
    RIGHT_FLYWHEEL_LAT = 0;
}