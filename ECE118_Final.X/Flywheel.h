#ifndef FLYWHEEL_H
#define FLYWHEEL_H

#include <IO_Ports.h>

#define LEFT_FLYWHEEL_LAT    PORTZ08_LAT
#define LEFT_FLYWHEEL_TRIS   PORTZ08_TRIS
#define RIGHT_FLYWHEEL_LAT   PORTZ10_LAT
#define RIGHT_FLYWHEEL_TRIS  PORTZ10_TRIS

// Initialize the flywheel
void Flywheel_Init(void);

// Start motors
void Flywheel_Start(void);

// Stop motors
void Flywheel_Stop(void);

#endif