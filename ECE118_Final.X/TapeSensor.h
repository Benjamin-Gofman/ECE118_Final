/*
 * Defines the six tape sensor inputs and functions.
 */

#ifndef TAPE_SENSOR_H
#define TAPE_SENSOR_H

#include <BOARD.h>
#include <IO_Ports.h>

// Read the seven sensor pins
#define FRONT_LEFT        PORTW05_BIT
#define FRONT_LEFT_TRIS   PORTW05_TRIS
#define FRONT_CENTER      PORTW04_BIT
#define FRONT_CENTER_TRIS PORTW04_TRIS
#define FRONT_RIGHT       PORTW03_BIT
#define FRONT_RIGHT_TRIS  PORTW03_TRIS
#define REAR_LEFT         PORTZ03_BIT
#define REAR_LEFT_TRIS    PORTZ03_TRIS
#define REAR_CENTER       PORTZ04_BIT
#define REAR_CENTER_TRIS  PORTZ04_TRIS
#define REAR_RIGHT        PORTZ05_BIT
#define REAR_RIGHT_TRIS   PORTZ05_TRIS
#define TOP               PORTY11_BIT
#define TOP_TRIS          PORTY11_TRIS

// Initialize sensor inputs
char TapeSensor_Init(void);

// Read the six floor sensors
char TapeSensor_ReadValues(unsigned int readings[6]);

// Check each sensor for black tape
char TapeSensor_CheckTape(unsigned int readings[6], unsigned char onTape[6]);

// Check top sensor: 1 = banner, 0 = no banner
unsigned char TapeSensor_CheckBanner(void);

#endif