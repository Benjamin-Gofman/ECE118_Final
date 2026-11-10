/*
 * Defines the six tape sensor inputs and functions.
 */

#ifndef TAPE_SENSOR_H
#define TAPE_SENSOR_H

#include <BOARD.h>
#include <IO_Ports.h>

// Read the seven sensor pins
#define FRONT_LEFT        PORTY04_BIT
#define FRONT_LEFT_TRIS   PORTY04_TRIS
#define FRONT_CENTER      PORTY06_BIT
#define FRONT_CENTER_TRIS PORTY06_TRIS
#define FRONT_RIGHT       PORTY07_BIT
#define FRONT_RIGHT_TRIS  PORTY07_TRIS
#define REAR_LEFT         PORTY08_BIT
#define REAR_LEFT_TRIS    PORTY08_TRIS
#define REAR_CENTER       PORTY09_BIT
#define REAR_CENTER_TRIS  PORTY09_TRIS
#define REAR_RIGHT        PORTY10_BIT
#define REAR_RIGHT_TRIS   PORTY10_TRIS
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