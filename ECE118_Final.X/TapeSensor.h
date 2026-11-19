/*
 * Defines the 7 tape sensor inputs and functions.
 */

#ifndef TAPE_SENSOR_H
#define TAPE_SENSOR_H

#include <BOARD.h>
#include <IO_Ports.h>

#define FRONT_LEFT        PORTW08_BIT
#define FRONT_LEFT_TRIS   PORTW08_TRIS
#define FRONT_CENTER      PORTW06_BIT
#define FRONT_CENTER_TRIS PORTW06_TRIS
#define FRONT_RIGHT       PORTW04_BIT
#define FRONT_RIGHT_TRIS  PORTW04_TRIS
#define REAR_LEFT         PORTY12_BIT
#define REAR_LEFT_TRIS    PORTY12_TRIS
#define REAR_CENTER       PORTY11_BIT
#define REAR_CENTER_TRIS  PORTY11_TRIS
#define REAR_RIGHT        PORTY10_BIT
#define REAR_RIGHT_TRIS   PORTY10_TRIS
#define TOP               PORTY03_BIT
#define TOP_TRIS          PORTY03_TRIS

// Initialize sensor inputs
char TapeSensor_Init(void);

// Read the six floor sensors
char TapeSensor_ReadValues(unsigned int readings[6]);

// Check each sensor for black tape
char TapeSensor_CheckTape(unsigned int readings[6], unsigned char onTape[6]);

// Check top sensor: 1 = banner, 0 = no banner
unsigned char TapeSensor_CheckBanner(void);

#endif