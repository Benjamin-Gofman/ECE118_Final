#ifndef OPPONENT_H
#define OPPONENT_H

#include <stdint.h>
#include <IO_Ports.h>

#define BEACON_TRIS  PORTW03_TRIS
#define BEACON_INPUT PORTW03_BIT

#define OPPONENT_SHOT_COUNT 2 // Number of balls released at opponent

typedef enum {
    OPPONENT_RUNNING,
    OPPONENT_FINISHED,
    OPPONENT_FAILED
} OpponentStatus;

// Initialize the opponent sensor
void Opponent_Init(void);

// Start searching for the opponent
void Opponent_StartSearch(uint32_t now);

// Handle the bumper press and start backing off
void Opponent_OnBumperPress(uint32_t now);

// Check if the opponent is detected
unsigned char OpponentBeaconDetected(void);

// Enables or disables beacon detection
void Opponent_EnableBeacon(unsigned char allowed);

// Run opponent search, approach, back off, and shooting
OpponentStatus Opponent_Update(uint32_t now);

// Returns the opponent state for the simulator
const char *Opponent_GetStateName(void);

#endif
