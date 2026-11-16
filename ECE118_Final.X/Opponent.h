#ifndef OPPONENT_H
#define OPPONENT_H

#include <stdint.h>
#include <IO_Ports.h>

#define BEACON_TRIS  PORTW06_TRIS
#define BEACON_INPUT PORTW06_BIT

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

// Run opponent search, approach, backoff, and shooting
OpponentStatus Opponent_Update(uint32_t now);

#endif
