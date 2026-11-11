#ifndef OPPONENT_H
#define OPPONENT_H

#include <IO_Ports.h>

#define BEACON_TRIS  PORTZ04_TRIS
#define BEACON_INPUT PORTZ04_BIT

#define SCAN_LIMIT_DEG  360u  // Max right turn
#define SCAN_PWM        150u  // Wheel speed during the scan

typedef enum {
    OPPONENT_SEARCHING,
    OPPONENT_FOUND,
    OPPONENT_NOT_FOUND
} OpponentSearchResult;

// Initialize the pin
void Opponent_Init(void);
// Detect the opponent
unsigned char OpponentBeaconDetected(void);
// Look for opponent
void Opponent_StartSearch(void);
// Check search result
OpponentSearchResult SearchOpponent(unsigned int rightTurnDegrees);

#endif