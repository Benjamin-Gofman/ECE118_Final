#include <xc.h>
#include "Opponent.h"
#include "Motor.h"

void Opponent_Init(void) {
    // Set pin as an input
    BEACON_TRIS = 1;
}

unsigned char OpponentBeaconDetected(void) {
    // Return 1 if HIGH and 0 if LOW
    return BEACON_INPUT;
}

void Opponent_StartSearch(void) {
    // Stop and turn in place
    Motor_Stop();
    Motor_SetLeftDirection(FORWARD);
    Motor_SetRightDirection(REVERSE);
}

OpponentSearchResult SearchOpponent(unsigned int rightTurnDegrees) {
    OpponentSearchResult result;

    // Check for beacon or timeout
    if (OpponentBeaconDetected()) {
        result = OPPONENT_FOUND;
    } else if (rightTurnDegrees >= SCAN_LIMIT_DEG) {
        result = OPPONENT_NOT_FOUND;
    } else {
        // Keep turning 
        Motor_SetLeftSpeed(SCAN_PWM);
        Motor_SetRightSpeed(SCAN_PWM);
        return OPPONENT_SEARCHING;
    }

    // Stop and head for beacon
    Motor_Stop();
    Motor_SetLeftDirection(FORWARD);
    Motor_SetRightDirection(FORWARD);
    return result;
}