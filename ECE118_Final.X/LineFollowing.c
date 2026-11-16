#include "LineFollowing.h"
#include "Motor.h"

typedef enum {
    DRIVE_FORWARD,
    ADJUST,
    TURN_RIGHT
} LineState;

static LineState lineState = DRIVE_FORWARD;

void LineFollowing(unsigned char onTape[6]) {
    unsigned char frontMiddle = onTape[1];

    // Turn right if either outer front sensor detects tape
    if (onTape[0] || onTape[2]) {
        lineState = TURN_RIGHT;
    } else if (!frontMiddle) {
        // Drive forward if no front tape is detected
        lineState = DRIVE_FORWARD;
    } else {
        // Adjust direction when only the front middle detects tape
        lineState = ADJUST;
    }

    switch (lineState) {
        case DRIVE_FORWARD:
            Motor_Forward(DRIVE_PWM);
            break;

        case ADJUST:
            // Use rear sensors to adjust direction
            Motor_Direction(FORWARD);

            if (onTape[3] && !onTape[5]) {
                Motor_SetLeftSpeed(0);
                Motor_SetRightSpeed(SLOW_PWM);
            } else if (onTape[5] && !onTape[3]) {
                Motor_SetLeftSpeed(SLOW_PWM);
                Motor_SetRightSpeed(0);
            } else {
                // Move slowly if neither rear sensor gives a direction
                Motor_Forward(SLOW_PWM);
            }
            break;

        case TURN_RIGHT:
            // Pivot right while an outer front sensor detects tape
            Motor_TurnRight(SLOW_PWM);
            break;
    }
}
