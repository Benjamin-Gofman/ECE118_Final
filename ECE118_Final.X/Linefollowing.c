
#include "LineFollowing.h"
#include "Motor.h"

#define DRIVE_PWM 600u // 60% PWM
#define SLOW_PWM  400u // 40% PWM

typedef enum {
    DRIVE_FORWARD,
    ADJUST,
    TURN_RIGHT
} LineState;

static LineState lineState = DRIVE_FORWARD;

void LineFollowing(unsigned char onTape[6]) {
    unsigned char frontMiddle = onTape[1];
    unsigned char backMiddle = onTape[4];

    // Drive forward when the front middle sensor sees white
    if (!frontMiddle) {
        lineState = DRIVE_FORWARD;
    } else if (frontMiddle && backMiddle) {
        // Both middle sensors see black tape
        lineState = DRIVE_FORWARD;
    } else {
        // Adjust so the rear middle sensor sees black
        lineState = ADJUST;
    }

    switch (lineState) {
        case DRIVE_FORWARD:
            Motor_SetLeftDirection(FORWARD);
            Motor_SetRightDirection(FORWARD);
            Motor_SetLeftSpeed(DRIVE_PWM);
            Motor_SetRightSpeed(DRIVE_PWM);
            break;

        case ADJUST:
            // Use the outer rear sensors to pick a direction
            Motor_SetLeftDirection(FORWARD);
            Motor_SetRightDirection(FORWARD);

            if (onTape[3] && !onTape[5]) {
                Motor_SetLeftSpeed(0);
                Motor_SetRightSpeed(SLOW_PWM);
            } else if (onTape[5] && !onTape[3]) {
                Motor_SetLeftSpeed(SLOW_PWM);
                Motor_SetRightSpeed(0);
            } else {
                // Advance slowly when neither side gives a clear direction
                Motor_SetLeftSpeed(SLOW_PWM);
                Motor_SetRightSpeed(SLOW_PWM);
            }
            break;

        case TURN_RIGHT:
            Motor_SetLeftDirection(FORWARD);
            Motor_SetRightDirection(FORWARD);
            Motor_SetLeftSpeed(SLOW_PWM);
            Motor_SetRightSpeed(0);
            break;
    }
}
