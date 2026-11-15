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

    /* The outside FRONT sensors have priority over the center sensors.
     * When either FrontLeft (index 0) or FrontRight (index 2) sees
     * black tape, always pivot RIGHT, regardless of which side triggered.
     * Tape bits: 1 = black tape, 0 = white floor.
     */
    if (onTape[0] || onTape[2]) {
        lineState = TURN_RIGHT;
    } else if (!frontMiddle) {
        /* No front tape detected: continue searching forward. */
        lineState = DRIVE_FORWARD;
    } else {
        /* Only FrontMiddle sees tape: use the existing rear-sensor
         * correction logic to decide how to approach the line.
         */
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
            /* Pivot in place to the RIGHT: left wheel moves forward,
             * right wheel moves backward. Keep turning while either
             * outside front tape sensor reports black.
             */
            Motor_SetLeftDirection(FORWARD);
            Motor_SetRightDirection(REVERSE);
            Motor_SetLeftSpeed(SLOW_PWM);
            Motor_SetRightSpeed(SLOW_PWM);
            break;
    }
}
