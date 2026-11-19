#include "LineFollowing.h"
#include "Motor.h"

static signed char tapeTurn = 0; // -1 = left, 1 = right

void LineFollowing_Reset(void) {
    tapeTurn = 0;
}

unsigned char LineFollowing(const unsigned char onTape[6],
        unsigned char drivingInReverse) {
    // If direction = forward, use FL. If direction = backward, use RR.
    unsigned char leftOnTape = onTape[drivingInReverse ? 5 : 0];
    // If direction = forward, use FM. If direction = backward, use RM.
    unsigned char middleOnTape = onTape[drivingInReverse ? 4 : 1];
    // If direction = forward, use FR. If direction = backward, use RL.
    unsigned char rightOnTape = onTape[drivingInReverse ? 3 : 2];

    if (middleOnTape) {
        tapeTurn = 0;
        if (drivingInReverse) {
            Motor_Reverse(SLOW_PWM);
        } else {
            Motor_Forward(SLOW_PWM);
        }
        return 1;
    }

    // Turn if side sensor sees tape and not presently turning
    if (!tapeTurn && (leftOnTape || rightOnTape)) {
        // Turn right going forward and left going backward
        tapeTurn = drivingInReverse ? -1 : 1;
    }

    // Turn right
    if (tapeTurn > 0) {
        Motor_TurnRight(SLOW_PWM);
        return 1;
    }

    // Turn left
    if (tapeTurn < 0) {
        Motor_TurnLeft(SLOW_PWM);
        return 1;
    }

    // Look for tape
    if (drivingInReverse) {
        Motor_Reverse(DRIVE_PWM);
    } else {
        Motor_Forward(DRIVE_PWM);
    }
    return 0;
}
