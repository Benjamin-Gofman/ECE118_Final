#include "LineFollowing.h"
#include "Motor.h"

void LineFollowing(const unsigned char onTape[6],
                   unsigned char drivingInReverse) {
    if (onTape[0] || onTape[2]) {
        if (drivingInReverse) {
            // Pivot left: physical left wheel back, right wheel forward.
            Motor_SetLeftDirection(REVERSE);
            Motor_SetRightDirection(FORWARD);
            Motor_Speed(SLOW_PWM);
        } else {
            Motor_TurnRight(SLOW_PWM);
        }
        return;
    }

    unsigned int travelLeft = onTape[1] ? SLOW_PWM : DRIVE_PWM;
    unsigned int travelRight = travelLeft;

    if (onTape[1]) {
        if (onTape[3] && !onTape[5]) {
            travelLeft = 0;
        } else if (onTape[5] && !onTape[3]) {
            travelRight = 0;
        }
    }

    if (drivingInReverse) {
        Motor_Direction(REVERSE);
        Motor_SetLeftSpeed(travelRight);
        Motor_SetRightSpeed(travelLeft);
    } else {
        Motor_Direction(FORWARD);
        Motor_SetLeftSpeed(travelLeft);
        Motor_SetRightSpeed(travelRight);
    }
}