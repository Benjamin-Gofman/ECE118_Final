#include "LineFollowing.h"
#include "Motor.h"

static unsigned char tapeFound = 0u;

/* Sets direction and motor speeds */
static void SetMotorSpeeds(char reverse, int leftPwm, int rightPwm) {
    // Set the 3 parameters
    Motor_Direction(reverse ? REVERSE : FORWARD);
    Motor_SetLeftSpeed(leftPwm);
    Motor_SetRightSpeed(rightPwm);
}

// Called by Navigation.c

void Set_TapeFound(unsigned char found) {
    tapeFound = found;
}

unsigned char LineFollowing(const unsigned char onTape[6],
        unsigned char reverse) {
    // Forward = FL, Backward = RL
    unsigned char leftOnTape = onTape[reverse ? 3 : 0];
    // Forward = FM, Backward = RM
    unsigned char middleOnTape = onTape[reverse ? 4 : 1];
    // Forward = FR, Backward = RR
    unsigned char rightOnTape = onTape[reverse ? 5 : 2];

    // Stop before reversing
    if (rightOnTape) {
        Motor_Stop();
        return 1u;
    }

    // Adjust the robot's position to stay between FL and FM (RL and RM in reverse)
    if (leftOnTape || middleOnTape) {
        tapeFound = 1u;
        SetMotorSpeeds(reverse, SLOW_PWM, SNAIL_PWM);
        return 1u;
    }

    if (!tapeFound) {
        // Drive straight
        SetMotorSpeeds(reverse, DRIVE_PWM, DRIVE_PWM);
        return 0u;
    }

    // Curve left to rediscover the tape
    SetMotorSpeeds(reverse, SNAIL_PWM, SLOW_PWM);
    return 0u;
}
